// ------------------------------------------------------------
// Vision/MockCameraManager.cs
// ------------------------------------------------------------
// Software-only ICameraManager for unit tests.
// Generates synthetic stereo frame pairs — two cameras side by side,
// each seeing a ball flying along a configurable straight-line trajectory.
//
// No Spinnaker SDK, no hardware required.
//
// Usage:
//   var mock = new MockCameraManager();
//   mock.SetTrajectory(speedMps: 30f, launchAngleDeg: 15f, azimuthDeg: 0f);
//   var engine = new SimulatorEngine(mock);
// ------------------------------------------------------------
using System;
using System.Collections.Concurrent;
using System.Drawing;
using System.Threading;
using System.Threading.Tasks;
using SportSimulator.Profiles;
using SportSimulator.Vision.Calibration;

namespace SportSimulator.Vision
{
    public class MockCameraManager : ICameraManager
    {
        // ── Camera rig geometry ────────────────────────────────────────────────
        // Must match whatever camera model StereoRectifier/Triangulator are
        // actually configured with (StereoCalibrationData.CreateDefaults(), in the
        // real-camera-absent path SimulatorEngine uses) — referencing its constants
        // directly rather than guessing our own, since a silent mismatch here
        // doesn't throw, it just makes the rectifier's remap sample from the wrong
        // place and wash out real disparity almost entirely. See the comment on
        // StereoCalibrationData.DefaultFx for the incident this was found in.
        private static readonly int   SensorW = StereoCalibrationData.DefaultImageWidth;
        private static readonly int   SensorH = StereoCalibrationData.DefaultImageHeight;
        private static readonly float FocalPx = (float)StereoCalibrationData.DefaultFx;
        private const float  BaselineM      = 0.4953f; // metres between cameras (19.5in)

        // Confirmed on-site 2026-07-22 mounting geometry (notes/14-session-2026-07-22-
        // hardware-bringup.md, "Mounting geometry" line): sensor 114in above the floor,
        // 33in horizontally forward of the tee/ball spot (toward the net, between the
        // ball and the impact screen), no yaw, no lateral offset. That's a real ~16.15°
        // downward tilt off nadir (straight down), not a level camera sitting at the
        // ball's own height — a prior version of this file had a CamHeightM=0 constant
        // that was never even wired into the projection math, silently modeling a level,
        // untilted camera instead. That fabricated a physically-wrong up-then-down pixel
        // reversal for any rising shot (the ball passing above, then back below, the
        // camera's own un-tilted line of sight) that a real downward-tilted camera would
        // never produce — a real shot recedes from a tilted overhead camera in one
        // direction, monotonically, the way a real launch monitor is expected to see it.
        // Modeled below as a pure pitch rotation about the lateral (X) axis, aimed
        // exactly at the tee — the real rig has no yaw or roll per the on-site check.
        // Shared with Triangulator (via StereoCalibrationData) rather than redefined
        // here — a mismatch between what this mock generates and what Triangulator
        // corrects for wouldn't throw, it would just silently reintroduce the
        // wrong-direction bug this same pair of constants was originally added to fix.
        private static readonly float CameraHeightM        = (float)StereoCalibrationData.DefaultCameraHeightM;
        private static readonly float CameraForwardOffsetM = (float)StereoCalibrationData.DefaultCameraForwardOffsetM;
        private static readonly float TiltOffNadirRad = MathF.Atan(CameraForwardOffsetM / CameraHeightM);
        private static readonly float SinTilt = MathF.Sin(TiltOffNadirRad);
        private static readonly float CosTilt = MathF.Cos(TiltOffNadirRad);

        // Ball starts at the tee itself — ground level, centred left-right, distance
        // zero along the world Z axis (which now measures distance from the TEE, not
        // from the camera; the camera's own offset from the tee is CameraForwardOffsetM,
        // folded into ProjectToCamera below). The camera's ~3.0145m slant distance to
        // this exact point is what previously had to be hand-set as a flat Z offset.
        private const float  BallStartX     = 0.0f;    // centred left-right

        // ── Trajectory ────────────────────────────────────────────────────────
        private float _speedMps       = 30f;
        private float _launchAngleDeg = 15f;
        private float _azimuthDeg     = 0f;

        // ── Synthetic noise ───────────────────────────────────────────────────
        // Was 1.5px, calibrated to the old BallDetector's bounding-rect-center method
        // (integer-quantized to 0.5px steps, plus real quantization/threshold jitter).
        // BallDetector now computes a moment-based sub-pixel centroid instead — real
        // achievable accuracy on a clean, well-contrasted blob this size is ~0.1–0.3px,
        // not 1.5px. Injecting noise sized for a detector this rig no longer runs
        // overstates real depth/velocity uncertainty: at Z=5m, 1.5px of disparity noise
        // implies ~6cm of depth error per measurement — comparable to or larger than
        // the ball's actual per-frame displacement, which was swamping the Kalman
        // velocity fit with noise levels that will never actually occur.
        private float _positionNoisePx = 0.3f;   // pixel std-dev per detection
        private readonly Random _rng = new(42);  // seeded for reproducibility

        // Fixed relative offsets (fractions of the ball's radius) for a handful of
        // interior speckles, generated once and drawn unrotated on every frame.
        // Without these, the ball was a perfectly flat white disc — FeaturePointTracker's
        // GFTT corner detector had no real interior texture to lock onto, so it tracked
        // edge/threshold noise instead, producing wildly different "rotation" readings
        // frame to frame (single-digit rpm one moment, ~1000rpm the next, random axis
        // each time). BallController applies that spin as a real Magnus force — one
        // such spurious high-rpm reading with a near-vertical axis was enough to send
        // the ball into a visible circular orbit in Unity instead of a straight,
        // decaying flight. Keeping the pattern's orientation fixed (never rotating it)
        // matches the physical truth: this mock doesn't model ball spin at all, so the
        // correct answer for Spin3D to recover here is ~0rpm, not noise.
        private readonly (float dx, float dy)[] _speckleOffsets;

        // ── Timing ────────────────────────────────────────────────────────────
        // Matches the real rig's confirmed default: SpinView reports 199.76fps
        // full-frame (see KalmanBallTracker.cs). Reduced-ROI fps is still unmeasured.
        private float _fps = 199.76f;

        public BlockingCollection<CameraFrame> FrameQueue { get; } = new(256);

        private CancellationTokenSource? _cts;
        private Task? _generateFramesTask;
        private string _sportId = "generic";

        // Looks up MinContourArea/MaxContourArea for whatever sport is active so the
        // synthetic blob actually lands inside BallDetector's accepted range for that
        // profile — a fixed radius here previously meant soccer's real 800–8000px²
        // window (this blob was ~113px²) never matched, so no mock shot was ever
        // detected end-to-end for soccer.
        private readonly SportProfileRegistry _profiles = new();
        private float _blobRadiusPx = 6f;

        public MockCameraManager()
        {
            const int speckleCount = 10;
            _speckleOffsets = new (float, float)[speckleCount];
            for (int i = 0; i < speckleCount; i++)
            {
                float angle  = (float)(_rng.NextDouble() * 2.0 * Math.PI);
                float radius = 0.25f + (float)_rng.NextDouble() * 0.5f; // 0.25–0.75 of ball radius
                _speckleOffsets[i] = (MathF.Cos(angle) * radius, MathF.Sin(angle) * radius);
            }
        }

        // ── Public API ────────────────────────────────────────────────────────

        /// <summary>Configure the simulated ball launch.</summary>
        public void SetTrajectory(float speedMps = 30f, float launchAngleDeg = 15f,
                                  float azimuthDeg = 0f, float fps = 199.76f,
                                  float positionNoisePx = 0.3f)
        {
            _speedMps        = speedMps;
            _launchAngleDeg  = launchAngleDeg;
            _azimuthDeg      = azimuthDeg;
            _fps             = fps;
            _positionNoisePx = positionNoisePx;
        }

        public void Initialize(string activeSportId = "generic") => SetActiveSport(activeSportId);

        public void ApplyProfile(string sportId) => SetActiveSport(sportId);

        private void SetActiveSport(string sportId)
        {
            _sportId = sportId;
            var p = _profiles.GetOrDefault(sportId);
            float midArea = (p.MinContourArea + p.MaxContourArea) / 2f;
            _blobRadiusPx = MathF.Sqrt(midArea / MathF.PI);
        }

        public void StartCapture()
        {
            _cts = new CancellationTokenSource();
            _generateFramesTask = Task.Factory.StartNew(() => GenerateFrames(_cts.Token),
                TaskCreationOptions.LongRunning);
        }

        // Signaling cancellation alone isn't enough to guarantee frame production
        // has actually stopped by the time this returns — GenerateFrames' waits are
        // cancellation-aware (see its comments) so it exits promptly, but "promptly"
        // still isn't "before this method returns" without actually waiting for it.
        // A caller that immediately checks "no more frames arrive" right after
        // Stop() (as Tests/MockCameraManagerTests.Stop_HaltsFrameProduction does)
        // would otherwise race against whatever's still in flight on that thread.
        public void Stop()
        {
            _cts?.Cancel();
            _generateFramesTask?.Wait(TimeSpan.FromSeconds(1));
        }

        public void Dispose() { Stop(); FrameQueue.Dispose(); }

        // ── Frame generation ──────────────────────────────────────────────────

        private void GenerateFrames(CancellationToken ct)
        {
            float intervalMs = 1000f / _fps;
            int   frameCount = (int)(_fps * 2); // 2 seconds of flight

            // Decompose velocity into world components
            float launchRad  = _launchAngleDeg * MathF.PI / 180f;
            float azimuthRad = _azimuthDeg      * MathF.PI / 180f;
            float vz = _speedMps * MathF.Cos(launchRad) * MathF.Cos(azimuthRad);
            float vx = _speedMps * MathF.Cos(launchRad) * MathF.Sin(azimuthRad);
            float vy = _speedMps * MathF.Sin(launchRad);

            float dt    = 1f / _fps;
            float bx    = BallStartX;
            float by    = 0f; // ground level, at address
            float bz    = 0f; // at the tee

            long  timestampUs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() * 1000L;

            // BallDetector.Detect() always treats the first frame it ever sees for a
            // given camera as a background reference, never a real detection (see its
            // class comment) — correct behavior for real hardware, which needs a real
            // first frame to learn the scene's actual lighting/background. But this
            // rig's real capture window (confirmed via AccuGolf's own install manual:
            // a 12in-deep hitting zone, only ~1-2 real frames' worth of travel at real
            // ball speeds) is tight enough that losing one of the ball's own frames to
            // background setup is a large fraction of the whole budget. A real
            // installation presumably captures its background reference before the
            // shot happens (during setup), not from the ball's own flight — so prime
            // each camera's background here with an empty (no-ball) frame first,
            // giving the detector something to diff against before the ball ever
            // appears, instead of burning the ball's actual first appearance on it.
            var empty0 = new CameraFrame { CameraIndex = 0, Data = new byte[SensorW * SensorH], Width = SensorW, Height = SensorH, TimestampUs = timestampUs, ExposureTimeUs = 500.0, GainDb = 0.0 };
            var empty1 = new CameraFrame { CameraIndex = 1, Data = new byte[SensorW * SensorH], Width = SensorW, Height = SensorH, TimestampUs = timestampUs, ExposureTimeUs = 500.0, GainDb = 0.0 };
            FrameQueue.TryAdd(empty0);
            FrameQueue.TryAdd(empty1);

            // A real rig runs continuously with the ball sitting at address before
            // the shot — KalmanBallTracker's rest-position seeding (see its class
            // comment) relies on seeing the ball genuinely at rest at least once so
            // it has a real "last seen at rest" timestamp to measure the shot's
            // start against, rather than falling back to its degraded (needs 2 real
            // detections) path. A handful of stationary frames here models that.
            const int restFrameCount = 10;
            for (int r = 0; r < restFrameCount && !ct.IsCancellationRequested; r++)
            {
                var rest0 = ProjectToCamera(bx, by, bz, 0, timestampUs);
                var rest1 = ProjectToCamera(bx, by, bz, 1, timestampUs);
                if (rest0 != null) FrameQueue.TryAdd(rest0);
                if (ct.IsCancellationRequested) break;
                if (rest1 != null) FrameQueue.TryAdd(rest1);
                timestampUs += (long)(dt * 1_000_000f);
                // Cancellation-aware wait, not a raw Thread.Sleep — Stop() needs to
                // take effect within this call, not after it finishes sleeping.
                // Was fine when this loop didn't exist (only 2 priming frames added
                // before the real trajectory), but the rest-frame loop lengthened
                // the pre-shot window enough that Tests/MockCameraManagerTests.cs's
                // Stop_HaltsFrameProduction went from an occasional flake to
                // consistently reproducing — a real regression, not a pre-existing
                // one, worth fixing here rather than in the test.
                ct.WaitHandle.WaitOne((int)intervalMs);
            }

            for (int f = 0; f < frameCount && !ct.IsCancellationRequested; f++)
            {
                // Advance ball position (gravity on Y)
                bx += vx * dt;
                by += vy * dt - 0.5f * 9.81f * dt * dt;
                bz += vz * dt;
                vy -= 9.81f * dt;

                // World-frame ball position handed to each camera; the stereo baseline
                // and the tilt/elevation are both applied inside ProjectToCamera now
                // (camIdx picks which side of the baseline that camera sits on).
                var frame0 = ProjectToCamera(bx, by, bz, 0, timestampUs);
                var frame1 = ProjectToCamera(bx, by, bz, 1, timestampUs);

                if (frame0 != null) FrameQueue.TryAdd(frame0);
                if (ct.IsCancellationRequested) break;
                if (frame1 != null) FrameQueue.TryAdd(frame1);

                timestampUs += (long)(dt * 1_000_000f);
                ct.WaitHandle.WaitOne((int)intervalMs); // cancellation-aware, see rest-frame loop's comment above
            }
        }

        // worldX/worldY/worldZ are the ball's position in TEE-centred world space
        // (Y-up, Z = downrange distance from the tee toward the net) — not yet
        // relative to either camera. Camera position is (±baseline/2, CameraHeightM,
        // CameraForwardOffsetM); its optical axis is aimed exactly at the tee, a pure
        // pitch rotation with no yaw/roll (see the class comment on CameraHeightM).
        private CameraFrame? ProjectToCamera(float worldX, float worldY, float worldZ,
                                              int camIdx, long timestampUs)
        {
            float camX = camIdx == 0 ? -BaselineM / 2f : BaselineM / 2f;

            float relX = worldX - camX;
            float relYWorld = worldY - CameraHeightM;
            float relZWorld = worldZ - CameraForwardOffsetM;

            // Rotate (relYWorld, relZWorld) into the camera's own tilted local frame.
            // Derived so the tee itself (0,0,0) lands exactly on the optical axis
            // (yLocal=0, zLocal=slant distance) — see notes/14-...md and the class
            // comment above for the geometry this comes from.
            float yLocal =  relYWorld * SinTilt - relZWorld * CosTilt;
            float zLocal = -relYWorld * CosTilt - relZWorld * SinTilt;

            if (zLocal <= 0f) return null; // behind camera

            // Perspective projection
            float px = FocalPx * relX / zLocal + SensorW / 2f;
            float py = FocalPx * (-yLocal) / zLocal + SensorH / 2f; // Y-down

            // Add Gaussian noise
            px += (float)SampleGaussian() * _positionNoisePx;
            py += (float)SampleGaussian() * _positionNoisePx;

            if (px < 0 || px >= SensorW || py < 0 || py >= SensorH) return null; // off-sensor

            // Build a minimal synthetic frame: black background, white ball blob.
            // Radius is sized (in SetActiveSport) to land inside the active profile's
            // MinContourArea/MaxContourArea band — not a physically-projected size,
            // just enough to make BallDetector actually fire for whichever sport is
            // selected. Clamped so a custom/generic profile with an extreme area
            // range can't produce a degenerate (near-zero or off-sensor) blob.
            int   r      = Math.Max(2, Math.Min(200, (int)MathF.Round(_blobRadiusPx)));
            var   data   = new byte[SensorW * SensorH];
            int   cx     = (int)px;
            int   cy     = (int)py;

            for (int dy = -r; dy <= r; dy++)
            for (int dx = -r; dx <= r; dx++)
            {
                if (dx*dx + dy*dy > r*r) continue;
                int row = cy + dy, col = cx + dx;
                if (row < 0 || row >= SensorH || col < 0 || col >= SensorW) continue;
                data[row * SensorW + col] = 255;
            }

            // Interior speckles for FeaturePointTracker to actually track — see the
            // class comment on _speckleOffsets. Offsets are fractions of r, so they
            // scale correctly with whatever sport profile sized the blob.
            const int speckleDotRadius = 2;
            const byte speckleValue = 90; // dark enough against the 255 fill for GFTT/stdDev checks
            foreach (var (odx, ody) in _speckleOffsets)
            {
                int scx = cx + (int)MathF.Round(odx * r);
                int scy = cy + (int)MathF.Round(ody * r);
                for (int dy = -speckleDotRadius; dy <= speckleDotRadius; dy++)
                for (int dx = -speckleDotRadius; dx <= speckleDotRadius; dx++)
                {
                    if (dx*dx + dy*dy > speckleDotRadius*speckleDotRadius) continue;
                    int row = scy + dy, col = scx + dx;
                    if (row < 0 || row >= SensorH || col < 0 || col >= SensorW) continue;
                    data[row * SensorW + col] = speckleValue;
                }
            }

            return new CameraFrame
            {
                CameraIndex    = camIdx,
                Data           = data,
                Width          = SensorW,
                Height         = SensorH,
                TimestampUs    = timestampUs,
                ExposureTimeUs = 500.0,
                GainDb         = 0.0
            };
        }

        // Box-Muller transform for Gaussian noise
        private double SampleGaussian()
        {
            double u1 = 1.0 - _rng.NextDouble();
            double u2 = 1.0 - _rng.NextDouble();
            return Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Sin(2.0 * Math.PI * u2);
        }
    }
}
