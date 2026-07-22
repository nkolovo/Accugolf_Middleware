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

namespace SportSimulator.Vision
{
    public class MockCameraManager : ICameraManager
    {
        // ── Camera rig geometry ────────────────────────────────────────────────
        // Matches the defaults in StereoCalibrationData.CreateDefaults().
        // Two cameras 495.3mm apart (19.5in measured), each with a 1280×1024
        // sensor, 8mm focal length.
        private const int    SensorW        = 1280;
        private const int    SensorH        = 1024;
        private const float  FocalPx        = 1800f;   // ≈ 8mm lens at 4.8µm pitch
        private const float  BaselineM      = 0.4953f; // metres between cameras (19.5in)
        private const float  CamHeightM     = 0.00f;   // cameras at origin height
        private const float  BallStartZ     = 0.0f;    // ball starts at camera plane
        private const float  BallStartX     = 0.0f;    // centred left-right

        // ── Trajectory ────────────────────────────────────────────────────────
        private float _speedMps       = 30f;
        private float _launchAngleDeg = 15f;
        private float _azimuthDeg     = 0f;

        // ── Synthetic noise ───────────────────────────────────────────────────
        private float _positionNoisePx = 1.5f;   // pixel std-dev per detection
        private readonly Random _rng = new(42);  // seeded for reproducibility

        // ── Timing ────────────────────────────────────────────────────────────
        // Matches the real rig's confirmed default: SpinView reports 199.76fps
        // full-frame (see KalmanBallTracker.cs). Reduced-ROI fps is still unmeasured.
        private float _fps = 199.76f;

        public BlockingCollection<CameraFrame> FrameQueue { get; } = new(256);

        private CancellationTokenSource? _cts;
        private string _sportId = "generic";

        // ── Public API ────────────────────────────────────────────────────────

        /// <summary>Configure the simulated ball launch.</summary>
        public void SetTrajectory(float speedMps = 30f, float launchAngleDeg = 15f,
                                  float azimuthDeg = 0f, float fps = 199.76f,
                                  float positionNoisePx = 1.5f)
        {
            _speedMps        = speedMps;
            _launchAngleDeg  = launchAngleDeg;
            _azimuthDeg      = azimuthDeg;
            _fps             = fps;
            _positionNoisePx = positionNoisePx;
        }

        public void Initialize(string activeSportId = "generic") => _sportId = activeSportId;

        public void ApplyProfile(string sportId) => _sportId = sportId;

        public void StartCapture()
        {
            _cts = new CancellationTokenSource();
            Task.Factory.StartNew(() => GenerateFrames(_cts.Token),
                TaskCreationOptions.LongRunning);
        }

        public void Stop() => _cts?.Cancel();

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
            float by    = CamHeightM;
            float bz    = BallStartZ + 0.5f; // start 0.5m in front of cameras

            long  timestampUs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() * 1000L;

            for (int f = 0; f < frameCount && !ct.IsCancellationRequested; f++)
            {
                // Advance ball position (gravity on Y)
                bx += vx * dt;
                by += vy * dt - 0.5f * 9.81f * dt * dt;
                bz += vz * dt;
                vy -= 9.81f * dt;

                // Project onto each camera sensor
                // Camera 0: at (-baseline/2, 0, 0)  looking +Z
                // Camera 1: at (+baseline/2, 0, 0)  looking +Z
                var frame0 = ProjectToCamera(bx + BaselineM / 2f, by, bz, 0, timestampUs);
                var frame1 = ProjectToCamera(bx - BaselineM / 2f, by, bz, 1, timestampUs);

                if (frame0 != null) FrameQueue.TryAdd(frame0);
                if (frame1 != null) FrameQueue.TryAdd(frame1);

                timestampUs += (long)(dt * 1_000_000f);
                Thread.Sleep((int)intervalMs);
            }
        }

        private CameraFrame? ProjectToCamera(float relX, float relY, float relZ,
                                              int camIdx, long timestampUs)
        {
            if (relZ <= 0f) return null; // behind camera

            // Perspective projection
            float px = FocalPx * relX / relZ + SensorW / 2f;
            float py = FocalPx * (-relY) / relZ + SensorH / 2f; // Y-down

            // Add Gaussian noise
            px += (float)SampleGaussian() * _positionNoisePx;
            py += (float)SampleGaussian() * _positionNoisePx;

            if (px < 0 || px >= SensorW || py < 0 || py >= SensorH) return null; // off-sensor

            // Build a minimal synthetic frame: black background, white ball blob
            int   r      = 6; // radius in pixels
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
