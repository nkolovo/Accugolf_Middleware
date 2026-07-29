// ------------------------------------------------------------
// App/SimulatorEngine.cs
// ------------------------------------------------------------
using System;
using System.Collections.Generic;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using SportSimulator.Models;
using SportSimulator.Profiles;
using SportSimulator.Tracking;
using SportSimulator.Transport;
using SportSimulator.Vision;
using SportSimulator.Vision.Calibration;

namespace SportSimulator.App
{
    public class SimulatorEngine : IDisposable
    {
        private readonly ICameraManager    _cameras;
        private readonly BallDetector      _detector = new();
        private readonly KalmanBallTracker _tracker = new();
        private readonly SportProfileRegistry _registry = new();
        private readonly StereoRectifier _rectifier = new();
        private readonly Triangulator    _triangulator = new();
        private readonly SpinEstimator   _spinEstimator = new();
        private readonly Spin3DEstimator _spin3DEstimator;

        /// <summary>
        /// Production constructor — uses the real Spinnaker-backed CameraManager.
        /// Only available on net48 (Spinnaker SDK is not present on net10.0).
        /// </summary>
#if NET48
        public SimulatorEngine() : this(new CameraManager()) { }
#endif

        /// <summary>
        /// Injection constructor — pass a MockCameraManager (or any ICameraManager)
        /// for unit tests on machines without the Spinnaker SDK.
        /// </summary>
        public SimulatorEngine(ICameraManager cameras)
        {
            _cameras = cameras;
            _activeProfile = _registry.GetOrDefault("soccer");
            _spin3DEstimator = new Spin3DEstimator(_triangulator);
            _skipSpinEstimation = cameras is MockCameraManager;
        }

        // MockCameraManager's ball has no real rotating surface texture — both
        // SpinEstimator (2D correlation) and Spin3DEstimator (natural-texture point
        // tracking) need one to produce a genuine signal. Without it, the recovered
        // axis is essentially random from frame to frame (confirmed live: readings
        // stayed under the plausibility cap yet still visibly perturbed the ball's
        // path in Unity — BallController overwrites its Magnus-force spin vector on
        // every packet, so a rapidly rotating random axis reads as a jittery,
        // "squiggly" path even though it nets out close to straight). Capping
        // magnitude only stops the worst outliers; it can't fix an axis that has no
        // real information in it. Skip spin estimation entirely for mock testing —
        // real spin validation needs a real textured ball on real hardware, same as
        // this code's comments have said from the start.
        private readonly bool _skipSpinEstimation;

        private UdpTransport? _udp;
        private SportProfile  _activeProfile = null!; // set in constructor before use
        private bool _running;

        // Latest detection from each camera, keyed by camera index
        private readonly Dictionary<int, DetectionResult> _pending = new();
        private readonly object _pendingLock = new();

        // Max frames the Kalman filter will predict forward without any detection.
        // Set from SportProfile.KalmanCoastFrames on each profile switch.
        private int _maxCoastFrames = 5;
        private int _coastFrames = 0;

        // Triangulator.TriangulateMonocular computes X/Y from a single camera's pixel
        // position PLUS the tracker's own last-known Z — not an independent Z
        // measurement (and X/Y inherit that same non-independence, since their scale
        // is pixel * Z / fx). Originally this got fed straight back into the Kalman
        // filter via Update(), so during a sustained run of monocular-only cycles
        // (common once real stereo agreement gets shaky further from the camera),
        // nothing ever independently confirmed Z — the filter just echoed back
        // whatever it already believed, and any small error in the Z-velocity
        // estimate compounded unbounded the longer the streak ran. Found live: a
        // faster (25 m/s) mock shot spent longer in this state per real-time
        // interval and the final result diverged to a nonsensical (10.6, 6.6, 55.4)
        // position. A first attempt at fixing this capped how many CONSECUTIVE
        // monocular fixes could feed Update() before falling back to coasting — but
        // no fixed tick count worked: the throttled [Tracking] log only prints every
        // 20th real cycle, so caps of both 5 and 15 were being exceeded well before
        // even the first post-stereo log line, freezing the sent result at an
        // under-developed early position every time. The actual fix is simpler and
        // needs no threshold at all: see the Predict()/Update() branch below.

        // Most recent ball crop seen per camera, for frame-to-frame spin correlation.
        // Keyed by CameraIndex. See SpinEstimator.cs for method + current limits
        // (reliable for soccer's spin rates at 120fps; not yet for golf/baseball/tennis).
        private readonly Dictionary<int, (byte[] crop, int size, long timestampUs)> _lastCrop = new();
        private float _lastSpinRpm = 0f;

        // Full 3D spin (magnitude + axis) via natural-texture point tracking —
        // see Vision/Spin3DEstimator.cs. Only runs when both cameras detected the
        // ball this cycle (needs a stereo pair for triangulation). Takes priority
        // over the single-camera 2D SpinEstimator above when it produces a valid
        // fit; the 2D estimate remains as a magnitude-only fallback otherwise
        // (e.g. too few tracked/matched points this cycle).
        // AxisUnit is camera-relative, NOT yet rotated into shot-relative
        // backspin/sidespin terms — see Spin3DEstimator.cs class comment.
        private Vec3 _lastSpinAxis = default;
        private int  _logFrameCounter = 0;

        // Holds the most recently computed BallData from a REAL (non-KalmanOnly)
        // measurement — i.e. the Kalman filter's fused estimate as of the latest
        // actual detection, not a coasted/extrapolated one. Overwritten every real
        // cycle, so it always reflects the best available consolidated result.
        // Actually sent to Unity only once, when the shot ends (see the Tier 4
        // coast-timeout branch) — see notes/11-unity-integration.md: the original
        // design called for one "shot result" message per ball, not a continuous
        // stream. The previous per-cycle Send() sent hundreds of packets per shot
        // (each overwriting BallController's velocity in Unity), which worked once
        // the underlying tracking was fixed, but doesn't match either the intended
        // design or how real launch monitors report a shot.
        private BallData? _pendingShotData;

        // Calibration file path — generated by running StereoCalibrator once.
        // ⚠️ SETUP TODO — run the calibration routine before first real use:
        //   1. Print a 9×6 checkerboard with 25mm squares
        //   2. Hold it at various positions/angles visible to both cameras
        //   3. Call StereoCalibrator.AddFramePair() for 15+ positions
        //   4. Call StereoCalibrator.Calibrate() — outputs this JSON file
        //   5. Aim for RMS reprojection error < 1.0px (< 0.5px is excellent)
        // Until this file exists, CreateDefaults() is used as a fallback.
        private const string CalibPath = "stereo_calibration.json";

        public void Start(string unityIp = "127.0.0.1", int sendPort = 7100, int listenPort = 7101)
        {
            LoadCalibration();

            _udp = new UdpTransport(unityIp, sendPort, listenPort);
            _udp.ProfileCommandReceived += OnProfileCommand;
            _udp.StartListening();

            _cameras.Initialize(_activeProfile.SportId);
            ApplyProfile(_activeProfile);
            _cameras.StartCapture();

            _running = true;
            Console.WriteLine("[Engine] Running. Waiting for stereo frame pairs...");
            ProcessLoop();
        }

        private void LoadCalibration()
        {
            StereoCalibrationData cal;
            if (System.IO.File.Exists(CalibPath))
            {
                cal = StereoCalibrationData.LoadFromFile(CalibPath);
                Console.WriteLine($"[Engine] Loaded calibration from {CalibPath}");
            }
            else
            {
                // ⚠️ SETUP TODO — using estimated defaults until calibration is run.
                // Baseline measured at 19.5in center-to-center → 19.5 * 0.0254 = 0.4953m.
                // Sensor resolution/focal length below are still unconfirmed placeholders —
                // update once the checkerboard calibration (StereoCalibrator) has been run.
                cal = StereoCalibrationData.CreateDefaults(baselineMetres: 0.4953);
                Console.WriteLine("[Engine] WARNING: Using default calibration. Run StereoCalibrator for accuracy.");
            }

            _rectifier.Build(cal);
            _triangulator.Configure(cal);
        }

        private void OnProfileCommand(ProfileSelectCommand cmd)
        {
            // Unity's ProfileSelectSender resends the same sport a few times on
            // startup in case the middleware isn't listening yet (no ack in this
            // protocol — see Assets/Scripts/ProfileSelectSender.cs). Re-applying an
            // already-active profile is a no-op in terms of configuration, but
            // ApplyProfile rebuilds the Kalman tracker from scratch — if one of
            // those redundant resends lands while a shot is actually mid-flight
            // (the mock test window overlaps it easily), it silently wipes the
            // tracker's converged velocity back to zero. Skip the reset entirely
            // when nothing is actually changing.
            if (string.Equals(cmd.SportId, _activeProfile.SportId, StringComparison.OrdinalIgnoreCase))
                return;

            var p = _registry.GetOrDefault(cmd.SportId);
            ApplyProfile(p);
            Console.WriteLine($"[Engine] Profile switched to: {p.DisplayName}");
        }

        private void ApplyProfile(SportProfile p)
        {
            _activeProfile  = p;
            _detector.SetProfile(p);
            _tracker.Configure(p);
            _cameras.ApplyProfile(p.SportId);
            _maxCoastFrames = p.KalmanCoastFrames;
            Console.WriteLine($"[Engine] Coast window set to {_maxCoastFrames} frames for {p.DisplayName}");
        }

        private void ProcessLoop()
        {
            while (_running)
            {
                // Drain all frames that arrived since last iteration
                DetectionResult? det0 = null, det1 = null;
                CameraFrame? rectFrame0 = null, rectFrame1 = null;

                while (_cameras.FrameQueue.TryTake(out var frame, 5))
                {
                    // Detection, triangulation (disparity + DLT via P0/P1), and
                    // stereo point-matching for 3D spin all assume RECTIFIED image
                    // coordinates — undistorted, with corresponding points on the
                    // same row between cameras. Rectify before anything else touches
                    // the frame.
                    var rectified = RectifyFrame(frame);
                    var det = _detector.Detect(rectified);
                    if (!det.Found) continue;
                    if (!_skipSpinEstimation) UpdateSpinEstimate(det);
                    if (det.CameraIndex == 0) { det0 = det; rectFrame0 = rectified; }
                    else                      { det1 = det; rectFrame1 = rectified; }
                }

                TriangulatedPoint? pt3d = null;

                // ── Tier 1 / 2: both cameras this cycle ─────────────────────────
                if (det0 != null && det1 != null)
                {
                    pt3d = _triangulator.TriangulateStereo(det0.Center, det1.Center);

                    // If stereo disagreed and fell to monocular internally, enrich it
                    // with the Kalman's last Z — but only once a real stereo fix has
                    // already established a genuine Z. Before that, LastState.z is
                    // itself a meaningless 0, so "enriching" with it just produces
                    // ANOTHER degenerate (0,0,0) point — found by seeing every single
                    // one of a Kalman seed's buffered measurements come back at exactly
                    // (0.000, 0.000, 0.000): this self-referential loop, not real data,
                    // was what the seed was actually being built from. Same root cause
                    // as the Tier 3 fix below, just a different code path into it.
                    if (pt3d.Tier == TriangulationTier.Monocular && pt3d.Z == 0)
                    {
                        if (!_tracker.HasFix)
                        {
                            _coastFrames++;
                            if (_coastFrames > _maxCoastFrames) continue;
                            pt3d = new TriangulatedPoint
                            {
                                X = 0, Y = 0, Z = 0,
                                Confidence = 0.05f,
                                Tier = TriangulationTier.KalmanOnly,
                                TierReason = "Stereo self-rejected, no prior fix yet to enrich from"
                            };
                        }
                        else
                        {
                            var (kx, ky, kz, _, _, _) = _tracker.LastState;
                            pt3d = _triangulator.TriangulateMonocular(det0.Center, ky, kz);
                        }
                    }

                    if (pt3d.Tier != TriangulationTier.KalmanOnly)
                    {
                        if (!_skipSpinEstimation) UpdateSpin3D(rectFrame0!, rectFrame1!, det0, pt3d);
                        _coastFrames = 0;
                    }
                }
                // ── Tier 3: only one camera this cycle ──────────────────────────
                else if (det0 != null || det1 != null)
                {
                    // TriangulateMonocular needs an existing depth reference (the
                    // tracker's own last Z) — before any real stereo (Tier 1/2) fix
                    // has ever landed, that's a meaningless 0, and treating it as
                    // real would hand the tracker a fabricated point. That point
                    // would then poison KalmanBallTracker's two-point velocity seed
                    // (see its class comment) once a genuine stereo fix follows.
                    // Coast instead until stereo actually establishes a real Z.
                    if (!_tracker.HasFix)
                    {
                        _coastFrames++;
                        if (_coastFrames > _maxCoastFrames) continue;
                        pt3d = new TriangulatedPoint
                        {
                            X = 0, Y = 0, Z = 0,
                            Confidence = 0.05f,
                            Tier = TriangulationTier.KalmanOnly,
                            TierReason = "Single camera, no stereo fix yet to anchor depth"
                        };
                    }
                    else
                    {
                        var single = (det0 ?? det1)!;
                        var (_, ky, kz, _, _, _) = _tracker.LastState;
                        pt3d = _triangulator.TriangulateMonocular(single.Center, ky, kz);
                        _coastFrames = 0;
                    }
                }
                // ── Tier 4: no detection — Kalman coast ─────────────────────────
                else
                {
                    _coastFrames++;
                    if (_coastFrames > _maxCoastFrames)
                    {
                        // Ball genuinely gone — this is the one point where a shot
                        // actually ends. Send whatever the last real measurement
                        // produced (if any shot was in progress) and reset for the
                        // next one.
                        SendPendingShotAndReset();
                        continue;
                    }

                    var (kx, ky, kz, _, _, _) = _tracker.LastState;
                    float coastConf = Math.Max(0.05f, 0.3f - _coastFrames * (0.25f / _maxCoastFrames));
                    pt3d = new TriangulatedPoint
                    {
                        X = kx, Y = ky, Z = kz,
                        Confidence = coastConf,
                        Tier       = TriangulationTier.KalmanOnly,
                        TierReason = $"Coasting frame {_coastFrames}/{_maxCoastFrames}"
                    };
                }

                // Triangulator now returns true world coordinates directly (X centered
                // on the baseline, Y true height/up, Z true downrange distance from the
                // tee — see Triangulator.ToWorldFrame) — no per-axis flip needed here
                // anymore. A "Flip Y: OpenCV Y-down -> Unity Y-up" step used to live here,
                // which was correct back when Triangulator returned raw camera-local
                // coordinates, but became actively wrong once Triangulator started doing
                // its own world-frame correction: it would have flipped an already-
                // correct Y back into the wrong sign.
                float mx = pt3d.X, my = pt3d.Y, mz = pt3d.Z;

                // Real frame-capture timestamp when we have one — NOT wall-clock time
                // of this call — so KalmanBallTracker can compute its own real elapsed
                // dt regardless of how many frames got batched into this cycle. Falls
                // back to wall-clock only for a pure coast cycle (no detection at all
                // this cycle, so no frame timestamp exists) — see KalmanBallTracker.
                long ts = det0?.TimestampUs ?? det1?.TimestampUs
                    ?? DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() * 1000;

                // Only feed genuinely independent measurements into Update() — Monocular
                // is deliberately routed to Predict() alongside KalmanOnly, not just
                // "real" tiers in general. Its X/Y/Z all derive from the tracker's own
                // last-known Z (see the class comment above), so calling Update() with
                // it would just be the filter confirming its own prior belief back to
                // itself — the exact self-referential loop that caused unbounded Z
                // drift on faster shots. Predict()-only means a long monocular streak
                // simply coasts on the velocity last established by real stereo, which
                // is a static Kalman gets right by construction, instead of ratcheting
                // off some part-fabricated "measurement" — no magic streak-length
                // threshold needed.
                (float x, float y, float z, float vx, float vy, float vz) state;
                if (pt3d.Tier == TriangulationTier.KalmanOnly || pt3d.Tier == TriangulationTier.Monocular)
                    state = _tracker.Predict(ts);
                else
                    state = _tracker.Update(mx, my, mz, ts);

                var (sx, sy, sz, svx, svy, svz) = state;
                float speed       = MathF.Sqrt(svx*svx + svy*svy + svz*svz);
                float launchAngle = _activeProfile.OutputLaunchAngle
                    ? MathF.Atan2(svy, MathF.Sqrt(svx*svx + svz*svz)) * 180f / MathF.PI
                    : 0f;

                // Average detection confidence with triangulation confidence
                float detConf = det0 != null && det1 != null
                    ? (det0.Confidence + det1.Confidence) * 0.5f
                    : (det0?.Confidence ?? det1?.Confidence ?? 0f);
                float finalConf = (detConf + pt3d.Confidence) * 0.5f;

                var ballData = new BallData
                {
                    SportId        = _activeProfile.SportId,
                    TimestampUs    = ts,
                    PosX = sx, PosY = sy, PosZ = sz,
                    VelX = svx, VelY = svy, VelZ = svz,
                    SpeedMps       = speed,
                    LaunchAngleDeg = launchAngle,
                    SpinRpm        = _activeProfile.OutputSpin ? _lastSpinRpm : 0f,
                    SpinAxisX      = _activeProfile.OutputSpin ? _lastSpinAxis.X : 0f,
                    SpinAxisY      = _activeProfile.OutputSpin ? _lastSpinAxis.Y : 0f,
                    SpinAxisZ      = _activeProfile.OutputSpin ? _lastSpinAxis.Z : 0f,
                    Confidence     = finalConf,
                    TrackingTier   = (int)pt3d.Tier
                };

                // Cache rather than send — only once the tracker has actually
                // converged (HasFix) does svx/svy/svz mean anything; before that,
                // "real" tiers still report a placeholder zero velocity while
                // gathering the seed buffer (see KalmanBallTracker.Update), so
                // caching on tier alone caught that and sent a spurious speed=0
                // "shot" if a brief early coast-timeout hit before seeding finished.
                // Actually sent once, when the shot ends (Tier 4 coast-timeout
                // branch above / SendPendingShotAndReset below).
                if (pt3d.Tier != TriangulationTier.KalmanOnly && _tracker.HasFix)
                    _pendingShotData = ballData;

                // Throttled live-tracking readout — diagnostic only, not a send
                // indicator. At ~200fps, logging every frame would be an unreadable
                // firehose; every 20th frame is still a live ~10Hz readout while
                // testing, without flooding the console.
                if (++_logFrameCounter % 20 == 0)
                {
                    Console.WriteLine(
                        $"[Tracking] pos=({sx:F2},{sy:F2},{sz:F2}) speed={speed:F1}m/s tier={pt3d.Tier} " +
                        $"conf={finalConf:F2} spin={_lastSpinRpm:F0}rpm axis=({_lastSpinAxis.X:F2},{_lastSpinAxis.Y:F2},{_lastSpinAxis.Z:F2})");
                }
            }
        }

        // Called once the ball has genuinely left trackable range (coast frames
        // exceeded). Sends the last real measurement's consolidated result — the
        // Kalman filter's fused estimate as of the final detection, incorporating
        // every real measurement across the whole visible flight — as a single
        // packet, then resets tracking state for the next shot. A no-op if no shot
        // was ever established (e.g. still waiting for the first ball).
        private void SendPendingShotAndReset()
        {
            if (_pendingShotData == null) return;

            _udp?.Send(_pendingShotData);
            Console.WriteLine(
                $"[Shot] Sent final result: pos=({_pendingShotData.PosX:F2},{_pendingShotData.PosY:F2},{_pendingShotData.PosZ:F2}) " +
                $"speed={_pendingShotData.SpeedMps:F1}m/s spin={_pendingShotData.SpinRpm:F0}rpm " +
                $"conf={_pendingShotData.Confidence:F2}");

            _pendingShotData = null;
            _tracker.Configure(_activeProfile);
            _coastFrames = 0;
            _lastSpinRpm = 0f;
            _lastSpinAxis = default;
        }

        // Apply the stereo undistort+rectify maps (built once in LoadCalibration)
        // to a raw captured frame.
        private CameraFrame RectifyFrame(CameraFrame frame)
        {
            using var raw = new Mat(frame.Height, frame.Width, DepthType.Cv8U, 1);
            raw.SetTo(frame.Data);
            using var rectified = _rectifier.Rectify(raw, frame.CameraIndex);

            // Bulk CopyTo, not a per-pixel Matrix<byte> indexer loop (that pattern is
            // fine for BallDetector.ExtractCrop's tiny 48×48 spin crop, but doing it
            // over a full 720×540 frame — 388,800 individual marshalled indexer calls,
            // per camera, per frame — took ~14ms measured, well over the ~5ms budget
            // at 200fps. That backlogged the frame queue faster than it could drain,
            // collapsing an entire shot into one stale batch by the time processing
            // caught up. CopyTo(byte[]) does the same read in one native call.
            var bytes = new byte[rectified.Rows * rectified.Cols];
            rectified.CopyTo(bytes);

            return new CameraFrame
            {
                CameraIndex    = frame.CameraIndex,
                Data           = bytes,
                Width          = rectified.Cols,
                Height         = rectified.Rows,
                TimestampUs    = frame.TimestampUs,
                ExposureTimeUs = frame.ExposureTimeUs,
                GainDb         = frame.GainDb
            };
        }

        // Full 3D spin (magnitude + axis) — only possible when both cameras
        // detected the ball this cycle, since it needs a stereo pair to
        // triangulate tracked feature points. Falls through silently (leaving
        // _lastSpinRpm/_lastSpinAxis at their previous values) when the fit isn't
        // valid this frame — UpdateSpinEstimate's single-camera 2D magnitude-only
        // fallback keeps running independently either way.
        private void UpdateSpin3D(CameraFrame leftFrame, CameraFrame rightFrame, DetectionResult det0, TriangulatedPoint pt3d)
        {
            if (!_activeProfile.OutputSpin) return;

            using var leftMat  = FrameToMat(leftFrame);
            using var rightMat = FrameToMat(rightFrame);

            var m = _spin3DEstimator.Update(leftMat, rightMat, det0.Center, det0.RadiusPx,
                                             pt3d.Disparity, leftFrame.TimestampUs);
            if (!m.Valid) return;

            _lastSpinRpm  = m.Rpm;
            _lastSpinAxis = m.AxisUnit;

            // Unthrottled, unlike the general trajectory log below — how OFTEN this
            // line appears at all is itself the key diagnostic for whether natural-
            // texture tracking is finding enough points on your real ball surfaces
            // (see Vision/FeaturePointTracker.cs). Axis is camera-relative, not yet
            // rotated into backspin/sidespin terms — see Vision/Spin3DEstimator.cs.
            Console.WriteLine($"[Spin3D] rpm={m.Rpm:F0} axis=({m.AxisUnit.X:F2},{m.AxisUnit.Y:F2},{m.AxisUnit.Z:F2}) points={m.PointsUsed}");
        }

        private static Mat FrameToMat(CameraFrame frame)
        {
            var m = new Mat(frame.Height, frame.Width, DepthType.Cv8U, 1);
            m.SetTo(frame.Data);
            return m;
        }

        // Correlate this detection's crop against the last one seen from the SAME
        // camera to estimate rotation. Pairs more than 50ms apart (dropped frame,
        // coast gap, camera switch) are skipped rather than fed to the estimator —
        // the rotation-search range assumes near-consecutive frames.
        private void UpdateSpinEstimate(DetectionResult det)
        {
            if (!_activeProfile.OutputSpin || det.Crop == null) return;

            if (_lastCrop.TryGetValue(det.CameraIndex, out var prev) && prev.size == det.CropSize)
            {
                float dt = (det.TimestampUs - prev.timestampUs) / 1_000_000f;
                if (dt > 0f && dt < 0.05f)
                {
                    var m = _spinEstimator.Estimate(prev.crop, det.Crop, det.CropSize, dt);
                    if (m.Valid) _lastSpinRpm = m.Rpm;
                }
            }

            _lastCrop[det.CameraIndex] = (det.Crop, det.CropSize, det.TimestampUs);
        }

        public void Stop() => _running = false;
        public void Dispose() { Stop(); _cameras.Dispose(); _udp?.Dispose(); }
    }
}