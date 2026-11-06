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
        private readonly StereoClock     _clock = new();
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

        // Stereo fixes of the shot in progress: its final velocity is fitted through
        // them, and they show when a "shot" has stopped in view.
        private readonly List<(long us, float x, float y, float z)> _shotSamples = new();
        private const int MinShotSamples     = 3;     // fewer than this isn't a ball in flight
        private const int FootContactSamples = 1;     // the foot may still be accelerating the ball
        private const int StoppedSamples     = 40;    // ~150ms at the real 262fps
        private const float StoppedWithinM   = 0.03f;
        private const float ShotGateM        = 0.3f;  // well above a real prediction error
        private const float FrameSeconds     = 1f / 262f;

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
        private int  _shotCount = 0;

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
            // Check the working directory first (dotnet run from the repo root),
            // then next to the exe — launching the built exe directly (shortcut,
            // double-click, startup task) sets the working directory elsewhere, and
            // silently falling back to CreateDefaults() on the real rig would send
            // Unity badly-scaled positions/velocities with no other symptom.
            string exeDirPath = System.IO.Path.Combine(AppContext.BaseDirectory, CalibPath);
            string? calibFile = System.IO.File.Exists(CalibPath) ? System.IO.Path.GetFullPath(CalibPath)
                              : System.IO.File.Exists(exeDirPath) ? exeDirPath
                              : null;

            StereoCalibrationData cal;
            if (calibFile != null)
            {
                cal = StereoCalibrationData.LoadFromFile(calibFile);
                Console.WriteLine($"[Engine] Loaded calibration from {calibFile}");
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
            // UDP is connectionless — there's no handshake/"connection" the way TCP
            // has one. This is the closest honest equivalent: a ProfileSelectCommand
            // is real, observable evidence Unity is running and reached the
            // middleware. Logged unconditionally (before the early-return below),
            // since without it this line never printed anything at all when Unity's
            // sport already matched the already-active default (soccer matches
            // soccer on this rig) — every one of Unity's startup retries hit the
            // early-return with zero visible confirmation Unity was ever heard from.
            Console.WriteLine($"[Engine] Received ProfileSelectCommand from Unity: '{cmd.SportId}'");

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
                // One cycle = one frame from each camera, or as much as arrives before
                // the 5ms timeout if a camera has gone quiet. Never "until the queue
                // goes quiet": two cameras at 200fps deliver a frame at least every
                // ~5ms, so with continuous real capture that wait never timed out and
                // nothing past this loop ever ran — no triangulation, no [Tracking], no
                // shot, however good the detections were. The mock hid this because its
                // frames stop after a short flight. (Found with a real-rate simulated
                // kick: ~1,570 detections, zero [Tracking] lines, zero packets.)
                DetectionResult? det0 = null, det1 = null;
                CameraFrame? rectFrame0 = null, rectFrame1 = null;
                bool seen0 = false, seen1 = false;

                // Spin only once a shot is underway. A resting ball has no spin to
                // measure, and estimating it on every frame couldn't keep up with 400
                // frames a second: the bounded queue stayed full for as long as the ball
                // sat there, so the kick's ~5 frames arrived late or were lost, one
                // camera's capture thread starved the other, and frames stopped pairing.
                bool estimateSpin = !_skipSpinEstimation && _tracker.HasFix;

                while (!(seen0 && seen1) && _cameras.FrameQueue.TryTake(out var frame, 5))
                {
                    if (frame.CameraIndex == 0) seen0 = true; else seen1 = true;

                    // Detection, triangulation (disparity + DLT via P0/P1), and
                    // stereo point-matching for 3D spin all assume RECTIFIED image
                    // coordinates — undistorted, with corresponding points on the
                    // same row between cameras. Rectify before anything else touches
                    // the frame.
                    var rectified = RectifyFrame(frame);
                    var det = _detector.Detect(rectified);
                    if (!det.Found) continue;
                    if (estimateSpin) UpdateSpinEstimate(det);
                    if (det.CameraIndex == 0) { det0 = det; rectFrame0 = rectified; }
                    else                      { det1 = det; rectFrame1 = rectified; }
                }

                // Only a stereo fix counts as seeing the ball. One camera alone can't give
                // depth, and after a real kick what one camera saw was usually a hand: it
                // kept a finished shot "alive" for over a second while its reported position
                // ran off to nonsense. Mid-shot, a fix also has to be where the ball could be.
                var stereo = det0 != null && det1 != null
                    ? _triangulator.TriangulateStereo(det0.Center, det1.Center)
                    : null;
                bool ballSeen = stereo != null
                    && stereo.Tier != TriangulationTier.Monocular
                    && (!_tracker.HasFix || IsNearPrediction(stereo));

                TriangulatedPoint pt3d;
                if (ballSeen)
                {
                    pt3d = stereo!;
                    if (estimateSpin) UpdateSpin3D(rectFrame0!, rectFrame1!, det0!, pt3d);
                    _coastFrames = 0;
                }
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
                    if (!_tracker.HasFix) continue;

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
                // dt regardless of how many frames got batched into this cycle.
                long ts = _clock.Next(det0?.TimestampUs, det1?.TimestampUs);

                (float x, float y, float z, float vx, float vy, float vz) state;
                if (pt3d.Tier == TriangulationTier.KalmanOnly)
                    state = _tracker.Predict(ts);
                else
                    state = _tracker.Update(mx, my, mz, ts);

                // A real shot's ball leaves the view. One that has stopped in view wasn't
                // a shot (a nudge, a foot, a bad detection), so drop it rather than stay
                // "in flight" on a resting ball and swallow the real kick that follows.
                if (ballSeen && _tracker.HasFix)
                {
                    _shotSamples.Add((ts, mx, my, mz));
                    if (HasStopped())
                    {
                        Console.WriteLine("[Engine] Ball stopped in view: not a shot, waiting for the next one");
                        ResetShot();
                        continue;
                    }
                }

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
                // Actually sent once, when the shot ends (coast-timeout branch
                // above / SendPendingShotAndReset below).
                if (pt3d.Tier != TriangulationTier.KalmanOnly && _tracker.HasFix)
                    _pendingShotData = ballData;

                // Throttled live-tracking readout — diagnostic only, not a send
                // indicator. Every ~5s while the ball waits, so the [Shot] line stands
                // out; a few lines across a shot's ~10 frames.
                if (++_logFrameCounter % (_tracker.HasFix ? 5 : 1310) == 0)
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

            if (_shotSamples.Count < MinShotSamples)
            {
                Console.WriteLine($"[Engine] Only {_shotSamples.Count} frame(s) of motion: not a shot, waiting for the next one");
                ResetShot();
                return;
            }

            ApplyFlightVelocity(_pendingShotData);

            // The fit sees through what started this "shot". A person shading the ball
            // while placing it makes its detected position jump, which looks like a shot
            // start, but across all its frames the ball hasn't really gone anywhere.
            // Sent to the game, these made it hop at 0.1 and 1.4 m/s (found live).
            if (_pendingShotData.SpeedMps < _activeProfile.MinSpeedMps)
            {
                Console.WriteLine($"[Engine] Only moving {_pendingShotData.SpeedMps:F1}m/s: not a shot, waiting for the next one");
                ResetShot();
                return;
            }

            _udp?.Send(_pendingShotData);

            float sideDeg = MathF.Atan2(_pendingShotData.VelX, _pendingShotData.VelZ) * 180f / MathF.PI;
            Console.WriteLine();
            Console.ForegroundColor = ConsoleColor.Green;
            Console.WriteLine(
                $"[Shot] #{++_shotCount}: {_pendingShotData.SpeedMps:F1} m/s, launch {_pendingShotData.LaunchAngleDeg:F0} deg, " +
                $"{MathF.Abs(sideDeg):F0} deg {(sideDeg >= 0 ? "right" : "left")}");
            Console.ResetColor();
            Console.WriteLine(
                $"       vel=({_pendingShotData.VelX:F1},{_pendingShotData.VelY:F1},{_pendingShotData.VelZ:F1}) " +
                $"frames={_shotSamples.Count} spin={_pendingShotData.SpinRpm:F0}rpm conf={_pendingShotData.Confidence:F2}");
            Console.WriteLine();

            ResetShot();
        }

        // A straight line through the flight's stereo fixes. The Kalman estimate starts
        // from the very first motion, when the foot is still accelerating the ball, and
        // has only ~10 frames to recover before the ball leaves the view: it read 7 m/s
        // for a real kick moving 11-12 m/s. Gravity is negligible over those ~40ms.
        private void ApplyFlightVelocity(BallData shot)
        {
            int first = _shotSamples.Count >= MinShotSamples + FootContactSamples ? FootContactSamples : 0;
            long t0 = _shotSamples[first].us;
            double n = 0, st = 0, stt = 0, sx = 0, sy = 0, sz = 0, stx = 0, sty = 0, stz = 0;
            for (int i = first; i < _shotSamples.Count; i++)
            {
                var (us, x, y, z) = _shotSamples[i];
                double t = (us - t0) / 1e6;
                n++; st += t; stt += t * t;
                sx += x; sy += y; sz += z;
                stx += t * x; sty += t * y; stz += t * z;
            }
            double denom = n * stt - st * st;
            if (denom <= 0) return;

            shot.VelX = (float)((n * stx - st * sx) / denom);
            shot.VelY = (float)((n * sty - st * sy) / denom);
            shot.VelZ = (float)((n * stz - st * sz) / denom);
            shot.SpeedMps = MathF.Sqrt(shot.VelX * shot.VelX + shot.VelY * shot.VelY + shot.VelZ * shot.VelZ);
            if (_activeProfile.OutputLaunchAngle)
                shot.LaunchAngleDeg = MathF.Atan2(shot.VelY, MathF.Sqrt(shot.VelX * shot.VelX + shot.VelZ * shot.VelZ)) * 180f / MathF.PI;
        }

        private bool IsNearPrediction(TriangulatedPoint p)
        {
            var (x, y, z, vx, vy, vz) = _tracker.LastState;
            float dt = (_coastFrames + 1) * FrameSeconds;
            float dx = p.X - (x + vx * dt), dy = p.Y - (y + vy * dt), dz = p.Z - (z + vz * dt);
            return MathF.Sqrt(dx * dx + dy * dy + dz * dz) < ShotGateM;
        }

        private bool HasStopped()
        {
            if (_shotSamples.Count <= StoppedSamples) return false;
            var a = _shotSamples[_shotSamples.Count - 1];
            var b = _shotSamples[_shotSamples.Count - 1 - StoppedSamples];
            float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
            return MathF.Sqrt(dx * dx + dy * dy + dz * dz) < StoppedWithinM;
        }

        private void ResetShot()
        {
            _pendingShotData = null;
            _shotSamples.Clear();
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
