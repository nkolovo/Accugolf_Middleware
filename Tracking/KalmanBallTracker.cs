// ------------------------------------------------------------
// Tracking/KalmanBallTracker.cs
// ------------------------------------------------------------
// Simple 6-state Kalman: [x, y, z, vx, vy, vz]
//
// Emgu.CV 4.9 matrix-writing notes:
//   Marshal.Copy to Mat.DataPointer does NOT reliably update KalmanFilter's
//   internal matrices — the native KalmanFilter struct may hold its own
//   pointers that differ from the managed Mat wrapper's DataPointer.
//
//   Reliable approaches:
//     • Diagonal matrices  → CvInvoke.SetIdentity(mat, scalar)
//     • Arbitrary matrices → build a Matrix<float> (has [r,c] indexer),
//                            then call matrix.Mat.CopyTo(target)
//     • Reading state      → copy to a Matrix<float> and read [r,c]
// ------------------------------------------------------------
using System;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using SportSimulator.Models;

namespace SportSimulator.Tracking
{
    public class KalmanBallTracker
    {
        private KalmanFilter? _kf;
        private bool _initialized;
        private long _lastTimestampUs;

        // Seeding used to buffer several real measurements and least-squares fit a
        // line through them (see git history) — needs several independent real
        // detections to average noise down. This rig's confirmed camera geometry
        // (114in height, ~33in forward, steep downward tilt — see notes/14-...md
        // and App/Program.cs) gives a real forward flight-tracking window of only
        // ~18 inches at the tee before the ball crosses the bottom of frame —
        // confirmed three independent ways this session (tilt-derived geometry,
        // a physical tape measurement in SpinView, and the mock's own live
        // behavior). At realistic shot speeds that's often only 1-2 real
        // detections total, nowhere near enough to average.
        //
        // Instead: the ball's REST position (address) is measured live — a
        // running average of real detections that agree with each other (i.e. the
        // ball is confirmed sitting still) — rather than assumed from a fixed
        // calibration constant. This handles a ball placed anywhere within frame,
        // not just an exact marked spot (a fixed-position version of this existed
        // briefly first, but broke the moment the ball wasn't placed exactly on
        // the calibrated tee — see conversation). There's no time pressure before
        // the shot the way there is during the ~1-2-frame flight window, so this
        // average can refine for as long as the ball actually sits still, getting
        // MORE accurate the longer it waits — genuinely more precise than a fixed
        // assumption, not just a fallback for one.
        //
        // The instant a real detection shows up meaningfully displaced from that
        // average, that's the shot starting: a single two-point delta (averaged
        // rest position/time -> this one noisy detection) gives a velocity
        // estimate with far less noise than a raw two-point delta between two
        // arbitrary noisy detections, and needs only ONE real detection after the
        // ball leaves rest — not several.
        //
        // Can't fully distinguish "the ball just became visible, already moving"
        // from "the ball was genuinely at rest and just moved" from a single
        // sample — both start the same way (first-ever detection, nothing to
        // compare against yet). This matters beyond just noise: a user carrying
        // or rolling the ball into position produces exactly this same pattern —
        // real, sample-to-sample motion that hasn't settled yet — and without a
        // minimum settling period, that placement motion would get seeded as a
        // "shot" the instant it looked displaced from wherever it was first seen.
        // MinRestSamplesForConfidence is a hard gate against that: a displacement
        // seen before the ball has been confirmed resting for at least this many
        // consecutive samples is treated as a NEW candidate rest position (still
        // being placed), not a shot — see Update(). Real players pause far longer
        // than this minimum (~25ms at 200fps) before actually striking the ball,
        // so it doesn't meaningfully delay detecting a genuine one.
        //
        // That gate alone doesn't catch a second, different case: the ball can
        // sit still for seconds (clearing the gate many times over) and then get
        // nudged a few cm to adjust its placement — not a shot, just a
        // repositioning. _minSpeedMps catches this the other way: below a
        // per-sport minimum implied speed, a displacement is treated the same as
        // a not-yet-settled one, regardless of how long the ball sat first (see
        // SportProfile.MinSpeedMps).
        //
        // Neither check can fully eliminate the ambiguity — a real hardware
        // impact trigger (IR break-beam/mic — see notes/04-hardware-triggering.md)
        // would, by supplying a known t0 independent of any position heuristic —
        // nothing else here would need to change to use it.
        private const float RestPositionToleranceM = 0.03f; // ball radius is a few cm; this must be well under one ball-width of drift

        // Minimum consecutive agreeing samples required before the ball is
        // considered genuinely at rest — see the gate in Update() and the class
        // comment above for why this exists (placement motion, not just noise).
        public const int MinRestSamplesForConfidence = 5;
        private float _restSumX, _restSumY, _restSumZ;
        private int _restSampleCount;
        private long _lastRestTimestampUs;

        public float RestX => _restSampleCount > 0 ? _restSumX / _restSampleCount : 0f;
        public float RestY => _restSampleCount > 0 ? _restSumY / _restSampleCount : 0f;
        public float RestZ => _restSampleCount > 0 ? _restSumZ / _restSampleCount : 0f;
        public int RestSampleCount => _restSampleCount;

        // The settling gate above (MinRestSamplesForConfidence) only catches
        // continuous motion that never stops — it doesn't catch the ball sitting
        // still for seconds and then getting nudged a few cm to adjust its
        // placement, which clears that gate easily and would otherwise look
        // exactly like a very slow "shot". Below this implied speed (position
        // delta / elapsed time), a displacement is treated as a repositioning
        // nudge — reset the candidate rest position, don't seed a shot — same
        // reasoning as the settling gate, just triggered by speed instead of
        // sample count. See SportProfile.MinSpeedMps for per-sport values and
        // why golf's is set far more cautiously than the others.
        private float _minSpeedMps;

        // The opposite tail: a single grossly-wrong detection (stereo mismatch,
        // a reflection/shadow that happens to pass the contour-area filter,
        // lighting flicker) can imply a velocity no real shot for this sport
        // could produce. Seeding now happens from just one post-rest detection,
        // so there's nothing else to average that one bad reading against —
        // above this implied speed, treat it the same as the too-slow case:
        // reset the candidate rest position rather than seed an obviously-wrong
        // "shot". Not yet observed happening in this session's testing (the
        // mock's synthetic noise doesn't produce this kind of outlier) — this is
        // cheap insurance against a plausible real-hardware failure mode, not a
        // fix for something confirmed to occur. See SportProfile.MaxSpeedMps.
        private float _maxSpeedMps;

        // Fallback dt for the very first Update/Predict call, when there's no prior
        // timestamp to diff against yet. Confirmed on-site 2026-07-22: SpinView
        // reports 199.76fps at default (full-frame) settings.
        private const float DefaultDt = 1f / 199.76f;

        // Guards against a degenerate transition matrix: MinDt covers duplicate/
        // non-monotonic timestamps (would otherwise give dt<=0), MaxDt caps a single
        // huge gap (e.g. a long coast) from producing an absurd position jump — real
        // coast-frame limits (SportProfile.KalmanCoastFrames) bound this in practice.
        private const float MinDt = 1e-4f;
        private const float MaxDt = 0.5f;

        public void Configure(SportProfile profile)
        {
            _kf = new KalmanFilter(6, 3, 0, DepthType.Cv32F);
            float q = profile.ProcessNoise;
            float r = profile.MeasurementNoise;
            _minSpeedMps = (float)profile.MinSpeedMps;
            _maxSpeedMps = (float)profile.MaxSpeedMps;

            SetTransitionMatrix(DefaultDt);

            SetMatrix(_kf.MeasurementMatrix, new float[,]
            {
                {1,0,0,0,0,0},
                {0,1,0,0,0,0},
                {0,0,1,0,0,0}
            });

            // CvInvoke.SetIdentity is the reliable way to set diagonal Mats
            CvInvoke.SetIdentity(_kf.ProcessNoiseCov,     new MCvScalar(q));
            CvInvoke.SetIdentity(_kf.MeasurementNoiseCov, new MCvScalar(r));
            CvInvoke.SetIdentity(_kf.ErrorCovPost,        new MCvScalar(1.0));

            _initialized = false;
            _restSumX = _restSumY = _restSumZ = 0f;
            _restSampleCount = 0;
        }

        // Position-only measurement update. timestampUs is the frame's own capture
        // timestamp — NOT wall-clock time of when this method happens to be called.
        // The transition matrix's dt is rebuilt from the real elapsed time since the
        // last call rather than assumed fixed, because callers don't call this once
        // per camera tick: SimulatorEngine.ProcessLoop batches a variable number of
        // frames per cycle depending on processing load, so a hardcoded dt here
        // systematically distorted the recovered velocity — see App/SimulatorEngine.cs
        // ProcessLoop and this class's git history for the incident that caught it
        // (mock ball moved far too slowly because reported speed was ~10x too low).
        public (float x, float y, float z, float vx, float vy, float vz) Update(
            float mx, float my, float mz, long timestampUs)
        {
            if (_kf == null) return (mx, my, mz, 0, 0, 0);

            if (!_initialized)
            {
                if (_restSampleCount == 0)
                {
                    // First-ever detection: nothing to compare against yet. Seed
                    // the running average with it and wait for the next one to
                    // tell us whether the ball is sitting still or already moving.
                    _restSumX = mx; _restSumY = my; _restSumZ = mz;
                    _restSampleCount = 1;
                    _lastRestTimestampUs = timestampUs;
                    LastState = (mx, my, mz, 0, 0, 0);
                    return LastState;
                }

                float avgX = RestX, avgY = RestY, avgZ = RestZ;
                float dx = mx - avgX, dy = my - avgY, dz = mz - avgZ;
                bool stillAtRest = MathF.Sqrt(dx * dx + dy * dy + dz * dz) <= RestPositionToleranceM;

                if (stillAtRest)
                {
                    // Fold this sample into the running average — more samples
                    // while genuinely at rest means a lower-noise reference, not
                    // just a fallback: there's no time pressure before the shot
                    // the way there is during the ~1-2-frame flight window.
                    _restSumX += mx; _restSumY += my; _restSumZ += mz;
                    _restSampleCount++;
                    _lastRestTimestampUs = timestampUs;
                    LastState = (RestX, RestY, RestZ, 0, 0, 0);
                    return LastState;
                }

                if (_restSampleCount < MinRestSamplesForConfidence)
                {
                    // Displaced, but the ball was never actually confirmed
                    // resting for long enough to trust — this is what a user
                    // carrying or rolling the ball into position looks like, not
                    // a shot: real motion, sample to sample, that just hasn't
                    // settled yet. Treat this as a NEW candidate rest position
                    // instead of a shot, and keep waiting for it to actually stop.
                    // Real players pause far longer than MinRestSamplesForConfidence
                    // (~25ms at 200fps) before actually striking the ball, so this
                    // doesn't meaningfully delay detecting a genuine shot.
                    _restSumX = mx; _restSumY = my; _restSumZ = mz;
                    _restSampleCount = 1;
                    _lastRestTimestampUs = timestampUs;
                    LastState = (mx, my, mz, 0, 0, 0);
                    return LastState;
                }

                // Displaced after being confirmed resting for a real stretch —
                // but still check whether the implied speed is even plausible
                // for a real shot (see _minSpeedMps's comment): a hand nudging
                // the ball to adjust its placement clears the settling gate
                // above easily (the ball may have sat for seconds), and without
                // this check would look exactly like a very slow "shot".
                float restDt = ClampDt((timestampUs - _lastRestTimestampUs) / 1_000_000f);
                float impliedSpeed = MathF.Sqrt(dx * dx + dy * dy + dz * dz) / restDt;

                if (impliedSpeed < _minSpeedMps)
                {
                    // Too slow to be a real shot for this sport — a repositioning
                    // nudge, not a strike. Treat it as a new candidate rest
                    // position, same as the settling-gate case above.
                    _restSumX = mx; _restSumY = my; _restSumZ = mz;
                    _restSampleCount = 1;
                    _lastRestTimestampUs = timestampUs;
                    LastState = (mx, my, mz, 0, 0, 0);
                    return LastState;
                }

                // _maxSpeedMps <= 0 means "not configured" — don't treat that as
                // "reject everything faster than zero" (see _maxSpeedMps's comment).
                if (_maxSpeedMps > 0 && impliedSpeed > _maxSpeedMps)
                {
                    // Faster than any real shot for this sport can be — almost
                    // certainly a bad detection (stereo mismatch, a reflection
                    // that passed the contour filter), not a real ball. Treat it
                    // the same as the too-slow case: reset the candidate rest
                    // position and wait for a cleaner reading, rather than seed
                    // an obviously-wrong "shot".
                    _restSumX = mx; _restSumY = my; _restSumZ = mz;
                    _restSampleCount = 1;
                    _lastRestTimestampUs = timestampUs;
                    LastState = (mx, my, mz, 0, 0, 0);
                    return LastState;
                }

                // Confirmed resting for a real stretch, and within a plausible
                // speed range for a genuine shot. Seed directly from a two-point
                // delta (see class comment): works the same whether the average
                // is built from exactly the minimum or much longer — noisier
                // nearer the minimum, but never wrong.
                SeedFromRestAndDetection(mx, my, mz, timestampUs, avgX, avgY, avgZ);
                return LastState;
            }

            SetTransitionMatrix(RealDt(timestampUs));
            _kf.Predict();

            using var meas = new Matrix<float>(3, 1);
            meas[0, 0] = mx; meas[1, 0] = my; meas[2, 0] = mz;

            CacheState(_kf.Correct(meas.Mat));
            _lastTimestampUs = timestampUs;
            return LastState;
        }

        /// <summary>Predict-only step (Tier 4 coasting). See Update's comment on timestampUs.</summary>
        public (float x, float y, float z, float vx, float vy, float vz) Predict(long timestampUs)
        {
            if (_kf == null || !_initialized) return LastState;
            SetTransitionMatrix(RealDt(timestampUs));
            CacheState(_kf.Predict());
            _lastTimestampUs = timestampUs;
            return LastState;
        }

        // Seeds directly from a single two-point delta: the measured (averaged,
        // low-noise once several samples have accumulated) rest position/timestamp,
        // to this one real (noisy) detection. Position is trusted at face value
        // (it's the only real measurement available); velocity is the delta
        // divided by real elapsed time since the ball was last seen at rest.
        // Commits with no extra Predict() step, since this detection's timestamp
        // IS "now" — advancing again would overshoot.
        private void SeedFromRestAndDetection(float mx, float my, float mz, long timestampUs,
                                               float restX, float restY, float restZ)
        {
            float dt = ClampDt((timestampUs - _lastRestTimestampUs) / 1_000_000f);
            float vx = (mx - restX) / dt;
            float vy = (my - restY) / dt;
            float vz = (mz - restZ) / dt;

            using var init = new Matrix<float>(6, 1);
            init[0, 0] = mx; init[1, 0] = my; init[2, 0] = mz;
            init[3, 0] = vx; init[4, 0] = vy; init[5, 0] = vz;
            init.Mat.CopyTo(_kf!.StatePost);

            SetTransitionMatrix(dt);
            _initialized = true;
            CacheState(_kf.StatePost);
            _lastTimestampUs = timestampUs;
        }

        private float RealDt(long timestampUs) => ClampDt((timestampUs - _lastTimestampUs) / 1_000_000f);

        private static float ClampDt(float dt) => Math.Max(MinDt, Math.Min(MaxDt, dt));

        private void SetTransitionMatrix(float dt)
        {
            SetMatrix(_kf!.TransitionMatrix, new float[,]
            {
                {1,0,0,dt,0,0},
                {0,1,0,0,dt,0},
                {0,0,1,0,0,dt},
                {0,0,0,1,0,0},
                {0,0,0,0,1,0},
                {0,0,0,0,0,1}
            });
        }

        public (float x, float y, float z, float vx, float vy, float vz) LastState
        { get; private set; }

        // True once the rest-position velocity seed has committed (see
        // SeedFromRestAndDetection). Callers feeding this tracker single-camera
        // monocular estimates that fall back to LastState.z as their own depth
        // reference (SimulatorEngine's Tier 3) should check this first — before
        // it's true, LastState.z is meaningless (0), and treating it as a real
        // depth would feed a fabricated point into the seed.
        public bool HasFix => _initialized;

        // ── Helpers ──────────────────────────────────────────────────────────

        private void CacheState(Mat state)
        {
            // state is a 6x1 CV_32F Mat returned by Predict/Correct
            using var m = new Matrix<float>(state.Rows, 1);
            state.CopyTo(m.Mat);
            LastState = (m[0,0], m[1,0], m[2,0], m[3,0], m[4,0], m[5,0]);
        }

        // Build a Matrix<float> from a 2-D array and copy it into a target Mat.
        private static void SetMatrix(Mat target, float[,] vals)
        {
            int rows = vals.GetLength(0), cols = vals.GetLength(1);
            using var m = new Matrix<float>(rows, cols);
            for (int r = 0; r < rows; r++)
                for (int c = 0; c < cols; c++)
                    m[r, c] = vals[r, c];
            m.Mat.CopyTo(target);
        }
    }
}
