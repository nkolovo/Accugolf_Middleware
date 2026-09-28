// ------------------------------------------------------------
// Tests/KalmanTrackerTests.cs
// ------------------------------------------------------------
using FluentAssertions;
using SportSimulator.Models;
using SportSimulator.Tracking;
using Xunit;

namespace SportSimulator.Tests
{
    public class KalmanTrackerTests
    {
        private static KalmanBallTracker MakeTracker(float processNoise = 0.01f,
                                                      float measureNoise  = 0.1f)
        {
            var tracker = new KalmanBallTracker();
            tracker.Configure(new SportProfile
            {
                SportId          = "test",
                ProcessNoise     = processNoise,
                MeasurementNoise = measureNoise,
                KalmanCoastFrames = 5
            });
            return tracker;
        }

        [Fact]
        public void FirstUpdate_ReturnsInputPosition()
        {
            var tracker = MakeTracker();
            var (x, y, z, _, _, _) = tracker.Update(1f, 2f, 3f, 0);

            // First update seeds the filter — position should be very close to input
            x.Should().BeApproximately(1f, 0.5f);
            y.Should().BeApproximately(2f, 0.5f);
            z.Should().BeApproximately(3f, 0.5f);
        }

        [Fact]
        public void ConstantVelocityTrajectory_PositionTracksAndVelocityConvergesEarly()
        {
            // KalmanBallTracker seeds its initial velocity from a rest-position delta
            // (see its class comment) rather than starting at zero — a real capture
            // window doesn't give a fast-moving ball hundreds of frames to "learn" its
            // own speed from scratch. So for a truly constant-velocity trajectory,
            // velocity should already be close to true almost immediately, and stay
            // close — not slowly climb over time.
            var tracker = MakeTracker();
            float dt = 1f / 120f;
            float vxTrue = 10f;

            // Ball sits at rest before the shot — every real shot starts this way
            // (see KalmanBallTracker.Update's settling gate, MinRestSamplesForConfidence)
            // — feed enough agreeing samples to clear it before any real motion.
            const int restSamples = 5;
            for (int i = 0; i < restSamples; i++)
                tracker.Update(0f, 0f, 5f, (long)((i - restSamples) * dt * 1_000_000f));

            for (int i = 0; i < 10; i++)
                tracker.Update(i * dt * vxTrue, 0f, 5f, (long)(i * dt * 1_000_000f));
            var (_, _, _, vxEarly, _, _) = tracker.LastState;

            for (int i = 10; i < 60; i++)
                tracker.Update(i * dt * vxTrue, 0f, 5f, (long)(i * dt * 1_000_000f));

            var (px, _, _, vxLate, _, _) = tracker.LastState;
            float expectedX = 59 * dt * vxTrue;

            px.Should().BeApproximately(expectedX, 0.25f,
                "position estimate should be within 25cm of measured position");
            vxEarly.Should().BeApproximately(vxTrue, 0.1f,
                "rest-position seeding should recover the true velocity almost immediately, not after many frames");
            vxLate.Should().BeApproximately(vxTrue, 0.1f,
                "velocity estimate should stay close to true — a perfectly constant-velocity trajectory has nothing left to converge on");
        }

        [Fact]
        public void Predict_WithoutMeasurement_AdvancesPosition()
        {
            var tracker = MakeTracker();
            // Ball sits at rest before the shot (see MinRestSamplesForConfidence's
            // settling gate in KalmanBallTracker.Update), then moves at a steady
            // ~10 m/s along x, ~48 m/s along z so the seed lands on a real, moving
            // state once it's displaced from rest.
            const long dtUs = 5006;
            const int restSamples = 5;
            for (int i = 0; i < restSamples; i++)
                tracker.Update(0f, 0f, 5f, (i - restSamples) * dtUs);
            for (int i = 0; i < 8; i++)
                tracker.Update(i * 0.05006f, 0f, 5f + i * 0.24f, i * dtUs);

            var (x0, _, _, _, _, _) = tracker.LastState;
            tracker.Predict(8 * dtUs + dtUs);
            var (x1, _, _, _, _, _) = tracker.LastState;

            x1.Should().BeGreaterThan(x0, "predict-only step should advance position");
        }

        [Fact]
        public void MultiplePredicts_DoNotExplode()
        {
            // Coasting should never produce NaN or very large values
            var tracker = MakeTracker();
            const long dtUs = 5006;
            const int restSamples = 5;
            for (int i = 0; i < restSamples; i++)
                tracker.Update(0f, 0f, 5f, (i - restSamples) * dtUs);
            for (int i = 0; i < 8; i++)
                tracker.Update(i * 0.05006f, 0f, 5f + i * 0.24f, i * dtUs);

            for (int i = 0; i < 10; i++) tracker.Predict(8 * dtUs + (i + 1) * dtUs);

            var (x, y, z, vx, vy, vz) = tracker.LastState;
            float[] vals = { x, y, z, vx, vy, vz };
            foreach (var v in vals)
            {
                float.IsNaN(v).Should().BeFalse("coasting should not produce NaN");
                System.Math.Abs(v).Should().BeLessThan(1000f, "coasting should not explode");
            }
        }

        [Fact]
        public void Configure_ResetsFilter()
        {
            // After reconfigure the filter should accept a new trajectory cleanly
            var tracker = MakeTracker();
            tracker.Update(100f, 50f, 200f, 0);
            tracker.Update(101f, 50f, 205f, 5006);

            // Reconfigure for a different sport
            tracker.Configure(new SportProfile
            {
                SportId = "golf", ProcessNoise = 0.005f, MeasurementNoise = 0.07f
            });
            var (x, y, z, _, _, _) = tracker.Update(0f, 0f, 1f, 0);

            x.Should().BeApproximately(0f, 0.5f, "filter should be reset after Configure");
        }

        // ── Speed-based shot/nudge gates (MinSpeedMps / MaxSpeedMps) ────────────

        private static KalmanBallTracker MakeTrackerWithSpeedGates(double minSpeedMps, double maxSpeedMps)
        {
            var tracker = new KalmanBallTracker();
            tracker.Configure(new SportProfile
            {
                SportId = "test", ProcessNoise = 0.01f, MeasurementNoise = 0.1f,
                KalmanCoastFrames = 5, MinSpeedMps = minSpeedMps, MaxSpeedMps = maxSpeedMps
            });
            return tracker;
        }

        // Feeds enough agreeing samples to clear MinRestSamplesForConfidence,
        // ending at timestamp -dtUs so the next Update at t=0 measures a clean
        // one-frame-interval dt against it.
        private static void SettleAtRest(KalmanBallTracker tracker, float x, float y, float z, long dtUs)
        {
            for (int i = 0; i < KalmanBallTracker.MinRestSamplesForConfidence; i++)
                tracker.Update(x, y, z, (i - KalmanBallTracker.MinRestSamplesForConfidence) * dtUs);
        }

        [Fact]
        public void Displacement_BelowMinSpeed_TreatedAsRepositionNotShot()
        {
            // Soccer-like gates (3-60 m/s). A 1cm shift over one ~5ms frame
            // interval implies ~2 m/s — below the 3 m/s floor, so this is a
            // repositioning nudge, not a shot (see KalmanBallTracker.Update).
            var tracker = MakeTrackerWithSpeedGates(minSpeedMps: 3, maxSpeedMps: 60);
            const long dtUs = 5006;
            SettleAtRest(tracker, 0f, 0f, 5f, dtUs);

            tracker.Update(0.01f, 0f, 5f, 0);

            tracker.HasFix.Should().BeFalse("a slow displacement should be treated as a repositioning nudge, not a shot");
        }

        [Fact]
        public void Displacement_AboveMaxSpeed_TreatedAsBadDetectionNotShot()
        {
            // A 1m jump over one ~5ms frame interval implies ~200 m/s — far
            // beyond any real soccer shot, so this is treated as a bad
            // detection (stereo mismatch, reflection, etc.), not a real shot.
            var tracker = MakeTrackerWithSpeedGates(minSpeedMps: 3, maxSpeedMps: 60);
            const long dtUs = 5006;
            SettleAtRest(tracker, 0f, 0f, 5f, dtUs);

            tracker.Update(1f, 0f, 5f, 0);

            tracker.HasFix.Should().BeFalse("an implausibly fast displacement should be treated as a bad detection, not a real shot");
        }

        [Fact]
        public void Displacement_WithinSpeedRange_SeedsFixNormally()
        {
            // Confirms the gates don't also block a genuine shot: ~0.1m over
            // ~5ms implies ~20 m/s, a realistic soccer shot speed.
            var tracker = MakeTrackerWithSpeedGates(minSpeedMps: 3, maxSpeedMps: 60);
            const long dtUs = 5006;
            SettleAtRest(tracker, 0f, 0f, 5f, dtUs);

            tracker.Update(0.1f, 0f, 5f, 0);

            tracker.HasFix.Should().BeTrue("a realistic shot speed should seed a fix");
            var (_, _, _, vx, _, _) = tracker.LastState;
            vx.Should().BeApproximately(19.976f, 0.5f);
        }

        [Fact]
        public void SlowNudgeFollowedByRealShot_NudgeRejectedThenShotSeedsFromNewPosition()
        {
            // Matches the scenario this gate was built for: someone places the
            // ball, it sits for a while, then gets nudged a couple cm to adjust
            // its placement (rejected — too slow), then genuinely gets struck
            // from that new spot a moment later (should seed normally).
            var tracker = MakeTrackerWithSpeedGates(minSpeedMps: 3, maxSpeedMps: 60);
            const long dtUs = 5006;
            SettleAtRest(tracker, 0f, 0f, 5f, dtUs);

            tracker.Update(0.02f, 0f, 5f, 0);
            tracker.HasFix.Should().BeFalse("the nudge itself should not be treated as a shot");

            // Ball settles again at its new spot...
            for (int i = 1; i <= KalmanBallTracker.MinRestSamplesForConfidence; i++)
                tracker.Update(0.02f, 0f, 5f, i * dtUs);

            // ...then actually gets struck, fast, from the new rest position.
            tracker.Update(0.12f, 0f, 5f, (KalmanBallTracker.MinRestSamplesForConfidence + 1) * dtUs);

            tracker.HasFix.Should().BeTrue("a real shot after the nudge should still seed correctly");
        }
    }
}
