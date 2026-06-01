// ------------------------------------------------------------
// Tests/KalmanTrackerAdditionalTests.cs
// ------------------------------------------------------------
// Covers edge-cases not addressed in KalmanTrackerTests.cs:
//   - Predict() before any Update() (default state, no-op)
//   - Negative-direction velocity sign correctness
//   - Stationary ball converges to near-zero velocity
// ------------------------------------------------------------
using FluentAssertions;
using SportSimulator.Models;
using SportSimulator.Tracking;
using Xunit;

namespace SportSimulator.Tests
{
    public class KalmanTrackerAdditionalTests
    {
        private static KalmanBallTracker MakeTracker(float q = 0.01f, float r = 0.1f)
        {
            var tracker = new KalmanBallTracker();
            tracker.Configure(new SportProfile
            {
                SportId          = "test",
                ProcessNoise     = q,
                MeasurementNoise = r,
                KalmanCoastFrames = 5
            });
            return tracker;
        }

        [Fact]
        public void Predict_BeforeAnyUpdate_DoesNotThrow()
        {
            // SimulatorEngine may call Predict() before the first detection arrives.
            // The tracker should return the default zero state without throwing.
            var tracker = MakeTracker();
            var act = () => tracker.Predict();

            act.Should().NotThrow();
        }

        [Fact]
        public void Predict_BeforeAnyUpdate_LastStateHasNoNaN()
        {
            var tracker = MakeTracker();
            tracker.Predict();
            var (x, y, z, vx, vy, vz) = tracker.LastState;

            float.IsNaN(x).Should().BeFalse();
            float.IsNaN(y).Should().BeFalse();
            float.IsNaN(z).Should().BeFalse();
            float.IsNaN(vx).Should().BeFalse();
            float.IsNaN(vy).Should().BeFalse();
            float.IsNaN(vz).Should().BeFalse();
        }

        [Fact]
        public void NegativeXVelocity_HasCorrectSign()
        {
            // Ball moving left (negative X) — Kalman velocity must be negative.
            var tracker = MakeTracker();
            float dt    = 1f / 120f;
            float vTrue = -10f;

            for (int i = 0; i < 60; i++)
                tracker.Update(i * dt * vTrue, 0f, 5f);

            var (_, _, _, vx, _, _) = tracker.LastState;
            vx.Should().BeLessThan(0f, "velocity must be negative for leftward motion");
        }

        [Fact]
        public void NegativeZVelocity_HasCorrectSign()
        {
            // Ball moving toward the cameras (negative Z) — uncommon but must not flip sign.
            var tracker = MakeTracker();
            float dt    = 1f / 120f;
            float vTrue = -5f;

            for (int i = 0; i < 60; i++)
                tracker.Update(0f, 0f, 5f + i * dt * vTrue);

            var (_, _, _, _, _, vz) = tracker.LastState;
            vz.Should().BeLessThan(0f);
        }

        [Fact]
        public void StationaryBall_VelocityConvergesToZero()
        {
            // A ball sitting still should eventually drive all velocity estimates to ~0.
            // The constant-velocity Kalman converges slowly — 60 frames is enough for
            // the velocity to be much closer to zero than the initial uncertainty.
            var tracker = MakeTracker();
            for (int i = 0; i < 60; i++)
                tracker.Update(1f, 2f, 3f);

            var (_, _, _, vx, vy, vz) = tracker.LastState;
            vx.Should().BeApproximately(0f, 0.5f);
            vy.Should().BeApproximately(0f, 0.5f);
            vz.Should().BeApproximately(0f, 0.5f);
        }

        [Fact]
        public void Update_AfterPredict_DoesNotThrow()
        {
            // Alternating Predict / Update should not corrupt the filter state.
            var tracker = MakeTracker();
            tracker.Update(0f, 0f, 1f);
            tracker.Predict();
            var act = () => tracker.Update(0.1f, 0f, 1.05f);
            act.Should().NotThrow();
        }

        [Fact]
        public void LastState_UpdatedAfterEachCall()
        {
            // LastState should reflect the most recent Predict or Update result.
            var tracker = MakeTracker();
            tracker.Update(1f, 0f, 5f);
            var state1 = tracker.LastState;

            tracker.Update(2f, 0f, 5f);
            var state2 = tracker.LastState;

            state2.Should().NotBe(state1, "LastState must change after a new Update");
        }
    }
}
