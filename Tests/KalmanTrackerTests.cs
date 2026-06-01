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
            var (x, y, z, _, _, _) = tracker.Update(1f, 2f, 3f);

            // First update seeds the filter — position should be very close to input
            x.Should().BeApproximately(1f, 0.5f);
            y.Should().BeApproximately(2f, 0.5f);
            z.Should().BeApproximately(3f, 0.5f);
        }

        [Fact]
        public void ConstantVelocityTrajectory_PositionTracksAndVelocityGrows()
        {
            // A constant-velocity Kalman filter with position-only measurements converges
            // velocity slowly — typically 200-400 frames to reach steady state.
            // What we CAN assert within 60 frames:
            //   1. Position estimate tracks the measured position closely.
            //   2. Velocity estimate has the correct sign and is growing.
            //   3. Velocity after 60 frames is greater than after 10 frames
            //      (i.e. the filter is learning, not stuck at zero).
            var tracker = MakeTracker();
            float dt = 1f / 120f;
            float vxTrue = 10f;

            for (int i = 0; i < 10; i++)
                tracker.Update(i * dt * vxTrue, 0f, 5f);
            var (_, _, _, vxEarly, _, _) = tracker.LastState;

            for (int i = 10; i < 60; i++)
                tracker.Update(i * dt * vxTrue, 0f, 5f);

            var (px, _, _, vxLate, _, _) = tracker.LastState;
            float expectedX = 59 * dt * vxTrue;

            // With unconverged velocity the filter lags; 0.25m is realistic at 60 frames
            px.Should().BeApproximately(expectedX, 0.25f,
                "position estimate should be within 25cm of measured position");
            vxLate.Should().BeGreaterThan(0f,
                "velocity should be positive for rightward motion");
            vxLate.Should().BeGreaterThan(vxEarly,
                "velocity estimate should grow as the filter accumulates evidence");
        }

        [Fact]
        public void Predict_WithoutMeasurement_AdvancesPosition()
        {
            var tracker = MakeTracker();
            tracker.Update(0f, 0f, 5f);
            tracker.Update(0.083f, 0f, 5.4f); // ~10 m/s along x, ~48 m/s along z

            var (x0, _, _, _, _, _) = tracker.LastState;
            tracker.Predict();
            var (x1, _, _, _, _, _) = tracker.LastState;

            x1.Should().BeGreaterThan(x0, "predict-only step should advance position");
        }

        [Fact]
        public void MultiplePredicts_DoNotExplode()
        {
            // Coasting should never produce NaN or very large values
            var tracker = MakeTracker();
            tracker.Update(0f, 0f, 5f);
            tracker.Update(0.083f, 0f, 5.4f);

            for (int i = 0; i < 10; i++) tracker.Predict();

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
            tracker.Update(100f, 50f, 200f);
            tracker.Update(101f, 50f, 205f);

            // Reconfigure for a different sport
            tracker.Configure(new SportProfile
            {
                SportId = "golf", ProcessNoise = 0.005f, MeasurementNoise = 0.07f
            });
            var (x, y, z, _, _, _) = tracker.Update(0f, 0f, 1f);

            x.Should().BeApproximately(0f, 0.5f, "filter should be reset after Configure");
        }
    }
}
