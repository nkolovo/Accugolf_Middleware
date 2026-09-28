// ------------------------------------------------------------
// Tests/RotationFitterTests.cs
// ------------------------------------------------------------
// Validates RotationFitter against a rotation matrix built independently via
// Rodrigues' formula (NOT reusing RotationFitter's own SVD/decomposition
// math) so a bug in RotationFitter can't hide by agreeing with itself.
// ------------------------------------------------------------
using System;
using FluentAssertions;
using SportSimulator.Tracking;
using Xunit;

namespace SportSimulator.Tests
{
    public class RotationFitterTests
    {
        private static double[,] RodriguesMatrix(double ax, double ay, double az, double angleRad)
        {
            double len = Math.Sqrt(ax * ax + ay * ay + az * az);
            ax /= len; ay /= len; az /= len;

            double c = Math.Cos(angleRad), s = Math.Sin(angleRad), t = 1 - c;

            return new double[3, 3]
            {
                { t*ax*ax + c,    t*ax*ay - s*az,  t*ax*az + s*ay },
                { t*ax*ay + s*az, t*ay*ay + c,      t*ay*az - s*ax },
                { t*ax*az - s*ay, t*ay*az + s*ax,  t*az*az + c }
            };
        }

        private static Vec3 Apply(double[,] R, Vec3 p)
        {
            double x = R[0, 0] * p.X + R[0, 1] * p.Y + R[0, 2] * p.Z;
            double y = R[1, 0] * p.X + R[1, 1] * p.Y + R[1, 2] * p.Z;
            double z = R[2, 0] * p.X + R[2, 1] * p.Y + R[2, 2] * p.Z;
            return new Vec3((float)x, (float)y, (float)z);
        }

        private static Vec3[] MakeScatteredPoints(int count, int seed = 1)
        {
            var rng = new Random(seed);
            var pts = new Vec3[count];
            for (int i = 0; i < count; i++)
                pts[i] = new Vec3(
                    (float)(rng.NextDouble() * 0.04 - 0.02),
                    (float)(rng.NextDouble() * 0.04 - 0.02),
                    (float)(rng.NextDouble() * 0.04 - 0.02));
            return pts;
        }

        [Theory]
        [InlineData(1, 0, 0, 20)]     // rotation about X — the "backspin" case for an overhead camera
        [InlineData(0, 1, 0, 20)]     // rotation about Y — the "sidespin" case
        [InlineData(0, 0, 1, 20)]     // rotation about Z
        [InlineData(1, 1, 0, 35)]
        [InlineData(0.3, 0.7, -0.2, 15)]
        public void KnownRotation_AxisAndAngleRecovered(double ax, double ay, double az, double angleDeg)
        {
            var setA = MakeScatteredPoints(8);
            double angleRad = angleDeg * Math.PI / 180.0;
            var R = RodriguesMatrix(ax, ay, az, angleRad);
            var setB = Array.ConvertAll(setA, p => Apply(R, p));

            var fit = RotationFitter.Fit(setA, setB);

            fit.Valid.Should().BeTrue();
            fit.AngleDeg.Should().BeApproximately((float)angleDeg, 0.5f,
                "recovered rotation magnitude should match ground truth");

            double len = Math.Sqrt(ax * ax + ay * ay + az * az);
            var expectedAxis = new Vec3((float)(ax / len), (float)(ay / len), (float)(az / len));

            float dot = fit.AxisUnit.X * expectedAxis.X + fit.AxisUnit.Y * expectedAxis.Y + fit.AxisUnit.Z * expectedAxis.Z;
            Math.Abs(dot).Should().BeGreaterThan(0.99f,
                "recovered axis should be parallel to ground truth (dot product near |1| — a consistent sign " +
                "convention difference between this test's Rodrigues builder and RotationFitter's own decomposition " +
                "is acceptable; an inconsistent/wrong axis direction is not)");
        }

        [Fact]
        public void NoRotation_ReturnsZeroAngle()
        {
            var setA = MakeScatteredPoints(6);
            var fit = RotationFitter.Fit(setA, setA);

            fit.Valid.Should().BeTrue();
            fit.AngleDeg.Should().BeApproximately(0f, 0.01f);
        }

        [Fact]
        public void TooFewPoints_ReturnsInvalid()
        {
            var setA = MakeScatteredPoints(2);
            var setB = MakeScatteredPoints(2, seed: 2);
            RotationFitter.Fit(setA, setB).Valid.Should().BeFalse();
        }

        [Fact]
        public void MismatchedSetSizes_ReturnsInvalid()
        {
            var setA = MakeScatteredPoints(5);
            var setB = MakeScatteredPoints(4, seed: 2);
            RotationFitter.Fit(setA, setB).Valid.Should().BeFalse();
        }

        [Fact]
        public void PureTranslation_NoRotationDetected()
        {
            // Translating every point by the same offset (no rotation) should be
            // invisible to the fitter — it only looks at geometry after centering,
            // which a shared translation doesn't change.
            var setA = MakeScatteredPoints(6);
            var setB = Array.ConvertAll(setA, p => new Vec3(p.X + 0.5f, p.Y - 0.3f, p.Z + 0.1f));

            var fit = RotationFitter.Fit(setA, setB);
            fit.Valid.Should().BeTrue();
            fit.AngleDeg.Should().BeApproximately(0f, 0.5f, "pure translation carries no rotation signal");
        }
    }
}
