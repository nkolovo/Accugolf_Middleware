// ------------------------------------------------------------
// Tests/SpinEstimatorTests.cs
// ------------------------------------------------------------
// Validates SpinEstimator's rotation-search/correlation logic against
// synthetic crops with a KNOWN ground-truth rotation, generated with the
// same WarpAffine/GetRotationMatrix2D call the estimator uses internally.
// This proves the search-and-score math is correct — it does NOT validate
// real-world texture tracking (dimples/panels/seams under real lighting
// and motion blur), which can only be checked against actual footage.
// ------------------------------------------------------------
using System;
using System.Drawing;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using FluentAssertions;
using SportSimulator.Vision;
using Xunit;

namespace SportSimulator.Tests
{
    public class SpinEstimatorTests
    {
        private const int Size = 48;
        private const float Dt120Fps = 1f / 120f;

        // A disc scattered with small "dimples" at varying radius/angle — stands in
        // for real dimple/panel/seam texture. A single small feature on an
        // otherwise rotation-invariant plain disc (a circle looks the same at every
        // angle) gives the correlation search too weak a signal to discriminate
        // between candidate angles; this spreads asymmetric texture across the
        // whole crop so rotation actually produces a sharp correlation peak.
        private static byte[] MakeAsymmetricCrop()
        {
            using var mat = new Mat(Size, Size, DepthType.Cv8U, 1);
            mat.SetTo(new MCvScalar(60));
            CvInvoke.Circle(mat, new Point(Size / 2, Size / 2), Size / 2 - 2, new MCvScalar(140), -1);

            var rng = new Random(7); // seeded — deterministic across runs
            var center = new PointF(Size / 2f, Size / 2f);
            for (int i = 0; i < 12; i++)
            {
                double angle = rng.NextDouble() * 2 * Math.PI;
                double r = 4 + rng.NextDouble() * (Size / 2f - 8);
                int px = (int)(center.X + r * Math.Cos(angle));
                int py = (int)(center.Y + r * Math.Sin(angle));
                int shade = rng.Next(0, 2) == 0 ? 220 : 15;
                CvInvoke.Circle(mat, new Point(px, py), 2, new MCvScalar(shade), -1);
            }

            return MatToBytes(mat);
        }

        private static byte[] Rotate(byte[] crop, float deg)
        {
            using var mat = BytesToMat(crop);
            using var rotated = new Mat();
            var center = new PointF(Size / 2f, Size / 2f);
            using var rotMatrix = new Mat();
            CvInvoke.GetRotationMatrix2D(center, deg, 1.0, rotMatrix);
            CvInvoke.WarpAffine(mat, rotated, rotMatrix, mat.Size);
            return MatToBytes(rotated);
        }

        private static Mat BytesToMat(byte[] data)
        {
            var m = new Mat(Size, Size, DepthType.Cv8U, 1);
            m.SetTo(data);
            return m;
        }

        private static byte[] MatToBytes(Mat mat)
        {
            using var m = new Matrix<byte>(Size, Size);
            mat.CopyTo(m.Mat);
            var bytes = new byte[Size * Size];
            for (int r = 0; r < Size; r++)
                for (int c = 0; c < Size; c++)
                    bytes[r * Size + c] = m[r, c];
            return bytes;
        }

        // ── No rotation ──────────────────────────────────────────────────────────

        [Fact]
        public void IdenticalCrops_NoRotationDetected()
        {
            var crop = MakeAsymmetricCrop();
            var result = new SpinEstimator().Estimate(crop, crop, Size, Dt120Fps);

            result.Valid.Should().BeFalse("identical crops have no rotation to find — 0° is already the best match");
        }

        // ── Known rotation ───────────────────────────────────────────────────────
        // Ground truths are on the estimator's 2° search grid, and cropB is built
        // with the exact same WarpAffine call the estimator uses to test candidate
        // angles — so the correct candidate should reproduce cropB almost exactly
        // and clearly win the correlation search.

        [Theory]
        [InlineData(20f)]
        [InlineData(-20f)]
        [InlineData(30f)]
        public void KnownRotation_IsRecoveredPrecisely(float groundTruthDeg)
        {
            var cropA = MakeAsymmetricCrop();
            var cropB = Rotate(cropA, groundTruthDeg);

            var result = new SpinEstimator().Estimate(cropA, cropB, Size, Dt120Fps);

            result.Valid.Should().BeTrue("a clearly rotated asymmetric pattern should register as spin");
            result.AngleDeg.Should().BeApproximately(groundTruthDeg, 2.5f,
                "recovered angle should land on (or immediately next to) the correct 2° search grid point");
        }

        [Fact]
        public void KnownRotation_RpmMatchesAngleOverTime()
        {
            const float groundTruthDeg = 24f; // 24° / (1/120s) / 6 = 480 rpm — mid soccer range
            var cropA = MakeAsymmetricCrop();
            var cropB = Rotate(cropA, groundTruthDeg);

            var result = new SpinEstimator().Estimate(cropA, cropB, Size, Dt120Fps);

            result.Valid.Should().BeTrue();
            float expectedRpm = Math.Abs(groundTruthDeg) / Dt120Fps / 6f;
            result.Rpm.Should().BeApproximately(expectedRpm, 40f, "rpm = angle / dt / 6");
        }

        // ── Robustness ───────────────────────────────────────────────────────────

        [Fact]
        public void MismatchedCropSize_ReturnsInvalid_DoesNotThrow()
        {
            var cropA = new byte[Size * Size];
            var cropB = new byte[10]; // wrong length

            var act = () => new SpinEstimator().Estimate(cropA, cropB, Size, Dt120Fps);
            act.Should().NotThrow();

            new SpinEstimator().Estimate(cropA, cropB, Size, Dt120Fps).Valid.Should().BeFalse();
        }

        [Fact]
        public void ZeroOrNegativeDt_ReturnsInvalid()
        {
            var crop = MakeAsymmetricCrop();
            new SpinEstimator().Estimate(crop, crop, Size, 0f).Valid.Should().BeFalse();
            new SpinEstimator().Estimate(crop, crop, Size, -0.01f).Valid.Should().BeFalse();
        }
    }
}
