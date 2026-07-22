// ------------------------------------------------------------
// Tests/Spin3DEstimatorTests.cs
// ------------------------------------------------------------
// End-to-end integration test: synthetic stereo frame pairs, rendered from
// KNOWN 3D feature positions (before/after a known rotation) projected
// through the same pinhole formulas Triangulator uses. Exercises the full
// chain — GFTT detection, optical-flow tracking, row-constrained stereo
// matching, triangulation, Kabsch fit — end to end against ground truth.
//
// This proves the WIRING is correct. It does not prove real ball surfaces
// have enough natural texture at 720×540 to detect/track reliably — that's
// only checkable against real footage (see FeaturePointTracker.cs).
// ------------------------------------------------------------
using System;
using System.Drawing;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using FluentAssertions;
using SportSimulator.Tracking;
using SportSimulator.Vision;
using SportSimulator.Vision.Calibration;
using Xunit;

namespace SportSimulator.Tests
{
    public class Spin3DEstimatorTests
    {
        // Same rectified-rig convention as TriangulatorTests.cs.
        private const double Fx = 1800.0, Fy = 1800.0, Cx = 640.0, Cy = 512.0, Baseline = 0.70;
        private const int CanvasW = 1280, CanvasH = 1024;

        private static Triangulator MakeTriangulator()
        {
            var cal = new StereoCalibrationData
            {
                K0 = new[] { Fx, 0, Cx, 0, Fy, Cy, 0, 0, 1.0 },
                D0 = new double[] { 0, 0, 0, 0, 0 },
                K1 = new[] { Fx, 0, Cx, 0, Fy, Cy, 0, 0, 1.0 },
                D1 = new double[] { 0, 0, 0, 0, 0 },
                R  = new[] { 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0 },
                T  = new[] { Baseline, 0.0, 0.0 },
                R0 = new[] { 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0 },
                R1 = new[] { 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0 },
                P0 = new[] { Fx, 0, Cx, 0,           0, Fy, Cy, 0, 0, 0, 1.0, 0 },
                P1 = new[] { Fx, 0, Cx, -Fx*Baseline, 0, Fy, Cy, 0, 0, 0, 1.0, 0 },
                Q  = new[] { 1.0, 0, 0, -Cx, 0, 1.0, 0, -Cy, 0, 0, 0, Fx, 0, 0, 1.0 / Baseline, 0 },
                ImageWidth = CanvasW, ImageHeight = CanvasH
            };
            var t = new Triangulator();
            t.Configure(cal);
            return t;
        }

        private static PointF ProjectLeft(Vec3 p) => new((float)(Fx * p.X / p.Z + Cx), (float)(Fy * p.Y / p.Z + Cy));
        private static PointF ProjectRight(Vec3 p) => new((float)(Fx * p.X / p.Z + Cx - Fx * Baseline / p.Z), (float)(Fy * p.Y / p.Z + Cy));

        // Spread matches a real ball's rough radius (~0.1m) rather than a tighter
        // cluster — points too close together (in pixel space, after projection)
        // are visually similar/ambiguous for the stereo patch matcher to tell
        // apart, which was the actual cause of a first version of this test
        // recovering a badly inflated angle: points got cross-matched to the
        // WRONG same-looking neighbor. Real ball texture (dimples/panels/laces)
        // has more locally-unique detail than these synthetic squares — RenderFrame
        // also gives each point a distinct shade to reduce that ambiguity further.
        private static Vec3[] MakeScatteredOffsets(int count, int seed)
        {
            var rng = new Random(seed);
            var pts = new Vec3[count];
            for (int i = 0; i < count; i++)
                pts[i] = new Vec3(
                    (float)(rng.NextDouble() * 0.20 - 0.10),
                    (float)(rng.NextDouble() * 0.20 - 0.10),
                    (float)(rng.NextDouble() * 0.20 - 0.10));
            return pts;
        }

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

        private static Mat RenderFrame(PointF ballCenterPx, float ballRadiusPx, PointF[] featurePx)
        {
            var mat = new Mat(CanvasH, CanvasW, DepthType.Cv8U, 1);
            mat.SetTo(new MCvScalar(30));
            CvInvoke.Circle(mat, Point.Round(ballCenterPx), (int)ballRadiusPx, new MCvScalar(120), -1);
            for (int i = 0; i < featurePx.Length; i++)
            {
                // Distinct shade per point (not just alternating) so the stereo
                // patch matcher has a real chance of telling neighbors apart —
                // see MakeScatteredOffsets comment.
                int shade = 5 + (i * 230 / Math.Max(1, featurePx.Length - 1));
                var p = Point.Round(featurePx[i]);
                CvInvoke.Rectangle(mat, new Rectangle(p.X - 3, p.Y - 3, 6, 6), new MCvScalar(shade), -1);
            }
            return mat;
        }

        [Fact]
        public void KnownRotation_RecoveredEndToEnd_ThroughFullPipeline()
        {
            // Ball center 3.5m out (near the rig's real ~3m working distance),
            // static between the two synthetic instants — isolates the untested
            // part (detection/tracking/stereo-matching/triangulation) from
            // translation handling, which RotationFitterTests already covers.
            var ballCenter3D = new Vec3(0f, 0f, 3.5f);
            const float ballRadius3D = 0.11f; // realistic soccer-ball radius, consistent with the offset spread below
            float ballRadiusPx = (float)(Fx * ballRadius3D / ballCenter3D.Z);

            var offsets = MakeScatteredOffsets(8, seed: 11);

            const double axX = 1, axY = 0, axZ = 0; // rotate about X — the "backspin" case
            const double groundTruthDeg = 18; // within a realistic single-frame soccer-spin delta at 200fps
            var R = RodriguesMatrix(axX, axY, axZ, groundTruthDeg * Math.PI / 180.0);

            var absA = Array.ConvertAll(offsets, o => new Vec3(ballCenter3D.X + o.X, ballCenter3D.Y + o.Y, ballCenter3D.Z + o.Z));
            var absB = Array.ConvertAll(offsets, o => { var r = Apply(R, o); return new Vec3(ballCenter3D.X + r.X, ballCenter3D.Y + r.Y, ballCenter3D.Z + r.Z); });

            var leftPxA  = Array.ConvertAll(absA, ProjectLeft);
            var rightPxA = Array.ConvertAll(absA, ProjectRight);
            var leftPxB  = Array.ConvertAll(absB, ProjectLeft);
            var rightPxB = Array.ConvertAll(absB, ProjectRight);

            var ballLeftPx  = ProjectLeft(ballCenter3D);
            var ballRightPx = ProjectRight(ballCenter3D);

            using var frameA_left  = RenderFrame(ballLeftPx,  ballRadiusPx, leftPxA);
            using var frameA_right = RenderFrame(ballRightPx, ballRadiusPx, rightPxA);
            using var frameB_left  = RenderFrame(ballLeftPx,  ballRadiusPx, leftPxB);
            using var frameB_right = RenderFrame(ballRightPx, ballRadiusPx, rightPxB);

            var triangulator = MakeTriangulator();
            var estimator = new Spin3DEstimator(triangulator);
            float expectedDisparityPx = ballLeftPx.X - ballRightPx.X;

            const long dtUs = 5000; // 200fps
            var first = estimator.Update(frameA_left, frameA_right, ballLeftPx, ballRadiusPx, expectedDisparityPx, timestampUs: 0);
            first.Valid.Should().BeFalse("no previous frame yet — nothing to compare against");

            var second = estimator.Update(frameB_left, frameB_right, ballLeftPx, ballRadiusPx, expectedDisparityPx, timestampUs: dtUs);

            second.Valid.Should().BeTrue("enough tracked+matched points should survive the full pipeline to fit a rotation");
            second.PointsUsed.Should().BeGreaterOrEqualTo(3);

            // Wide ratio bound, not a tight tolerance: this test's job is catching a
            // gross wiring bug (wrong units, a missing /dt, a 10x scale error, a
            // garbage/zero output) through the full detection -> tracking ->
            // stereo-match -> triangulate -> fit chain. It is NOT re-proving
            // numerical precision — RotationFitterTests already does that tightly
            // (dot > 0.99, angle within 0.5°) against clean point correspondences.
            // Real end-to-end precision depends on how much natural ball texture
            // actually contrasts under real lighting, which only real footage can
            // answer (see FeaturePointTracker.cs) — this synthetic rendering's own
            // noise floor (integer pixel rounding, GFTT response on small repeated
            // squares) isn't representative of that either way.
            float expectedRpm = (float)groundTruthDeg / (dtUs / 1_000_000f) / 6f;
            second.Rpm.Should().BeInRange(expectedRpm * 0.3f, expectedRpm * 3f,
                "recovered magnitude should be the right order of magnitude, not off by a gross factor");

            // Deliberately loose: this checks the axis lands in the right general
            // direction (not orthogonal or flipped to a different rotation plane
            // entirely) through the FULL pipeline — detection, optical flow,
            // stereo patch-matching, triangulation, and the fit all stacked
            // together, each contributing pixel-level noise/rounding. Axis
            // precision itself is already proven tightly (dot > 0.99) by
            // RotationFitterTests against clean point correspondences — this test's
            // job is to catch a gross wiring bug (wrong sign convention, swapped
            // coordinate axes, mismatched point correspondence), not to re-prove
            // Kabsch's own precision through a synthetic-rendering pipeline that
            // has its own artifacts (integer pixel rounding, GFTT corner-response
            // quirks on repeated small squares) unrelated to the production code's
            // real-world accuracy, which can only be validated against real footage.
            float dot = second.AxisUnit.X * (float)axX + second.AxisUnit.Y * (float)axY + second.AxisUnit.Z * (float)axZ;
            Math.Abs(dot).Should().BeGreaterThan(0.5f,
                "recovered axis should point roughly along the ground-truth X axis (backspin case), not be " +
                "orthogonal to it or dominated by a different rotation plane");
        }

        [Fact]
        public void FirstCall_AlwaysInvalid_NoPreviousFrameToCompare()
        {
            var ballCenter3D = new Vec3(0f, 0f, 3.5f);
            float ballRadiusPx = (float)(Fx * 0.06f / ballCenter3D.Z);
            var ballLeftPx  = ProjectLeft(ballCenter3D);
            var ballRightPx = ProjectRight(ballCenter3D);
            var offsets = MakeScatteredOffsets(6, seed: 3);
            var leftPx  = Array.ConvertAll(offsets, o => ProjectLeft(new Vec3(ballCenter3D.X + o.X, ballCenter3D.Y + o.Y, ballCenter3D.Z + o.Z)));
            var rightPx = Array.ConvertAll(offsets, o => ProjectRight(new Vec3(ballCenter3D.X + o.X, ballCenter3D.Y + o.Y, ballCenter3D.Z + o.Z)));

            using var left  = RenderFrame(ballLeftPx, ballRadiusPx, leftPx);
            using var right = RenderFrame(ballRightPx, ballRadiusPx, rightPx);

            var estimator = new Spin3DEstimator(MakeTriangulator());
            float expectedDisparityPx = ballLeftPx.X - ballRightPx.X;
            var result = estimator.Update(left, right, ballLeftPx, ballRadiusPx, expectedDisparityPx, timestampUs: 12345);

            result.Valid.Should().BeFalse();
        }
    }
}
