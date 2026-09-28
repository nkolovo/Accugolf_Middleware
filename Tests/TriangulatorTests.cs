// ------------------------------------------------------------
// Tests/TriangulatorTests.cs
// ------------------------------------------------------------
using FluentAssertions;
using System.Drawing;
using SportSimulator.Vision;
using SportSimulator.Vision.Calibration;
using Xunit;

namespace SportSimulator.Tests
{
    public class TriangulatorTests
    {
        // Ideal rectified stereo rig: fx=fy=1800px, cx=640, cy=512, baseline=0.70m.
        // P0 and P1 are the standard rectified projection matrices:
        //   P0 = [fx  0  cx    0 ]     (left camera, no offset)
        //        [ 0 fy  cy    0 ]
        //        [ 0  0   1    0 ]
        //
        //   P1 = [fx  0  cx  -fx*B ]   (right camera, offset by -fx*baseline)
        //        [ 0 fy  cy    0   ]
        //        [ 0  0   1    0   ]
        //
        // For a ball at world (0, 0, 2.0m):
        //   pt0 = (640, 512)   disparity = fx*B/Z = 1800*0.70/2 = 630px
        //   pt1 = (10,  512)   → pt0.X - pt1.X = 630
        private static StereoCalibrationData MakeCalibration(double baseline = 0.70)
        {
            const double fx = 1800.0, fy = 1800.0, cx = 640.0, cy = 512.0;
            double B = baseline;
            return new StereoCalibrationData
            {
                K0 = new[] { fx, 0, cx, 0, fy, cy, 0, 0, 1.0 },
                D0 = new double[] { 0, 0, 0, 0, 0 },
                K1 = new[] { fx, 0, cx, 0, fy, cy, 0, 0, 1.0 },
                D1 = new double[] { 0, 0, 0, 0, 0 },
                R  = new[] { 1.0, 0, 0,  0, 1.0, 0,  0, 0, 1.0 },
                T  = new[] { B, 0.0, 0.0 },
                R0 = new[] { 1.0, 0, 0,  0, 1.0, 0,  0, 0, 1.0 },
                R1 = new[] { 1.0, 0, 0,  0, 1.0, 0,  0, 0, 1.0 },
                P0 = new[] { fx, 0, cx, 0,      0, fy, cy, 0,  0, 0, 1.0, 0 },
                P1 = new[] { fx, 0, cx, -fx*B,  0, fy, cy, 0,  0, 0, 1.0, 0 },
                Q  = new[] { 1.0, 0, 0, -cx,  0, 1.0, 0, -cy,  0, 0, 0, fx,  0, 0, 1.0/B, 0 },
                ImageWidth = 1280, ImageHeight = 1024
            };
        }

        private static Triangulator MakeTriangulator(double baseline = 0.70)
        {
            var t = new Triangulator();
            t.Configure(MakeCalibration(baseline));
            return t;
        }

        // ── World-frame correction (tilted rig) ─────────────────────────────────
        // Regression coverage for the "ball goes the wrong way" bug: this rig's
        // camera is mounted tilted BACKWARD at the tee (114in up, 33in forward of
        // it — see notes/14-...md), so "further along the camera's own view axis"
        // and "further downrange toward the net" are opposite directions. Before
        // Triangulator.ToWorldFrame existed, a real shot's reported Z DECREASED as
        // the ball flew toward the net — sent straight to Unity as a negative
        // Z-velocity, i.e. the ball moving backward. These tests project a KNOWN
        // world point through the same math MockCameraManager uses to generate
        // synthetic frames, then verify TriangulateStereo recovers that same world
        // point — a controlled round-trip, not an inference from noisy live output
        // (which varies run-to-run with RNG draws and isn't a valid basis for
        // verifying a sign convention).
        private const double RigHeightM = StereoCalibrationData.DefaultCameraHeightM;
        private const double RigForwardOffsetM = StereoCalibrationData.DefaultCameraForwardOffsetM;

        private static StereoCalibrationData MakeTiltedCalibration(double baseline = 0.4953)
        {
            var cal = MakeCalibration(baseline);
            cal.CameraHeightM = RigHeightM;
            cal.CameraForwardOffsetM = RigForwardOffsetM;
            return cal;
        }

        // Mirrors MockCameraManager.ProjectToCamera's math exactly (world point,
        // tilted/elevated camera pair -> pixel coordinates for both cameras) so
        // these tests exercise a genuine round trip through Triangulator's inverse.
        private static (PointF pt0, PointF pt1) ProjectThroughTiltedRig(
            double worldX, double worldY, double worldZ, double baseline,
            double fx, double fy, double cx, double cy)
        {
            double tiltRad = System.Math.Atan(RigForwardOffsetM / RigHeightM);
            double sinT = System.Math.Sin(tiltRad), cosT = System.Math.Cos(tiltRad);

            PointF ProjectForCamera(double camX)
            {
                double relX = worldX - camX;
                double relY = worldY - RigHeightM;
                double relZ = worldZ - RigForwardOffsetM;
                double yLocal = relY * sinT - relZ * cosT;
                double zLocal = -relY * cosT - relZ * sinT;
                double px = fx * relX / zLocal + cx;
                double py = fy * (-yLocal) / zLocal + cy;
                return new PointF((float)px, (float)py);
            }

            return (ProjectForCamera(-baseline / 2.0), ProjectForCamera(baseline / 2.0));
        }

        [Fact]
        public void TriangulateStereo_TiltedRig_TeeRoundTripsToOrigin()
        {
            var t = new Triangulator();
            t.Configure(MakeTiltedCalibration());
            var (pt0, pt1) = ProjectThroughTiltedRig(0, 0, 0, 0.4953, 1800, 1800, 640, 512);

            var result = t.TriangulateStereo(pt0, pt1);

            result.X.Should().BeApproximately(0f, 0.01f);
            result.Y.Should().BeApproximately(0f, 0.01f);
            result.Z.Should().BeApproximately(0f, 0.01f);
        }

        [Fact]
        public void TriangulateStereo_TiltedRig_DownrangeShot_ZIncreasesTowardNet()
        {
            // The bug this guards against: with the tilt correction missing, this
            // exact point's raw camera-local Z comes back SMALLER than the tee's
            // (D=~3.01 -> ~2.2), i.e. it looks like the ball moved backward. The
            // world-frame-corrected Z must increase from 0 toward the true 1.2m the
            // ball actually traveled downrange, not decrease.
            var t = new Triangulator();
            t.Configure(MakeTiltedCalibration());
            var (pt0, pt1) = ProjectThroughTiltedRig(0, 0.3, 1.2, 0.4953, 1800, 1800, 640, 512);

            var result = t.TriangulateStereo(pt0, pt1);

            result.Z.Should().BeApproximately(1.2f, 0.02f,
                "world Z must increase toward the net as the ball flies forward, not decrease");
            result.Y.Should().BeApproximately(0.3f, 0.02f);
            result.X.Should().BeApproximately(0f, 0.02f);
        }

        [Fact]
        public void TriangulateStereo_TiltedRig_LateralOffset_RoundTripsCorrectly()
        {
            var t = new Triangulator();
            t.Configure(MakeTiltedCalibration());
            var (pt0, pt1) = ProjectThroughTiltedRig(0.5, 0.2, 0.8, 0.4953, 1800, 1800, 640, 512);

            var result = t.TriangulateStereo(pt0, pt1);

            result.X.Should().BeApproximately(0.5f, 0.02f);
            result.Y.Should().BeApproximately(0.2f, 0.02f);
            result.Z.Should().BeApproximately(0.8f, 0.02f);
        }

        // ── TriangulateStereo ────────────────────────────────────────────────────

        [Fact]
        public void TriangulateStereo_PerfectDisparity_ReturnsTier1AndCorrectDepth()
        {
            // Ball at (0, 0, 2.0m). pt0 at image centre; pt1 offset by 630px disparity.
            // Both the disparity formula and the DLT should agree exactly → Tier 1.
            var t = MakeTriangulator();
            var result = t.TriangulateStereo(new PointF(640f, 512f), new PointF(10f, 512f));

            result.Tier.Should().Be(TriangulationTier.FullStereo);
            result.Z.Should().BeApproximately(2.0f, 0.05f);
            result.Confidence.Should().BeGreaterThan(0.79f,
                "Tier 1 confidence band is 0.80–1.00");
        }

        [Fact]
        public void TriangulateStereo_PerfectDisparity_XIsBaselineCentered_YIsZero()
        {
            // Ball is centred on camera0's own image (pixel = principal point) — but
            // Triangulator now re-centers X onto the stereo pair's shared baseline
            // midpoint (see Triangulator.ToWorldFrame), not camera0's own optical
            // axis. A point straight down camera0's axis sits baseline/2 to camera0's
            // own side of that midpoint, so world X = -baseline/2, not 0. This
            // fixture has no tilt configured (CameraHeightM defaults to 0), so Y
            // passes through unchanged and is still 0.
            var t = MakeTriangulator(baseline: 0.70);
            var result = t.TriangulateStereo(new PointF(640f, 512f), new PointF(10f, 512f));

            result.X.Should().BeApproximately(-0.35f, 0.05f);
            result.Y.Should().BeApproximately(0f, 0.05f);
        }

        [Fact]
        public void TriangulateStereo_NegativeDisparity_FallsToMonocular()
        {
            // pt0.X < pt1.X → disparity ≤ 0 — ball "behind" baseline, physically impossible
            var t = MakeTriangulator();
            var result = t.TriangulateStereo(new PointF(100f, 512f), new PointF(200f, 512f));

            result.Tier.Should().Be(TriangulationTier.Monocular);
            result.Confidence.Should().BeLessThan(0.5f);
        }

        [Fact]
        public void TriangulateStereo_ZeroDisparity_FallsToMonocular()
        {
            // Exactly equal X coordinates → disparity = 0 → undefined depth
            var t = MakeTriangulator();
            var result = t.TriangulateStereo(new PointF(400f, 400f), new PointF(400f, 400f));

            result.Tier.Should().Be(TriangulationTier.Monocular);
        }

        [Fact]
        public void TriangulateStereo_TinyDisparity_ZOutOfRange_FallsToMonocular()
        {
            // Disparity = 0.2px → Z = 1800*0.70/0.2 = 6300m, far beyond the 10m max.
            var t = MakeTriangulator();
            var result = t.TriangulateStereo(new PointF(640.2f, 512f), new PointF(640.0f, 512f));

            result.Tier.Should().Be(TriangulationTier.Monocular);
        }

        [Fact]
        public void TriangulateStereo_DisparityProportionalToBaseline()
        {
            // Doubling the baseline should double the recovered Z for the same pixel disparity.
            // t70: disparity=630px → Z = 1800*0.70/630 = 2.0m
            // t35: same 630px disparity → Z = 1800*0.35/630 = 1.0m
            var t70 = MakeTriangulator(baseline: 0.70);
            var t35 = MakeTriangulator(baseline: 0.35);

            var r70 = t70.TriangulateStereo(new PointF(640f, 512f), new PointF(10f, 512f));
            var r35 = t35.TriangulateStereo(new PointF(640f, 512f), new PointF(10f, 512f));

            // Both should produce a real stereo result (not Monocular fallback)
            r70.Tier.Should().NotBe(TriangulationTier.KalmanOnly);
            r35.Tier.Should().NotBe(TriangulationTier.KalmanOnly);

            r70.Z.Should().BeApproximately(2.0f, 0.1f);
            r35.Z.Should().BeApproximately(1.0f, 0.1f);
            r70.Z.Should().BeApproximately(r35.Z * 2f, 0.2f,
                "doubling baseline doubles recovered Z for the same pixel disparity");
        }

        // ── TriangulateMonocular ─────────────────────────────────────────────────

        [Fact]
        public void TriangulateMonocular_PrincipalPoint_XIsBaselineCentered_YIsZero()
        {
            // A pixel at the principal point back-projects to (0, 0, Z) relative to
            // camera0's own axis — but the result is re-centered onto the baseline
            // midpoint (same reasoning as TriangulateStereo's equivalent test above),
            // so world X = -baseline/2, not 0. No tilt configured in this fixture, so
            // Z passes through unchanged as the known depth, and Y is still 0.
            var t = MakeTriangulator(baseline: 0.70);
            var result = t.TriangulateMonocular(new PointF(640f, 512f), knownWorldY: 0f, knownWorldZ: 3.0f);

            result.Tier.Should().Be(TriangulationTier.Monocular);
            result.Z.Should().BeApproximately(3.0f, 0.001f);
            result.X.Should().BeApproximately(-0.35f, 0.001f);
            result.Y.Should().BeApproximately(0f, 0.001f);
        }

        [Fact]
        public void TriangulateMonocular_OffCentrePixel_CorrectWorldX()
        {
            // Pixel 180px right of principal point at Z=3.0m, relative to camera0's
            // own axis: X_local = (820 - 640) * 3.0 / 1800 = 180 * 3 / 1800 = 0.30m.
            // Re-centered onto the baseline midpoint: world X = 0.30 - 0.35 = -0.05m.
            var t = MakeTriangulator(baseline: 0.70);
            var result = t.TriangulateMonocular(new PointF(820f, 512f), knownWorldY: 0f, knownWorldZ: 3.0f);

            result.X.Should().BeApproximately(-0.05f, 0.01f);
            result.Y.Should().BeApproximately(0f, 0.001f);
            result.Z.Should().BeApproximately(3.0f, 0.001f);
        }

        [Fact]
        public void TriangulateMonocular_ConfidenceIsLow()
        {
            var t = MakeTriangulator();
            var result = t.TriangulateMonocular(new PointF(640f, 512f), knownWorldY: 0f, knownWorldZ: 2.0f);

            result.Confidence.Should().BeLessThan(0.5f,
                "monocular result has lower confidence than stereo");
        }

        // ── DepthPrecisionMm ─────────────────────────────────────────────────────

        [Fact]
        public void DepthPrecisionMm_GrowsWithZSquared()
        {
            // Depth precision ∝ Z², so doubling Z quadruples the error.
            var t = MakeTriangulator();
            float p1 = t.DepthPrecisionMm(1.0f);
            float p2 = t.DepthPrecisionMm(2.0f);

            p2.Should().BeApproximately(p1 * 4f, p1 * 0.1f,
                "precision degrades with Z²");
        }

        [Fact]
        public void DepthPrecisionMm_AtTwoMetres_IsReasonableForGolf()
        {
            // With fx=1800px and baseline=0.70m:
            //   precision = Z² * 1000 / (fx * B) = 4*1000 / 1260 ≈ 3.17mm at 2m
            var t = MakeTriangulator();
            float prec = t.DepthPrecisionMm(2.0f);

            prec.Should().BeApproximately(3.17f, 0.2f);
            prec.Should().BePositive();
        }

        [Fact]
        public void DepthPrecisionMm_WiderBaseline_BetterPrecision()
        {
            float p70 = MakeTriangulator(0.70).DepthPrecisionMm(2.0f);
            float p90 = MakeTriangulator(0.90).DepthPrecisionMm(2.0f);

            p90.Should().BeLessThan(p70,
                "wider baseline means smaller depth error");
        }
    }
}
