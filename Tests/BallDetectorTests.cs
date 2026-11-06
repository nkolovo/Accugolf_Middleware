// ------------------------------------------------------------
// Tests/BallDetectorTests.cs
// ------------------------------------------------------------
// All tests use a small 320×240 synthetic frame to keep allocations tiny.
// A "ball frame" is a black background with a white filled circle at (cx, cy).
// ------------------------------------------------------------
using FluentAssertions;
using SportSimulator.Models;
using SportSimulator.Vision;
using Xunit;

namespace SportSimulator.Tests
{
    public class BallDetectorTests
    {
        private const int W = 320, H = 240;

        private static SportProfile GolfProfile() => new()
        {
            SportId        = "golf",
            IsSphere       = true,
            MinContourArea = 20,
            MaxContourArea = 3000
        };

        private static CameraFrame BlackFrame(int camIdx = 0, long ts = 1000) => new()
        {
            CameraIndex = camIdx,
            Data        = new byte[W * H], // all zeros = black
            Width       = W,
            Height      = H,
            TimestampUs = ts
        };

        private static CameraFrame BallFrame(int cx, int cy, int radius = 8,
                                              int camIdx = 0, long ts = 2000)
        {
            var data = new byte[W * H];
            for (int dy = -radius; dy <= radius; dy++)
            for (int dx = -radius; dx <= radius; dx++)
            {
                if (dx * dx + dy * dy > radius * radius) continue;
                int row = cy + dy, col = cx + dx;
                if (row < 0 || row >= H || col < 0 || col >= W) continue;
                data[row * W + col] = 255;
            }
            return new CameraFrame
            {
                CameraIndex = camIdx, Data = data,
                Width = W, Height = H, TimestampUs = ts
            };
        }

        // ── Background initialisation ────────────────────────────────────────────

        [Fact]
        public void FirstFrame_InitializesBackground_ReturnsNotFound()
        {
            var detector = new BallDetector();
            detector.SetProfile(GolfProfile());

            var result = detector.Detect(BlackFrame());

            result.Found.Should().BeFalse("first frame seeds the background model, no diff yet");
        }

        [Fact]
        public void IdenticalToBackground_ReturnsNotFound()
        {
            var detector = new BallDetector();
            detector.SetProfile(GolfProfile());

            detector.Detect(BlackFrame());                   // seed background
            var result = detector.Detect(BlackFrame(ts: 2000)); // same image

            result.Found.Should().BeFalse("uniform black frame matches background exactly");
        }

        // ── Background model ─────────────────────────────────────────────────────

        private static CameraFrame Lit(CameraFrame frame, byte level)
        {
            for (int i = 0; i < frame.Data.Length; i++)
                if (frame.Data[i] == 0) frame.Data[i] = level;
            return frame;
        }

        // While only part of the ball has crossed the frame edge its blob is below
        // MinContourArea; that part must not be absorbed into the background.
        [Fact]
        public void BallEnteringThroughFrameEdge_IsFoundAtRest_AndLeavesNoGhost()
        {
            var detector = new BallDetector();
            detector.SetProfile(new SportProfile { IsSphere = true, MinContourArea = 400, MaxContourArea = 3000 });
            detector.Detect(BlackFrame());

            for (int cx = -20; cx <= 60; cx++)
                detector.Detect(BallFrame(cx, 120, radius: 20));

            for (int i = 0; i < 50; i++)
            {
                var resting = detector.Detect(BallFrame(60, 120, radius: 20));
                resting.Found.Should().BeTrue("a ball that rolled in stays visible however long it rests");
                resting.Center.X.Should().BeApproximately(60f, 1f);
            }

            detector.Detect(BlackFrame()).Found.Should().BeFalse("nothing should remain once the ball has left");
        }

        [Fact]
        public void GradualLightingDrift_IsTrackedAroundARestingBall()
        {
            var detector = new BallDetector();
            detector.SetProfile(GolfProfile());
            detector.Detect(BlackFrame());

            // One level every 100 frames: about 2.6 levels a second at the real frame rate.
            for (int i = 0; i < 3000; i++)
            {
                var result = detector.Detect(Lit(BallFrame(160, 120), (byte)(1 + i / 100)));
                result.Found.Should().BeTrue();
                result.Center.X.Should().BeApproximately(160f, 1f);
                result.RadiusPx.Should().BeLessThan(12f, "drift must not merge into the ball's blob");
            }
        }

        // Pixels of a person close to the floor's brightness must not get learned: that
        // left ghost blobs stuck to the ball after the person moved away.
        [Fact]
        public void SomethingNearBackgroundBrightness_PassingThrough_LeavesNoGhost()
        {
            var detector = new BallDetector();
            detector.SetProfile(GolfProfile());
            detector.Detect(Lit(BlackFrame(), 80));

            // A big dim shape, within the threshold of the floor, lingering for half a second.
            for (int i = 0; i < 130; i++)
                detector.Detect(Lit(BlackFrame(), 100));

            detector.Detect(Lit(BlackFrame(), 80)).Found.Should().BeFalse();
            var ball = detector.Detect(Lit(BallFrame(160, 120), 80));
            ball.Found.Should().BeTrue();
            ball.RadiusPx.Should().BeLessThan(12f, "the ball's blob must be just the ball");
        }

        // Darker than the floor isn't the ball: its shadow pulled its center a different
        // way in each camera, and a dark foot stayed joined to it through the kick.
        [Fact]
        public void BrighterThanBackground_IgnoresADarkShapeTouchingTheBall()
        {
            var detector = new BallDetector();
            detector.SetProfile(new SportProfile { IsSphere = true, BrighterThanBackground = true, MinContourArea = 400, MaxContourArea = 3000 });
            detector.Detect(Lit(BlackFrame(), 80));

            var frame = Lit(BallFrame(160, 120, radius: 20), 80);
            for (int row = 120; row < H; row++)         // a dark leg from the ball off the frame's edge
                for (int col = 180; col < 220; col++)
                    frame.Data[row * W + col] = 10;

            var result = detector.Detect(frame);

            result.Found.Should().BeTrue();
            result.Center.X.Should().BeApproximately(160f, 1.5f);
            result.Center.Y.Should().BeApproximately(120f, 1.5f);
            result.RadiusPx.Should().BeApproximately(20f, 2f);
        }

        [Fact]
        public void LightingStepAcrossTheFrame_IsAbsorbedAfterAFewSeconds()
        {
            var detector = new BallDetector();
            detector.SetProfile(GolfProfile());
            detector.Detect(BlackFrame());

            for (int i = 0; i < 1300; i++)
                detector.Detect(Lit(BlackFrame(), 100)).Found.Should().BeFalse("a frame-filling blob isn't a ball");

            var ball = detector.Detect(Lit(BallFrame(160, 120), 100));
            ball.Found.Should().BeTrue("detection recovers once the new lighting is the background");
            ball.Center.X.Should().BeApproximately(160f, 1f);
        }

        // ── Ball detection ───────────────────────────────────────────────────────

        [Fact]
        public void BallInSecondFrame_IsDetected()
        {
            var detector = new BallDetector();
            detector.SetProfile(GolfProfile());

            detector.Detect(BlackFrame());
            var result = detector.Detect(BallFrame(cx: 160, cy: 120, radius: 8));

            result.Found.Should().BeTrue();
            result.Center.X.Should().BeApproximately(160f, 10f);
            result.Center.Y.Should().BeApproximately(120f, 10f);
        }

        [Fact]
        public void DetectedBall_ConfidenceIsPositive()
        {
            var detector = new BallDetector();
            detector.SetProfile(GolfProfile());

            detector.Detect(BlackFrame());
            var result = detector.Detect(BallFrame(cx: 160, cy: 120, radius: 8));

            result.Confidence.Should().BeGreaterThan(0f);
            result.Confidence.Should().BeLessOrEqualTo(1f);
        }

        // ── Profile filtering ────────────────────────────────────────────────────

        [Fact]
        public void TooSmallContour_IsFiltered()
        {
            var detector = new BallDetector();
            detector.SetProfile(new SportProfile
            {
                IsSphere       = true,
                MinContourArea = 500, // large threshold — radius=2 gives area ≈ 12px²
                MaxContourArea = 3000
            });

            detector.Detect(BlackFrame());
            var result = detector.Detect(BallFrame(cx: 160, cy: 120, radius: 2));

            result.Found.Should().BeFalse("contour area below MinContourArea should be rejected");
        }

        [Fact]
        public void TooLargeContour_IsFiltered()
        {
            var detector = new BallDetector();
            detector.SetProfile(new SportProfile
            {
                IsSphere       = true,
                MinContourArea = 10,
                MaxContourArea = 50 // tight max — radius=20 gives area ≈ 1256px²
            });

            detector.Detect(BlackFrame());
            var result = detector.Detect(BallFrame(cx: 160, cy: 120, radius: 20));

            result.Found.Should().BeFalse("contour area above MaxContourArea should be rejected");
        }

        // ── DetectionResult metadata ─────────────────────────────────────────────

        [Fact]
        public void DetectionResult_CarriesCameraIndex()
        {
            var detector = new BallDetector();
            detector.SetProfile(GolfProfile());

            var bg = BlackFrame(camIdx: 1, ts: 100);
            detector.Detect(bg);

            var frame = BallFrame(cx: 160, cy: 120, camIdx: 1, ts: 99999);
            var result = detector.Detect(frame);

            result.CameraIndex.Should().Be(1);
        }

        [Fact]
        public void DetectionResult_CarriesTimestamp()
        {
            var detector = new BallDetector();
            detector.SetProfile(GolfProfile());

            detector.Detect(BlackFrame(ts: 1000));
            var result = detector.Detect(BallFrame(cx: 160, cy: 120, ts: 55555));

            result.TimestampUs.Should().Be(55555);
        }

        [Fact]
        public void NotFound_Result_StillCarriesMetadata()
        {
            // Even when no ball is found, CameraIndex and TimestampUs should be propagated
            var detector = new BallDetector();
            detector.SetProfile(GolfProfile());

            detector.Detect(BlackFrame(camIdx: 0, ts: 1000)); // background

            var emptyFrame = BlackFrame(camIdx: 0, ts: 7777);
            var result = detector.Detect(emptyFrame);

            result.Found.Should().BeFalse();
            result.TimestampUs.Should().Be(7777);
            result.CameraIndex.Should().Be(0);
        }
    }
}
