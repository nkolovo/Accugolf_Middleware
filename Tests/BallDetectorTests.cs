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
