// ------------------------------------------------------------
// Tests/MockCameraManagerTests.cs
// ------------------------------------------------------------
using FluentAssertions;
using SportSimulator.Vision;
using System.Threading;
using Xunit;

namespace SportSimulator.Tests
{
    public class MockCameraManagerTests
    {
        // ── Frame production ─────────────────────────────────────────────────────

        [Fact]
        public void StartCapture_ProducesFrames()
        {
            using var mock = new MockCameraManager();
            mock.Initialize("golf");
            mock.StartCapture();

            // Wait up to 2 seconds for at least one frame
            bool got = mock.FrameQueue.TryTake(out _, millisecondsTimeout: 2000);

            got.Should().BeTrue("mock should produce frames after StartCapture");
        }

        [Fact]
        public void StartCapture_BothCameraIndicesAppear()
        {
            using var mock = new MockCameraManager();
            mock.Initialize("golf");
            mock.StartCapture();

            bool sawCam0 = false, sawCam1 = false;

            // Drain up to 30 frames or until both cameras seen
            for (int i = 0; i < 30 && !(sawCam0 && sawCam1); i++)
            {
                if (!mock.FrameQueue.TryTake(out var frame, millisecondsTimeout: 500)) break;
                if (frame.CameraIndex == 0) sawCam0 = true;
                if (frame.CameraIndex == 1) sawCam1 = true;
            }

            sawCam0.Should().BeTrue("camera 0 should produce frames");
            sawCam1.Should().BeTrue("camera 1 should produce frames");
        }

        [Fact]
        public void Frames_HaveCorrectDimensions()
        {
            using var mock = new MockCameraManager();
            mock.Initialize("golf");
            mock.StartCapture();

            mock.FrameQueue.TryTake(out var frame, millisecondsTimeout: 2000);

            frame.Should().NotBeNull();
            frame!.Width.Should().Be(1280);
            frame.Height.Should().Be(1024);
            frame.Data.Length.Should().Be(1280 * 1024, "data must be one byte per pixel (Mono8)");
        }

        [Fact]
        public void Frames_HaveNonEmptyPixelData()
        {
            // MockCameraManager draws a white circle on each frame;
            // at least some pixels must be non-zero.
            using var mock = new MockCameraManager();
            mock.Initialize("golf");
            mock.StartCapture();

            CameraFrame? ballFrame = null;
            for (int i = 0; i < 20; i++)
            {
                if (!mock.FrameQueue.TryTake(out var f, millisecondsTimeout: 500)) break;
                bool hasBright = false;
                foreach (var b in f.Data) { if (b > 0) { hasBright = true; break; } }
                if (hasBright) { ballFrame = f; break; }
            }

            ballFrame.Should().NotBeNull("at least one frame should contain a visible ball blob");
        }

        [Fact]
        public void Frames_TimestampIsPositive()
        {
            using var mock = new MockCameraManager();
            mock.Initialize("golf");
            mock.StartCapture();

            mock.FrameQueue.TryTake(out var frame, millisecondsTimeout: 2000);

            frame.Should().NotBeNull();
            frame!.TimestampUs.Should().BeGreaterThan(0);
        }

        [Fact]
        public void Frames_TimestampsIncrease_PerCamera()
        {
            // Both cameras emit frames with the same timestamp for each loop iteration,
            // so the interleaved queue is not globally monotonic. Check per-camera instead.
            using var mock = new MockCameraManager();
            mock.Initialize("golf");
            mock.StartCapture();

            long prevCam0 = 0, prevCam1 = 0;
            int outOfOrder = 0;

            for (int i = 0; i < 20; i++)
            {
                if (!mock.FrameQueue.TryTake(out var frame, millisecondsTimeout: 500)) break;
                if (frame.CameraIndex == 0)
                {
                    if (frame.TimestampUs < prevCam0) outOfOrder++;
                    prevCam0 = frame.TimestampUs;
                }
                else
                {
                    if (frame.TimestampUs < prevCam1) outOfOrder++;
                    prevCam1 = frame.TimestampUs;
                }
            }

            outOfOrder.Should().Be(0, "per-camera timestamps must be non-decreasing");
        }

        // ── Stop behaviour ───────────────────────────────────────────────────────

        [Fact]
        public void Stop_HaltsFrameProduction()
        {
            using var mock = new MockCameraManager();
            mock.Initialize("golf");
            mock.StartCapture();

            // Let some frames arrive first
            mock.FrameQueue.TryTake(out _, millisecondsTimeout: 1000);

            mock.Stop();

            // Drain whatever is already in the buffer
            while (mock.FrameQueue.TryTake(out _, millisecondsTimeout: 0)) { }

            // After Stop, no new frames should arrive within 500ms
            bool gotFrame = mock.FrameQueue.TryTake(out _, millisecondsTimeout: 500);

            gotFrame.Should().BeFalse("Stop should halt frame production");
        }

        // ── Trajectory configuration ─────────────────────────────────────────────

        [Fact]
        public void SetTrajectory_VerySlowSpeed_FramesStillProduced()
        {
            // Note: speedMps=0 with the default start position (bz=0.5m) places both
            // cameras off-sensor due to the 350mm baseline offset — no frames would
            // be produced. Use a slow but non-zero speed so the ball moves into view.
            using var mock = new MockCameraManager();
            mock.SetTrajectory(speedMps: 5f, launchAngleDeg: 10f);
            mock.Initialize("generic");
            mock.StartCapture();

            bool got = mock.FrameQueue.TryTake(out _, millisecondsTimeout: 2000);
            got.Should().BeTrue("frames should arrive for a slow ball trajectory");
        }

        [Fact]
        public void ApplyProfile_DoesNotThrow()
        {
            using var mock = new MockCameraManager();
            mock.Initialize("golf");
            var act = () => mock.ApplyProfile("soccer");
            act.Should().NotThrow();
        }
    }
}
