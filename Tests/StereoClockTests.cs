// ------------------------------------------------------------
// Tests/StereoClockTests.cs
// ------------------------------------------------------------
// The real cameras' timestamps come from two unrelated device clocks, so a
// cycle's time must stay continuous whichever camera detected the ball.
// ------------------------------------------------------------
using FluentAssertions;
using SportSimulator.Vision;
using Xunit;

namespace SportSimulator.Tests
{
    public class StereoClockTests
    {
        private const long Cam0 = 1_000_000_000, Cam1 = 3_456_789_012, Frame = 3817;

        [Fact]
        public void BothCameras_UsesCamera0Time()
        {
            new StereoClock().Next(Cam0, Cam1).Should().Be(Cam0);
        }

        [Fact]
        public void OnlyCamera1_IsConvertedToCamera0Time()
        {
            var clock = new StereoClock();
            clock.Next(Cam0, Cam1);

            clock.Next(null, Cam1 + Frame).Should().Be(Cam0 + Frame);
        }

        [Fact]
        public void NoDetection_AdvancesOneFrame()
        {
            var clock = new StereoClock();
            clock.Next(Cam0, Cam1);

            clock.Next(null, null).Should().Be(Cam0 + Frame);
            clock.Next(null, null).Should().Be(Cam0 + 2 * Frame);
        }

        [Fact]
        public void DetectionAfterCoasting_ReturnsToRealTime()
        {
            var clock = new StereoClock();
            clock.Next(Cam0, Cam1);
            clock.Next(null, null);

            clock.Next(Cam0 + 3 * Frame, null).Should().Be(Cam0 + 3 * Frame);
        }
    }
}
