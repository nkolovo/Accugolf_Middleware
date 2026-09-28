// ------------------------------------------------------------
// Tests/StereoRectifierTests.cs
// ------------------------------------------------------------
using FluentAssertions;
using SportSimulator.Vision.Calibration;
using Xunit;

namespace SportSimulator.Tests
{
    public class StereoRectifierTests
    {
        // Unrectified calibration inputs: intrinsics (K/D) plus stereo extrinsics
        // (R/T) — the same shape StereoCalibrator.Calibrate() produces from a real
        // checkerboard run. R0/R1/P0/P1/Q are Build()'s outputs, computed via
        // CvInvoke.StereoRectify from these inputs — not supplied here.
        private static StereoCalibrationData MakeCalibration()
        {
            const double fx = 1800.0, fy = 1800.0, cx = 640.0, cy = 512.0, B = 0.70;
            return new StereoCalibrationData
            {
                K0 = new[] { fx, 0, cx,  0, fy, cy,  0, 0, 1.0 },
                D0 = new double[] { 0, 0, 0, 0, 0 },
                K1 = new[] { fx, 0, cx,  0, fy, cy,  0, 0, 1.0 },
                D1 = new double[] { 0, 0, 0, 0, 0 },
                R  = new[] { 1.0, 0, 0,  0, 1.0, 0,  0, 0, 1.0 }, // no rotation between cameras
                T  = new[] { B, 0.0, 0.0 },                        // baseline along X (metres)
                ImageWidth  = 1280,
                ImageHeight = 1024
            };
        }

        [Fact]
        public void IsReady_FalseBeforeBuild()
        {
            var rectifier = new StereoRectifier();
            rectifier.IsReady.Should().BeFalse(
                "rectifier must be built before it can be used");
        }

        [Fact]
        public void Build_WithValidCalibration_SetsIsReady()
        {
            var rectifier = new StereoRectifier();
            rectifier.Build(MakeCalibration());
            rectifier.IsReady.Should().BeTrue();
        }

        [Fact]
        public void Build_CanBeCalledMultipleTimes()
        {
            // Re-calibrating (e.g. after the rig is adjusted) should succeed cleanly.
            var rectifier = new StereoRectifier();
            rectifier.Build(MakeCalibration());
            rectifier.Build(MakeCalibration()); // second build — should not throw
            rectifier.IsReady.Should().BeTrue();
        }
    }
}
