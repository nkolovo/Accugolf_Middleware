// ------------------------------------------------------------
// Tests/StereoCalibrationDataTests.cs
// ------------------------------------------------------------
using FluentAssertions;
using SportSimulator.Vision.Calibration;
using System.IO;
using System;
using Xunit;

namespace SportSimulator.Tests
{
    public class StereoCalibrationDataTests
    {
        // K0 layout (row-major 3×3):
        //   [fx, 0, cx,  0, fy, cy,  0, 0, 1]
        //    [0] [1] [2] [3] [4] [5] [6] [7] [8]

        [Fact]
        public void CreateDefaults_FocalLengthIsPositive()
        {
            var cal = StereoCalibrationData.CreateDefaults();
            cal.K0[0].Should().BeGreaterThan(0, "fx must be positive");
            cal.K0[4].Should().BeGreaterThan(0, "fy must be positive");
        }

        [Fact]
        public void CreateDefaults_BaselineStoredInT()
        {
            // T is negative here by OpenCV convention (X1 = R*X0 + T maps left-camera
            // points into the right camera's frame — a right camera physically further
            // +X means points appear at a SMALLER X in its own frame) — see the class
            // comment above CreateDefaults. Triangulator.Configure's standard
            // `baseline = -P1[0,3]/P1[0,0]` formula depends on this sign; getting it
            // backwards silently poisoned every real-stereo depth (see that class's
            // comment on _baselineM).
            var cal = StereoCalibrationData.CreateDefaults(baselineMetres: 0.84);
            cal.T[0].Should().BeApproximately(-0.84, 0.0001);
            cal.T[1].Should().BeApproximately(0.0, 0.0001, "T[1] should be zero (side-by-side rig)");
            cal.T[2].Should().BeApproximately(0.0, 0.0001, "T[2] should be zero (side-by-side rig)");
        }

        [Theory]
        [InlineData(0.60)]
        [InlineData(0.70)]
        [InlineData(0.84)]
        [InlineData(0.90)]
        public void CreateDefaults_DifferentBaselines_AllProducePositiveFocalLength(double b)
        {
            var cal = StereoCalibrationData.CreateDefaults(b);
            cal.T[0].Should().BeApproximately(-b, 0.0001, "T is negated baselineMetres — see CreateDefaults_BaselineStoredInT");
            cal.K0[0].Should().BeGreaterThan(0);
        }

        [Fact]
        public void CreateDefaults_ImageDimensionsAreCorrect()
        {
            // 720×540 confirmed on-site (Blackfly S BFS-PGE-04S2M, fixed resolution).
            var cal = StereoCalibrationData.CreateDefaults();
            cal.ImageWidth.Should().Be(720);
            cal.ImageHeight.Should().Be(540);
        }

        [Fact]
        public void CreateDefaults_PrincipalPointAtImageCentre()
        {
            var cal = StereoCalibrationData.CreateDefaults();
            double cx = cal.K0[2];
            double cy = cal.K0[5];
            cx.Should().BeApproximately(360.0, 1.0, "cx should be half of image width");
            cy.Should().BeApproximately(270.0, 1.0, "cy should be half of image height");
        }

        [Fact]
        public void CreateDefaults_SquarePixelSensor_FxEqualsFy()
        {
            // For a square-pixel sensor (true of this camera and virtually all
            // machine-vision sensors), fx and fy must be equal regardless of
            // resolution/aspect ratio — aspect ratio only shifts cx/cy, not the
            // fx/fy ratio. Regression test for a bug where fy was incorrectly
            // scaled by (cy/cx).
            var cal = StereoCalibrationData.CreateDefaults();
            cal.K0[4].Should().BeApproximately(cal.K0[0], 0.001, "fy should equal fx for a square-pixel sensor");
        }

        [Fact]
        public void CreateDefaults_CamerasSameIntrinsics()
        {
            // Matched camera pairs should have identical intrinsics
            var cal = StereoCalibrationData.CreateDefaults();
            cal.K1[0].Should().BeApproximately(cal.K0[0], 0.001, "fx should match between cameras");
            cal.K1[4].Should().BeApproximately(cal.K0[4], 0.001, "fy should match between cameras");
            cal.K1[2].Should().BeApproximately(cal.K0[2], 0.001, "cx should match between cameras");
        }

        [Fact]
        public void CreateDefaults_DistortionCoeffsAreZero()
        {
            // Default calibration assumes no distortion
            var cal = StereoCalibrationData.CreateDefaults();
            foreach (var d in cal.D0)
                d.Should().BeApproximately(0.0, 0.0001);
            foreach (var d in cal.D1)
                d.Should().BeApproximately(0.0, 0.0001);
        }

        [Fact]
        public void CreateDefaults_ArraySizesAreCorrect()
        {
            var cal = StereoCalibrationData.CreateDefaults();
            cal.K0.Length.Should().Be(9,  "3×3 intrinsic matrix");
            cal.D0.Length.Should().Be(5,  "5 distortion coefficients");
            cal.R.Length.Should().Be(9,   "3×3 rotation matrix");
            cal.T.Length.Should().Be(3,   "3×1 translation vector");
        }

        [Fact]
        public void SaveToFile_ThenLoadFromFile_RoundTripsAllFields()
        {
            var original = StereoCalibrationData.CreateDefaults(0.75);
            var path = Path.Combine(Path.GetTempPath(), $"cal_roundtrip_{Guid.NewGuid()}.json");
            try
            {
                original.SaveToFile(path);
                File.Exists(path).Should().BeTrue("file should exist after save");

                var loaded = StereoCalibrationData.LoadFromFile(path);

                loaded.T[0].Should().BeApproximately(original.T[0], 0.0001);
                loaded.K0[0].Should().BeApproximately(original.K0[0], 0.001);
                loaded.K0[4].Should().BeApproximately(original.K0[4], 0.001);
                loaded.K0[2].Should().BeApproximately(original.K0[2], 0.001);
                loaded.ImageWidth.Should().Be(original.ImageWidth);
                loaded.ImageHeight.Should().Be(original.ImageHeight);
            }
            finally
            {
                if (File.Exists(path)) File.Delete(path);
            }
        }

        [Fact]
        public void SaveToFile_ProducesReadableJson()
        {
            var cal = StereoCalibrationData.CreateDefaults();
            var path = Path.Combine(Path.GetTempPath(), $"cal_json_{Guid.NewGuid()}.json");
            try
            {
                cal.SaveToFile(path);
                var text = File.ReadAllText(path);
                text.Should().Contain("ImageWidth");
                text.Should().Contain("720");
            }
            finally
            {
                if (File.Exists(path)) File.Delete(path);
            }
        }
    }
}
