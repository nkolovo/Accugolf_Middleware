// ------------------------------------------------------------
// Vision/Calibration/StereoCalibrationData.cs
// ------------------------------------------------------------
using System;
using System.IO;
using System.Text.Json;
using Emgu.CV;
using Emgu.CV.Structure;

namespace SportSimulator.Vision.Calibration
{
    /// <summary>
    /// Holds intrinsics for both cameras and the stereo extrinsics (R, T).
    /// Load from JSON after running StereoCalibrator, or use defaults.
    /// </summary>
    public class StereoCalibrationData
    {
        // Camera 0 (left) intrinsics
        public double[] K0 { get; set; } = new double[9];   // 3x3 row-major
        public double[] D0 { get; set; } = new double[5];   // k1,k2,p1,p2,k3

        // Camera 1 (right) intrinsics
        public double[] K1 { get; set; } = new double[9];
        public double[] D1 { get; set; } = new double[5];

        // Stereo extrinsics: rotation matrix R (3x3) and translation T (3x1, metres)
        public double[] R  { get; set; } = new double[9];
        public double[] T  { get; set; } = new double[3];

        // Rectification outputs (computed, not calibrated — filled by StereoRectifier)
        public double[] R0 { get; set; } = new double[9];
        public double[] R1 { get; set; } = new double[9];
        public double[] P0 { get; set; } = new double[12];
        public double[] P1 { get; set; } = new double[12];
        public double[] Q  { get; set; } = new double[16];  // disparity-to-depth map

        public int ImageWidth  { get; set; } = 720;
        public int ImageHeight { get; set; } = 540;

        // --------------------------------------------------------
        // Defaults derived from on-site measurements (2026-07-22), NOT a real
        // calibration. REPLACE by running StereoCalibrator with a checkerboard —
        // see the class comment above.
        // --------------------------------------------------------
        public static StereoCalibrationData CreateDefaults(double baselineMetres = 0.4953)
        {
            // Resolved on-site 2026-07-22:
            //
            // 1. BASELINE (parameter above, 0.4953m) — 19.5in measured center-to-center.
            //
            // 2. SENSOR RESOLUTION — confirmed 720×540 (Blackfly S BFS-PGE-04S2M,
            //    fixed/locked — no ROI windowing available on this model).
            //
            // 3. FOV MEASUREMENT — floor footprint measured under the camera's FOV:
            //    123.5cm × 98cm. Used here as an approximate frontal-plane width at
            //    the slant distance to the ball spot (≈3.015m, from the 114in
            //    height / 33in forward-offset mounting geometry) — NOT a rigorous
            //    derivation. The floor is a plane tilted ~16° relative to the
            //    camera's view axis, so this footprint is really a trapezoid, not
            //    a clean frontal rectangle; treating it as one introduces error
            //    that grows toward the near/far edges of the visible floor patch.
            //    Good enough as a placeholder, not as calibration.
            //
            // Once StereoCalibrator has been run and stereo_calibration.json exists,
            // these defaults are only used as a fallback if the JSON is missing.

            const double MEASUREMENT_DISTANCE_M = 3.015; // slant distance, camera to ball spot
            const double HALF_WIDTH_M = 0.6175;           // half of the measured 123.5cm floor width

            const int IMAGE_WIDTH  = 720;
            const int IMAGE_HEIGHT = 540;

            double cx = IMAGE_WIDTH  / 2.0;
            double cy = IMAGE_HEIGHT / 2.0;

            // fx back-calculated from the FOV measurement (horizontal)
            double fx = (IMAGE_WIDTH / 2.0) / (HALF_WIDTH_M / MEASUREMENT_DISTANCE_M);

            // fy = fx. For a square-pixel sensor (true of virtually all machine-vision
            // sensors, including this one), the pixel-to-radian conversion is identical
            // in both axes regardless of resolution or aspect ratio — aspect ratio only
            // affects cx/cy, not the fx/fy ratio. (The previous `fx * (cy/cx)` formula
            // here was wrong: for 1280×1024 it computed fy = 0.8·fx while its own
            // comment claimed fy = fx for that exact resolution — contradictory.)
            double fy = fx;

            var d = new StereoCalibrationData();

            // Left camera — identity-ish intrinsics
            d.K0 = new[] { fx, 0, cx,  0, fy, cy,  0, 0, 1.0 };
            d.D0 = new[] { 0.0, 0.0, 0.0, 0.0, 0.0 };

            // Right camera — same intrinsics (assumed matched pair)
            d.K1 = new[] { fx, 0, cx,  0, fy, cy,  0, 0, 1.0 };
            d.D1 = new[] { 0.0, 0.0, 0.0, 0.0, 0.0 };

            // Extrinsics: cameras are side-by-side, right cam offset by baseline on X axis
            d.R = new[] { 1.0,0,0, 0,1.0,0, 0,0,1.0 }; // no rotation between cams
            d.T = new[] { baselineMetres, 0.0, 0.0 };

            d.ImageWidth  = IMAGE_WIDTH;
            d.ImageHeight = IMAGE_HEIGHT;

            return d;
        }

        public void SaveToFile(string path) =>
            File.WriteAllText(path, JsonSerializer.Serialize(this, new JsonSerializerOptions { WriteIndented = true }));

        public static StereoCalibrationData LoadFromFile(string path) =>
            JsonSerializer.Deserialize<StereoCalibrationData>(File.ReadAllText(path))
            ?? CreateDefaults();
    }
}