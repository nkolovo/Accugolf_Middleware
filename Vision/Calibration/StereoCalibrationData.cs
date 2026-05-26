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

        public int ImageWidth  { get; set; } = 1280;
        public int ImageHeight { get; set; } = 1024;

        // --------------------------------------------------------
        // Defaults: 6mm lens on a 1/1.8" sensor, 700mm baseline.
        // REPLACE these with your actual calibration output.
        // --------------------------------------------------------
        public static StereoCalibrationData CreateDefaults(double baselineMetres = 0.70)
        {
            // ⚠️ CALIBRATION TODO — update these before first use:
            //
            // 1. BASELINE (parameter above, default 0.70m)
            //    Measure the physical centre-to-centre distance between your two
            //    AccuGolf cameras in metres. If you measured centre-to-edge = Xmm,
            //    baseline = 2 * X / 1000.0.
            //    e.g. centre-to-edge = 350mm → baseline = 0.70
            //         centre-to-edge = 420mm → baseline = 0.84
            //
            // 2. SENSOR RESOLUTION  ← update imageWidth / imageHeight / cx / cy below
            //    Check SpinView or Spinnaker DeviceInfo for your camera's native
            //    resolution. Common FLIR options:
            //      1280×1024 (5:4)  → cx=640, cy=512   fy=fx
            //      1280×960  (4:3)  → cx=640, cy=480   fy=fx*(480/640)
            //      1280×720  (16:9) → cx=640, cy=360   fy=fx*(360/640)
            //
            // 3. FOV MEASUREMENT  ← determines fx (and fy for non-square sensors)
            //    You measured: at D inches from the camera, the visible horizontal
            //    width is ~1000mm (500mm either side of centre).
            //    Formula: fx = (imageWidth/2) / (0.500 / D_metres)
            //    Update MEASUREMENT_DISTANCE_M and HALF_WIDTH_M below with your values.
            //    Current values: 108–120in range → midpoint ~3700px.
            //
            // Once StereoCalibrator has been run and stereo_calibration.json exists,
            // these defaults are only used as a fallback if the JSON is missing.

            // ── ⚠️ UPDATE THESE ────────────────────────────────────────────────
            const double MEASUREMENT_DISTANCE_M = 2.8956; // midpoint of 108in–120in
                                                           // replace with your actual
                                                           // measured distance in metres
                                                           // (108in=2.7432, 120in=3.048)
            const double HALF_WIDTH_M = 0.500;            // half of visible horizontal
                                                           // width at above distance (m)
                                                           // you measured ~1000mm total
                                                           //  → 500mm half-width

            const int IMAGE_WIDTH  = 1280; // ⚠️ confirm against your FLIR sensor spec
            const int IMAGE_HEIGHT = 1024; // ⚠️ confirm against your FLIR sensor spec
            // ───────────────────────────────────────────────────────────────────

            double cx = IMAGE_WIDTH  / 2.0;
            double cy = IMAGE_HEIGHT / 2.0;

            // fx back-calculated from FOV measurement (horizontal)
            double fx = (IMAGE_WIDTH / 2.0) / (HALF_WIDTH_M / MEASUREMENT_DISTANCE_M);

            // fy: equal to fx for square-pixel sensors with 5:4 aspect (1280×1024).
            // For other resolutions, fy = fx * (cy / cx).
            // This formula is general — it stays correct if you update IMAGE_HEIGHT above.
            double fy = fx * (cy / cx);

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