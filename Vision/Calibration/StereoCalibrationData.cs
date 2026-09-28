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

        // World-mounting geometry: how the STEREO PAIR AS A WHOLE sits relative to
        // the ground/tee, as opposed to R/T above (which only describe the two
        // cameras' pose relative to EACH OTHER). Stereo calibration never produces
        // this — it's a rig-installation fact, not a camera-intrinsic one — so it
        // defaults to 0 (a level, untilted rig, i.e. no correction applied; see
        // Triangulator's use of these). CreateDefaults() below fills in this rig's
        // confirmed on-site values; a real calibration JSON without these fields
        // deserializes to 0/0 (untitled) rather than silently applying a wrong tilt.
        public double CameraHeightM        { get; set; } = 0.0;
        public double CameraForwardOffsetM { get; set; } = 0.0;

        // --------------------------------------------------------
        // Defaults derived from on-site measurements (2026-07-22), NOT a real
        // calibration. REPLACE by running StereoCalibrator with a checkerboard —
        // see the class comment above.
        // --------------------------------------------------------
        // Exposed so anything simulating this rig (MockCameraManager) can generate
        // frames under the SAME camera model these defaults assume, instead of an
        // independently-guessed resolution/focal-length that silently disagrees.
        // A mismatch here doesn't throw — it just makes StereoRectifier's remap
        // sample from the wrong place, which can wash out real disparity almost
        // entirely (found live-testing the mock pipeline: true ~290px disparity at
        // working distance measured as only ~4-5px after rectification).
        public const int DefaultImageWidth  = 720;
        public const int DefaultImageHeight = 540;
        public const double DefaultMeasurementDistanceM = 3.015; // slant distance, camera to ball spot
        public const double DefaultHalfWidthM = 0.6175;          // half of the measured 123.5cm floor width
        public static double DefaultFx => (DefaultImageWidth / 2.0) / (DefaultHalfWidthM / DefaultMeasurementDistanceM);

        // Confirmed on-site mounting geometry (notes/14-session-2026-07-22-hardware-
        // bringup.md, "Mounting geometry" line): sensor 114in above the floor, 33in
        // horizontally forward of the tee/ball spot, aimed at the tee, no yaw. Also
        // referenced by MockCameraManager, which needs the SAME numbers to generate
        // synthetic frames under a camera model consistent with what Triangulator
        // assumes for its world-frame correction (see Triangulator.ToWorldFrame) —
        // a mismatch here wouldn't throw, it would just make mock-tested trajectories
        // silently disagree with what the real rig would report.
        public const double DefaultCameraHeightM        = 2.8956; // 114in
        public const double DefaultCameraForwardOffsetM = 0.8382; // 33in

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

            double cx = DefaultImageWidth  / 2.0;
            double cy = DefaultImageHeight / 2.0;

            // fx back-calculated from the FOV measurement (horizontal)
            double fx = DefaultFx;

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

            // Extrinsics: cameras are side-by-side, right cam offset by baseline on X axis.
            // Sign: OpenCV's stereo convention defines T such that a point in camera0's
            // frame maps to camera1's frame via X1 = R*X0 + T. If camera1 (right) sits
            // physically +baselineMetres to the right of camera0 (left) along X, then
            // the SAME point expressed in camera1's frame has a SMALLER X — so T's X
            // component must be NEGATIVE here. Verified empirically against this
            // Emgu.CV/OpenCV build via CvInvoke.StereoRectify: a positive T here made
            // Triangulator's standard `baseline = -P1[0,3]/P1[0,0]` formula come out
            // negative, which poisoned every real-stereo Z (TriangulateStereo's avgZ
            // range check rejected it) and silently fell back to Monocular's hardcoded
            // (0,0,0) on every frame, real detections included.
            d.R = new[] { 1.0,0,0, 0,1.0,0, 0,0,1.0 }; // no rotation between cams
            d.T = new[] { -baselineMetres, 0.0, 0.0 };

            d.ImageWidth  = DefaultImageWidth;
            d.ImageHeight = DefaultImageHeight;

            d.CameraHeightM        = DefaultCameraHeightM;
            d.CameraForwardOffsetM = DefaultCameraForwardOffsetM;

            return d;
        }

        public void SaveToFile(string path) =>
            File.WriteAllText(path, JsonSerializer.Serialize(this, new JsonSerializerOptions { WriteIndented = true }));

        public static StereoCalibrationData LoadFromFile(string path) =>
            JsonSerializer.Deserialize<StereoCalibrationData>(File.ReadAllText(path))
            ?? CreateDefaults();
    }
}