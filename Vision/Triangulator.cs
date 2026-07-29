// ------------------------------------------------------------
// Vision/Triangulator.cs
// ------------------------------------------------------------
// Given matched 2D detections from left + right rectified cameras,
// reconstructs the 3D world point using the Q disparity-to-depth
// matrix and OpenCV triangulatePoints.
// Wide baseline (500–900mm): good depth resolution at 1–5m range.
// ------------------------------------------------------------
using System;
using System.Drawing;
using System.Runtime.InteropServices;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using SportSimulator.Vision.Calibration;

namespace SportSimulator.Vision
{
    public enum TriangulationTier
    {
        FullStereo   = 1,  // both cams agree well       — high confidence
        BlendedStereo = 2, // both cams, moderate disagreement — medium confidence
        Monocular    = 3,  // only one cam detected ball  — low confidence
        KalmanOnly   = 4   // no detection this frame     — prediction only
    }

    public class TriangulatedPoint
    {
        public float X { get; set; }
        public float Y { get; set; }
        public float Z { get; set; }
        public float Disparity { get; set; }
        public float Confidence { get; set; }      // 0.0 – 1.0
        public TriangulationTier Tier { get; set; }
        public string TierReason { get; set; } = "";
    }

    public class Triangulator
    {
        private Mat? _P0, _P1;
        private double _baselineM;
        private double _fx, _fy, _cx, _cy;

        // World-mounting geometry (see StereoCalibrationData.CameraHeightM/
        // CameraForwardOffsetM) — how the stereo pair as a whole sits relative to
        // the tee, as opposed to R/T above (the two cameras' pose relative to EACH
        // OTHER). Defaults to 0/0 (a level, untilted rig) when unconfigured, in
        // which case ToWorldFrame below skips the tilt rotation entirely.
        private double _cameraHeightM, _cameraForwardOffsetM;
        private double _tiltRad, _slantD, _sinTilt, _cosTilt;

        // Tier thresholds (metres)
        private const float FullAgreementM    = 0.05f;  // < 5cm  → Tier 1
        private const float BlendedAgreementM = 0.15f;  // < 15cm → Tier 2, else bad match

        public void Configure(StereoCalibrationData cal)
        {
            _P0 = ArrayToMat(cal.P0, 3, 4);
            _P1 = ArrayToMat(cal.P1, 3, 4);
            _fx        =  cal.P0[0];
            _fy        =  cal.P0[5];
            _cx        =  cal.P0[2];
            _cy        =  cal.P0[6];
            // Standard OpenCV stereoRectify convention: baseline = -P1[0,3]/P1[0,0].
            _baselineM = -cal.P1[3] / cal.P1[0];

            _cameraHeightM        = cal.CameraHeightM;
            _cameraForwardOffsetM = cal.CameraForwardOffsetM;
            if (_cameraHeightM > 0)
            {
                _tiltRad = Math.Atan(_cameraForwardOffsetM / _cameraHeightM);
                _slantD  = Math.Sqrt(_cameraHeightM * _cameraHeightM + _cameraForwardOffsetM * _cameraForwardOffsetM);
                _sinTilt = Math.Sin(_tiltRad);
                _cosTilt = Math.Cos(_tiltRad);
            }

            Console.WriteLine($"[Triangulator] Baseline: {_baselineM*1000:F1}mm  fx: {_fx:F1}px");
        }

        // Converts from the raw disparity/DLT output — X relative to camera0's own
        // optical axis, Y/Z along the camera's own (possibly tilted) view axis — into
        // true world coordinates: X centered on the stereo pair's shared baseline
        // midpoint, Y true vertical height (up positive), Z true downrange distance
        // from the tee (toward the net, increasing as the ball actually flies away).
        //
        // Why this matters, found live: this rig's camera is mounted with a real,
        // confirmed ~16° tilt aimed BACKWARD at the tee (114in up, ~33in forward of
        // it — see notes/14-...md) — meaning "further along the camera's own view
        // axis" and "further downrange toward the net" are OPPOSITE directions.
        // Before this correction, a real shot's reported Z DECREASED as the ball
        // flew toward the net, and the middleware fed that straight to Unity as a
        // negative Z-velocity — sent the ball backward, away from the net, exactly
        // matching the raw camera-tilted axis's own (wrong, for this purpose)
        // sense of "forward". Every check earlier in this session looked at speed
        // magnitude and rough position plausibility, never this sign, so it stayed
        // invisible until tested end-to-end in Unity.
        //
        // The X recentering is unconditional (always needed, tilt or not). The Y/Z
        // rotation only applies when a real tilt is configured (_cameraHeightM > 0)
        // — an untilted/unconfigured rig (e.g. TriangulatorTests.cs's generic
        // fixture) passes Y/Z through unchanged, since camera-local and world axes
        // already coincide when there's no tilt to correct for.
        //
        // yLocal comes in via the standard pinhole formula (pixelY-cy)*Z/fy, which
        // is Y-DOWN (a physically higher point gives a SMALLER pixel row, hence a
        // more negative Y here) — this is exactly what the old, now-removed
        // "Flip Y: OpenCV Y-down -> Unity Y-up" step in SimulatorEngine used to
        // correct. The rotation math below was derived in terms of a Y-UP physical
        // quantity, so it needs that same negation applied to its input — missing
        // this initially caused the rotation to come out rotated into the wrong
        // quadrant entirely (caught by TriangulateStereo_TiltedRig_DownrangeShot_
        // ZIncreasesTowardNet, a controlled round-trip test, not by hand-checking
        // noisy live output).
        private (float x, float y, float z) ToWorldFrame(float xLocal, float yLocal, float zLocal)
        {
            float worldX = xLocal - (float)(_baselineM / 2.0);

            if (_cameraHeightM <= 0)
                return (worldX, yLocal, zLocal);

            float yPhysical = -yLocal;
            float worldY = (float)(yPhysical * _sinTilt + (_slantD - zLocal) * _cosTilt);
            float worldZ = (float)((_slantD - zLocal) * _sinTilt - yPhysical * _cosTilt);
            return (worldX, worldY, worldZ);
        }

        // ── Tier 1 / 2: full stereo — both cameras detected the ball ────────────
        public TriangulatedPoint TriangulateStereo(PointF pt0, PointF pt1)
        {
            float disparity = pt0.X - pt1.X;
            if (disparity <= 0.5f)
                return Monocular(pt0, "Negative disparity — ball behind baseline");

            // Disparity formula (fast)
            float Z1 = (float)(_fx * _baselineM / disparity);
            float X1 = (float)((pt0.X - _cx) * Z1 / _fx);
            float Y1 = (float)((pt0.Y - _cy) * Z1 / _fy);

            // DLT cross-check via OpenCV TriangulatePoints
            using var p0m = PointToMat(pt0);
            using var p1m = PointToMat(pt1);
            using var h4  = new Mat();
            CvInvoke.TriangulatePoints(_P0!, _P1!, p0m, p1m, h4);

            // TriangulatePoints outputs 4×N CV_32F when given CV_32F projection matrices.
            // Use Matrix<float> with indexer for reliable reading.
            using var dlt = new Matrix<float>(h4.Rows, h4.Cols);
            h4.CopyTo(dlt.Mat);
            float w  = dlt[3, 0];
            float X2 = dlt[0, 0] / w;
            float Y2 = dlt[1, 0] / w;
            float Z2 = dlt[2, 0] / w;

            float diffZ = Math.Abs(Z1 - Z2);
            float avgX  = (X1 + X2) * 0.5f;
            float avgY  = (Y1 + Y2) * 0.5f;
            float avgZ  = (Z1 + Z2) * 0.5f;

            if (avgZ < 0.05f || avgZ > 10f)
                return Monocular(pt0, $"Z={avgZ:F2}m out of range");

            if (diffZ < FullAgreementM)
            {
                // Tier 1: methods agree tightly
                float conf = 1.0f - (diffZ / FullAgreementM) * 0.2f; // 0.80–1.00
                var (wx1, wy1, wz1) = ToWorldFrame(avgX, avgY, avgZ);
                return new TriangulatedPoint
                {
                    X = wx1, Y = wy1, Z = wz1,
                    Disparity  = disparity,
                    Confidence = conf,
                    Tier       = TriangulationTier.FullStereo,
                    TierReason = $"Stereo agreement {diffZ*100:F1}cm"
                };
            }

            if (diffZ < BlendedAgreementM)
            {
                // Tier 2: moderate disagreement — weight disparity formula more heavily
                // (it's more stable for horizontal stereo rigs)
                float blend = 1f - (diffZ - FullAgreementM) / (BlendedAgreementM - FullAgreementM);
                float bX = X1 * blend + X2 * (1f - blend);
                float bY = Y1 * blend + Y2 * (1f - blend);
                float bZ = Z1 * blend + Z2 * (1f - blend);
                float conf = 0.4f + blend * 0.4f; // 0.40–0.80

                var (wx2, wy2, wz2) = ToWorldFrame(bX, bY, bZ);
                return new TriangulatedPoint
                {
                    X = wx2, Y = wy2, Z = wz2,
                    Disparity  = disparity,
                    Confidence = conf,
                    Tier       = TriangulationTier.BlendedStereo,
                    TierReason = $"Blended stereo, disagreement {diffZ*100:F1}cm"
                };
            }

            // Disagreement > 15cm — likely a mismatched detection; fall to monocular
            return Monocular(pt0, $"Stereo disagreement {diffZ*100:F1}cm > threshold");
        }

        // ── Tier 3: monocular fallback — one camera only ─────────────────────────
        // Uses the detected camera's 2D position + the Kalman tracker's own last
        // (world-frame) belief for depth. knownWorldY/knownWorldZ come straight
        // from KalmanBallTracker.LastState, which is in WORLD coordinates (same as
        // everything this class returns) — they get inverted back to a camera-local
        // depth here (see ToWorldFrame's comment on why local and world axes differ
        // on this rig) so the pinhole formula below sees a real depth-along-the-
        // camera's-own-axis, then the fresh pixel-derived X/Y get rotated back out
        // to world frame before returning. Passing a world-frame Z straight into the
        // old (pre-tilt-fix) version of this formula silently produced nonsense once
        // Triangulator started reporting world coordinates instead of camera-local
        // ones — this two-step (world→local depth→pinhole→local→world) round trip
        // is what keeps it correct either way.
        public TriangulatedPoint TriangulateMonocular(PointF pt, float knownWorldY, float knownWorldZ)
        {
            float zLocal = _cameraHeightM > 0
                ? (float)(_slantD - knownWorldY * _cosTilt - knownWorldZ * _sinTilt)
                : knownWorldZ;

            float xLocal = (float)((pt.X - _cx) * zLocal / _fx);
            float yLocal = (float)((pt.Y - _cy) * zLocal / _fy);

            var (wx, wy, wz) = ToWorldFrame(xLocal, yLocal, zLocal);
            return new TriangulatedPoint
            {
                X = wx, Y = wy, Z = wz,
                Disparity  = 0,
                Confidence = 0.35f,
                Tier       = TriangulationTier.Monocular,
                TierReason = "Single camera — Z from Kalman"
            };
        }

        // Internal monocular when stereo fails mid-computation
        private TriangulatedPoint Monocular(PointF pt, string reason)
        {
            // Without a known Z we return a point at origin with near-zero confidence;
            // SimulatorEngine will blend with Kalman prediction
            return new TriangulatedPoint
            {
                X = 0, Y = 0, Z = 0,
                Disparity  = 0,
                Confidence = 0.25f,
                Tier       = TriangulationTier.Monocular,
                TierReason = reason
            };
        }

        // Theoretical depth precision at distance Z_m for this rig.
        // With fx≈3700px and 700mm baseline:
        //   @ 1.0m →  0.27mm precision
        //   @ 2.0m →  1.08mm precision
        //   @ 3.0m →  2.43mm precision
        // Significantly better than the original 1067px estimate due to
        // the narrower FOV (more telephoto) of the AccuGolf cameras.
        public float DepthPrecisionMm(float Z_m) =>
            (float)(_baselineM > 0 ? (Z_m * Z_m * 1000f) / (_fx * _baselineM) : float.NaN);

        // SetTo(float[]) treats the array as a scalar — only the first element is used.
        // Matrix<float> with indexer is the reliable approach (same pattern as KalmanBallTracker).
        private static Mat PointToMat(PointF p)
        {
            using var m = new Matrix<float>(2, 1);
            m[0, 0] = p.X;
            m[1, 0] = p.Y;
            return m.Mat.Clone();
        }

        // TriangulatePoints requires CV_32F projection matrices — convert from double[].
        private static Mat ArrayToMat(double[] arr, int rows, int cols)
        {
            using var m = new Matrix<float>(rows, cols);
            for (int i = 0; i < rows; i++)
                for (int j = 0; j < cols; j++)
                    m[i, j] = (float)arr[i * cols + j];
            return m.Mat.Clone();
        }
    }
}