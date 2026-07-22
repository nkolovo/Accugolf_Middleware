// ------------------------------------------------------------
// Vision/SpinEstimator.cs
// ------------------------------------------------------------
// Estimates ball spin by rotationally correlating two consecutive ball-crop
// images from the SAME camera (dimple/panel/seam texture tracking).
//
// ⚠️ SCOPE — read before trusting this for anything but soccer:
//
// This only resolves rotations up to ~SearchRangeDeg between frames before
// aliasing makes the match ambiguous (a texture pattern rotated 150° often
// scores similarly to one rotated -30° — same reason a spoked wheel looks
// like it spins backward on video). At the 120fps this engine assumes
// (KalmanBallTracker.cs, dt = 1/120f):
//
//   °/frame = rpm * 6 * dt  →  rpm = °/frame / (6 * dt)
//
//   Soccer   ~300–600 rpm  → 15–30°/frame   — well inside range, reliable
//   Baseball ~1500–3000rpm → 75–150°/frame  — aliased, unreliable
//   Tennis   ~1500–5000rpm → 75–250°/frame  — aliased, unreliable
//   Golf     ~2500–9000rpm → 125–450°/frame — aliased, unreliable
//
// For the faster sports, this needs a higher-fps burst capture around impact
// (decoupled from the steady 120fps tracking loop) before it'll produce
// trustworthy numbers — see the roadmap note on burst capture. Don't wire
// this into golf/baseball/tennis profiles expecting real numbers yet.
//
// SINGLE-CAMERA LIMITATION: this measures the rotation visible face-on to
// ONE camera — i.e. the spin component about that camera's own viewing
// axis. It does not decompose full 3D backspin/sidespin; that requires
// comparing both cameras' near-simultaneous crops against each other, which
// is a follow-up step once this magnitude estimate is validated against
// real footage.
// ------------------------------------------------------------
using System;
using System.Drawing;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;

namespace SportSimulator.Vision
{
    public class SpinMeasurement
    {
        public bool Valid { get; set; }
        public float Rpm { get; set; }
        public float AngleDeg { get; set; }   // signed rotation found between the two crops
        public float Confidence { get; set; } // 0.0–1.0, unvalidated heuristic — see MinConfidence
    }

    public class SpinEstimator
    {
        // ±60° comfortably covers soccer's ~15–30°/frame at 120fps while staying
        // under the ~90° aliasing ceiling. Widen only if you also raise capture fps.
        private const int SearchRangeDeg = 60;
        private const int CoarseStepDeg  = 2;

        // Required improvement of the best-fit angle's correlation score over the
        // 0°-rotation baseline before a measurement is trusted. This is a starting
        // guess, not a validated threshold — tune against real footage.
        private const float MinConfidence = 0.15f;

        public SpinMeasurement Estimate(byte[] cropA, byte[] cropB, int size, float dtSeconds)
        {
            if (cropA.Length != size * size || cropB.Length != size * size || dtSeconds <= 0f)
                return new SpinMeasurement { Valid = false };

            using var matA = BytesToMat(cropA, size);
            using var matB = BytesToMat(cropB, size);

            float baselineScore = Correlate(matA, matB, 0f);
            float bestAngle = 0f, bestScore = baselineScore;

            for (int deg = -SearchRangeDeg; deg <= SearchRangeDeg; deg += CoarseStepDeg)
            {
                if (deg == 0) continue;
                float score = Correlate(matA, matB, deg);
                if (score > bestScore) { bestScore = score; bestAngle = deg; }
            }

            float confidence = MathF.Max(0f, MathF.Min(1f, bestScore - baselineScore));
            if (bestAngle == 0f || confidence < MinConfidence)
                return new SpinMeasurement { Valid = false, Confidence = confidence };

            float rpm = Math.Abs(bestAngle) / dtSeconds / 6f; // deg/s ÷ 6 = rpm

            return new SpinMeasurement
            {
                Valid      = true,
                AngleDeg   = bestAngle,
                Rpm        = rpm,
                Confidence = confidence
            };
        }

        // Rotate A by `deg` about its own center and score normalized correlation
        // against B. MatchTemplate with image and template the same size collapses
        // to a single-cell result containing the normalized correlation coefficient
        // at that one alignment — a cheap way to get an NCC score without hand-rolling
        // the mean/variance math.
        private static float Correlate(Mat matA, Mat matB, float deg)
        {
            using var rotated = new Mat();
            if (deg == 0f)
            {
                matA.CopyTo(rotated);
            }
            else
            {
                var center = new PointF(matA.Cols / 2f, matA.Rows / 2f);
                using var rotMatrix = new Mat();
                CvInvoke.GetRotationMatrix2D(center, deg, 1.0, rotMatrix);
                CvInvoke.WarpAffine(matA, rotated, rotMatrix, matA.Size);
            }

            using var result = new Mat();
            CvInvoke.MatchTemplate(matB, rotated, result, TemplateMatchingType.CcoeffNormed);

            using var m = new Matrix<float>(1, 1);
            result.CopyTo(m.Mat);
            return m[0, 0];
        }

        private static Mat BytesToMat(byte[] data, int size)
        {
            var m = new Mat(size, size, DepthType.Cv8U, 1);
            m.SetTo(data);
            return m;
        }
    }
}
