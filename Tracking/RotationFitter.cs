// ------------------------------------------------------------
// Tracking/RotationFitter.cs
// ------------------------------------------------------------
// Fits the best-fit rigid rotation between two matched 3D point sets using
// the Kabsch algorithm (SVD of the cross-covariance matrix). This is what
// makes full 3D spin (axis + magnitude) recoverable, as opposed to
// SpinEstimator's single-camera 2D correlation, which can only see the
// rotation component about the camera's own viewing axis — blind to any
// axis lying within the image plane (e.g. backspin/topspin viewed from
// nearly overhead, exactly this rig's situation).
//
// Points must already correspond by INDEX — setA[i] and setB[i] must be the
// same physical feature at two different times. This class does no point
// matching/tracking itself, only fits the rotation given correspondences.
// ------------------------------------------------------------
using System;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;

namespace SportSimulator.Tracking
{
    public readonly struct Vec3
    {
        public readonly float X, Y, Z;
        public Vec3(float x, float y, float z) { X = x; Y = y; Z = z; }
        public static Vec3 operator -(Vec3 a, Vec3 b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
    }

    public class RotationFit
    {
        public bool Valid { get; set; }
        public Vec3 AxisUnit { get; set; }  // unit rotation axis, in whatever frame the input points were in
        public float AngleDeg { get; set; } // rotation magnitude >= 0 — direction is encoded via AxisUnit (right-hand rule)
    }

    public static class RotationFitter
    {
        // 3 points is the mathematical minimum for a unique rotation; more is
        // more robust to per-point triangulation noise. Below this, don't guess.
        private const int MinPoints = 3;

        public static RotationFit Fit(Vec3[] setA, Vec3[] setB)
        {
            if (setA.Length != setB.Length || setA.Length < MinPoints)
                return new RotationFit { Valid = false };

            int n = setA.Length;
            var centroidA = Centroid(setA);
            var centroidB = Centroid(setB);

            // Cross-covariance H = sum_i (a_i - centroidA) ⊗ (b_i - centroidB)
            var h = new double[3, 3];
            for (int i = 0; i < n; i++)
            {
                var a = setA[i] - centroidA;
                var b = setB[i] - centroidB;
                h[0, 0] += (double)a.X * b.X; h[0, 1] += (double)a.X * b.Y; h[0, 2] += (double)a.X * b.Z;
                h[1, 0] += (double)a.Y * b.X; h[1, 1] += (double)a.Y * b.Y; h[1, 2] += (double)a.Y * b.Z;
                h[2, 0] += (double)a.Z * b.X; h[2, 1] += (double)a.Z * b.Y; h[2, 2] += (double)a.Z * b.Z;
            }

            using var hMat = new Matrix<double>(3, 3);
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    hMat[r, c] = h[r, c];

            using var wMat  = new Matrix<double>(3, 3);
            using var uMat  = new Matrix<double>(3, 3);
            using var vtMat = new Matrix<double>(3, 3); // V^T, per OpenCV SVD::compute convention
            CvInvoke.SVDecomp(hMat.Mat, wMat.Mat, uMat.Mat, vtMat.Mat, SvdFlag.Default);

            var U  = ToArray(uMat);
            var Vt = ToArray(vtMat);
            var V  = Transpose(Vt);
            var Ut = Transpose(U);

            // Kabsch reflection fix: without this, a reflection (det = -1) can win
            // out over a true rotation when the point geometry is close to planar.
            double d = Determinant(Multiply(V, Ut));
            double sign = d < 0 ? -1.0 : 1.0;
            var D = new double[3, 3] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, sign } };

            var R = Multiply(Multiply(V, D), Ut);

            return DecomposeAxisAngle(R);
        }

        // Standard axis-angle (Rodrigues) extraction from a rotation matrix.
        // Degenerates near 0° (no rotation, handled explicitly) and near 180°
        // (not handled — not expected in our per-frame rotation range; see
        // SpinEstimator.cs comments on why frames must stay near-consecutive).
        private static RotationFit DecomposeAxisAngle(double[,] R)
        {
            double trace = R[0, 0] + R[1, 1] + R[2, 2];
            double cosAngle = (trace - 1.0) / 2.0;
            if (cosAngle < -1.0) cosAngle = -1.0;
            if (cosAngle > 1.0) cosAngle = 1.0;
            double angleRad = Math.Acos(cosAngle);

            if (angleRad < 1e-6)
                return new RotationFit { Valid = true, AngleDeg = 0f, AxisUnit = new Vec3(0, 0, 1) };

            double sinAngle = Math.Sin(angleRad);
            double ax = (R[2, 1] - R[1, 2]) / (2 * sinAngle);
            double ay = (R[0, 2] - R[2, 0]) / (2 * sinAngle);
            double az = (R[1, 0] - R[0, 1]) / (2 * sinAngle);

            return new RotationFit
            {
                Valid    = true,
                AngleDeg = (float)(angleRad * 180.0 / Math.PI),
                AxisUnit = new Vec3((float)ax, (float)ay, (float)az)
            };
        }

        private static Vec3 Centroid(Vec3[] pts)
        {
            float sx = 0, sy = 0, sz = 0;
            foreach (var p in pts) { sx += p.X; sy += p.Y; sz += p.Z; }
            return new Vec3(sx / pts.Length, sy / pts.Length, sz / pts.Length);
        }

        // ── Minimal hand-rolled 3×3 linear algebra ──────────────────────────────
        // Kept separate from Emgu.CV's Matrix<T> for this part (only used for the
        // SVD call itself) so transpose/multiply/determinant are simple enough to
        // read and verify directly, rather than trusting indexer/shape conventions
        // through several chained Matrix<T> operations.

        private static double[,] ToArray(Matrix<double> m)
        {
            var a = new double[3, 3];
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    a[r, c] = m[r, c];
            return a;
        }

        private static double[,] Multiply(double[,] a, double[,] b)
        {
            var result = new double[3, 3];
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                {
                    double sum = 0;
                    for (int k = 0; k < 3; k++) sum += a[r, k] * b[k, c];
                    result[r, c] = sum;
                }
            return result;
        }

        private static double[,] Transpose(double[,] a)
        {
            var result = new double[3, 3];
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    result[c, r] = a[r, c];
            return result;
        }

        private static double Determinant(double[,] a) =>
              a[0, 0] * (a[1, 1] * a[2, 2] - a[1, 2] * a[2, 1])
            - a[0, 1] * (a[1, 0] * a[2, 2] - a[1, 2] * a[2, 0])
            + a[0, 2] * (a[1, 0] * a[2, 1] - a[1, 1] * a[2, 0]);
    }
}
