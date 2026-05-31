// ------------------------------------------------------------
// Tracking/KalmanBallTracker.cs
// ------------------------------------------------------------
// Simple 6-state Kalman: [x, y, z, vx, vy, vz]
//
// Emgu.CV 4.9 matrix-writing notes:
//   Marshal.Copy to Mat.DataPointer does NOT reliably update KalmanFilter's
//   internal matrices — the native KalmanFilter struct may hold its own
//   pointers that differ from the managed Mat wrapper's DataPointer.
//
//   Reliable approaches:
//     • Diagonal matrices  → CvInvoke.SetIdentity(mat, scalar)
//     • Arbitrary matrices → build a Matrix<float> (has [r,c] indexer),
//                            then call matrix.Mat.CopyTo(target)
//     • Reading state      → copy to a Matrix<float> and read [r,c]
// ------------------------------------------------------------
using System;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using SportSimulator.Models;

namespace SportSimulator.Tracking
{
    public class KalmanBallTracker
    {
        private KalmanFilter? _kf;
        private bool _initialized;

        public void Configure(SportProfile profile)
        {
            _kf = new KalmanFilter(6, 3, 0, DepthType.Cv32F);
            float q = profile.ProcessNoise;
            float r = profile.MeasurementNoise;

            float dt = 1f / 120f; // constant-velocity model at 120 fps

            SetMatrix(_kf.TransitionMatrix, new float[,]
            {
                {1,0,0,dt,0,0},
                {0,1,0,0,dt,0},
                {0,0,1,0,0,dt},
                {0,0,0,1,0,0},
                {0,0,0,0,1,0},
                {0,0,0,0,0,1}
            });

            SetMatrix(_kf.MeasurementMatrix, new float[,]
            {
                {1,0,0,0,0,0},
                {0,1,0,0,0,0},
                {0,0,1,0,0,0}
            });

            // CvInvoke.SetIdentity is the reliable way to set diagonal Mats
            CvInvoke.SetIdentity(_kf.ProcessNoiseCov,     new MCvScalar(q));
            CvInvoke.SetIdentity(_kf.MeasurementNoiseCov, new MCvScalar(r));
            CvInvoke.SetIdentity(_kf.ErrorCovPost,        new MCvScalar(1.0));

            _initialized = false;
        }

        public (float x, float y, float z, float vx, float vy, float vz) Update(
            float mx, float my, float mz)
        {
            if (_kf == null) return (mx, my, mz, 0, 0, 0);

            if (!_initialized)
            {
                using var init = new Matrix<float>(6, 1);
                init[0, 0] = mx; init[1, 0] = my; init[2, 0] = mz;
                // vx, vy, vz stay at 0 (Matrix zeroed on construction)
                init.Mat.CopyTo(_kf.StatePost);
                _initialized = true;
            }

            _kf.Predict();

            using var meas = new Matrix<float>(3, 1);
            meas[0, 0] = mx; meas[1, 0] = my; meas[2, 0] = mz;

            CacheState(_kf.Correct(meas.Mat));
            return LastState;
        }

        /// <summary>Predict-only step (Tier 4 coasting).</summary>
        public (float x, float y, float z, float vx, float vy, float vz) Predict()
        {
            if (_kf == null || !_initialized) return LastState;
            CacheState(_kf.Predict());
            return LastState;
        }

        public (float x, float y, float z, float vx, float vy, float vz) LastState
        { get; private set; }

        // ── Helpers ──────────────────────────────────────────────────────────

        private void CacheState(Mat state)
        {
            // state is a 6x1 CV_32F Mat returned by Predict/Correct
            using var m = new Matrix<float>(state.Rows, 1);
            state.CopyTo(m.Mat);
            LastState = (m[0,0], m[1,0], m[2,0], m[3,0], m[4,0], m[5,0]);
        }

        // Build a Matrix<float> from a 2-D array and copy it into a target Mat.
        private static void SetMatrix(Mat target, float[,] vals)
        {
            int rows = vals.GetLength(0), cols = vals.GetLength(1);
            using var m = new Matrix<float>(rows, cols);
            for (int r = 0; r < rows; r++)
                for (int c = 0; c < cols; c++)
                    m[r, c] = vals[r, c];
            m.Mat.CopyTo(target);
        }
    }
}
