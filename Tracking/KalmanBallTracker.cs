// ------------------------------------------------------------
// Tracking/KalmanBallTracker.cs
// ------------------------------------------------------------
using System;
using Emgu.CV;
using Emgu.CV.Structure;
using SportSimulator.Models;

namespace SportSimulator.Tracking
{
    // Simple 6-state Kalman: [x, y, z, vx, vy, vz]
    public class KalmanBallTracker
    {
        private KalmanFilter? _kf;
        private bool _initialized;

        public void Configure(SportProfile profile)
        {
            _kf = new KalmanFilter(6, 3, 0, Emgu.CV.CvEnum.DepthType.Cv32F);
            float q = profile.ProcessNoise;
            float r = profile.MeasurementNoise;

            // Transition matrix (constant velocity model)
            float dt = 1f / 120f; // assume 120 fps
            var F = new float[,]
            {
                {1,0,0,dt,0,0},
                {0,1,0,0,dt,0},
                {0,0,1,0,0,dt},
                {0,0,0,1,0,0},
                {0,0,0,0,1,0},
                {0,0,0,0,0,1}
            };
            SetMatrix(_kf.TransitionMatrix, F);

            // Measurement matrix: observe x,y,z only
            var H = new float[,]
            {
                {1,0,0,0,0,0},
                {0,1,0,0,0,0},
                {0,0,1,0,0,0}
            };
            SetMatrix(_kf.MeasurementMatrix, H);

            SetIdentity(_kf.ProcessNoiseCov, q);
            SetIdentity(_kf.MeasurementNoiseCov, r);
            SetIdentity(_kf.ErrorCovPost, 1f);
            _initialized = false;
        }

        public (float x, float y, float z, float vx, float vy, float vz) Update(float mx, float my, float mz)
        {
            if (_kf == null) return (mx, my, mz, 0, 0, 0);

            if (!_initialized)
            {
                _kf.StatePost[0, 0] = mx;
                _kf.StatePost[1, 0] = my;
                _kf.StatePost[2, 0] = mz;
                _initialized = true;
            }

            _kf.Predict();

            using var measurement = new Matrix<float>(3, 1);
            measurement[0, 0] = mx; measurement[1, 0] = my; measurement[2, 0] = mz;
            var state = _kf.Correct(measurement);
            CacheState(state);
            return LastState;
        }

        // Predict-only step (Tier 4 coasting) — advances filter without a measurement
        public (float x, float y, float z, float vx, float vy, float vz) Predict()
        {
            if (_kf == null || !_initialized) return LastState;
            var state = _kf.Predict();
            CacheState(state);
            return LastState;
        }

        public (float x, float y, float z, float vx, float vy, float vz) LastState { get; private set; }

        private void CacheState(Mat state) =>
            LastState = (state[0,0], state[1,0], state[2,0],
                         state[3,0], state[4,0], state[5,0]);

        private void SetMatrix(Mat m, float[,] vals)
        {
            for (int r = 0; r < vals.GetLength(0); r++)
                for (int c = 0; c < vals.GetLength(1); c++)
                    m.SetValue(r, c, vals[r, c]);
        }

        private void SetIdentity(Mat m, float scale)
        {
            for (int i = 0; i < Math.Min(m.Rows, m.Cols); i++)
                m.SetValue(i, i, scale);
        }
    }
}