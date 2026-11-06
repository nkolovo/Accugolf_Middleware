// ------------------------------------------------------------
// Vision/StereoClock.cs
// ------------------------------------------------------------
// Puts each stereo cycle's timestamp on one clock (camera 0's).
//
// A frame's TimestampUs comes from that camera's own device clock, and the two
// cameras' clocks aren't related: each counts from its own power-up. A cycle where
// only camera 1 saw the ball therefore handed KalmanBallTracker a time from another
// clock, and a cycle with no detection handed it wall-clock time. Either way the
// tracker clamped the bogus gap to its 0.5s maximum and jumped the ball forward by
// half a second of flight (found simulating a kick on net48: Z=7.69m one frame after
// the ball left view). The mock never showed this because its cameras share a clock.
// ------------------------------------------------------------
namespace SportSimulator.Vision
{
    public class StereoClock
    {
        private const long FrameIntervalUs = 3817; // the real 262fps

        private long _cam1ToCam0Us;
        private long _lastUs;

        /// <param name="cam0Us">Camera 0's detection time this cycle, if it detected the ball.</param>
        /// <param name="cam1Us">Camera 1's detection time this cycle, if it detected the ball.</param>
        public long Next(long? cam0Us, long? cam1Us)
        {
            if (cam0Us.HasValue && cam1Us.HasValue)
                _cam1ToCam0Us = cam0Us.Value - cam1Us.Value;

            if (cam0Us.HasValue)      _lastUs = cam0Us.Value;
            else if (cam1Us.HasValue) _lastUs = cam1Us.Value + _cam1ToCam0Us;
            else                      _lastUs += FrameIntervalUs; // a cycle is one frame per camera

            return _lastUs;
        }
    }
}
