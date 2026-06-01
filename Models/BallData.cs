// ------------------------------------------------------------
// Models/BallData.cs
// ------------------------------------------------------------
namespace SportSimulator.Models
{
    public class BallData
    {
        public string SportId { get; set; } = "";
        public long TimestampUs { get; set; }       // microseconds since epoch

        // 3D position (meters, Unity coordinate space)
        public float PosX { get; set; }
        public float PosY { get; set; }
        public float PosZ { get; set; }

        // Velocity vector (m/s)
        public float VelX { get; set; }
        public float VelY { get; set; }
        public float VelZ { get; set; }

        public float SpeedMps { get; set; }
        public float LaunchAngleDeg { get; set; }   // null-equiv 0 if not applicable

        // Spin (rpm) — omitted/zero if profile.OutputSpin == false
        public float SpinRpm { get; set; }
        public float SpinAxisX { get; set; }
        public float SpinAxisY { get; set; }
        public float SpinAxisZ { get; set; }

        // Puck-specific
        public float TiltAngleDeg { get; set; }     // only if profile.OutputTilt

        public float Confidence { get; set; }       // 0.0 – 1.0
        public int   TrackingTier { get; set; }     // 1=FullStereo 2=Blended 3=Monocular 4=KalmanOnly
    }
}