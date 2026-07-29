// ------------------------------------------------------------
// Models/SportProfile.cs
// ------------------------------------------------------------
namespace SportSimulator.Models
{
    public class SportProfile
    {
        public string SportId { get; set; } = "generic";
        public string DisplayName { get; set; } = "Generic";

        // Ball physical properties
        public float DiameterMm { get; set; }       // 0 = not a sphere (e.g. puck)
        public float MassGrams { get; set; }
        public bool IsSphere { get; set; } = true;  // false for hockey puck

        // Detection tuning
        public int MinContourArea { get; set; }
        public int MaxContourArea { get; set; }

        // Used by KalmanBallTracker's rest-position seeding to tell a real shot
        // apart from the ball just being nudged/repositioned at address: the
        // settling gate (MinRestSamplesForConfidence) only catches continuous
        // motion that never stops, not a ball that sat still for a while and
        // then got moved a few cm to adjust its placement. If the IMPLIED speed
        // of a displacement (position delta / elapsed time) is below this, it's
        // treated as a new candidate rest position instead of a shot. Set well
        // below the sport's realistic slowest real shot, comfortably above a
        // hand adjustment's speed — see SportProfileRegistry for per-sport
        // values and the golf/putt caveat (a genuinely slow, deliberate shot can
        // still be indistinguishable from a nudge by speed alone).
        public double MinSpeedMps { get; set; }
        public double MaxSpeedMps { get; set; }
        public bool UseInfrared { get; set; } = false;

        // Output schema flags
        public bool OutputSpin { get; set; } = true;
        public bool OutputLaunchAngle { get; set; } = true;
        public bool OutputTilt { get; set; } = false; // puck tilt angle

        // Kalman filter tuning
        public float ProcessNoise { get; set; } = 0.01f;
        public float MeasurementNoise { get; set; } = 0.1f;

        // Kalman coast window: max frames to predict forward without a detection.
        // Derived from how long the ball is realistically in the simulator's FOV.
        // Fast sports need short windows to avoid over-projecting; slow sports can
        // tolerate longer windows to cover genuine occlusion.
        //
        // Formula reference (700mm simulator, 120fps):
        //   frames_in_view ≈ (simulator_length_m / typical_speed_mps) * fps
        //   coast = frames_in_view + small_occlusion_buffer (1–3 frames)
        public int KalmanCoastFrames { get; set; } = 5;
    }
}