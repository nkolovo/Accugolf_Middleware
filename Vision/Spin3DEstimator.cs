// ------------------------------------------------------------
// Vision/Spin3DEstimator.cs
// ------------------------------------------------------------
// Orchestrates full 3D spin measurement: FeaturePointTracker (natural-
// texture detection + optical-flow tracking + stereo matching) feeds
// RotationFitter (Kabsch rigid-rotation fit) to recover both spin
// MAGNITUDE and AXIS — unlike SpinEstimator's single-camera 2D correlation,
// which can only see the rotation component about that one camera's own
// viewing axis (blind to backspin/topspin from this rig's overhead angle —
// see SpinEstimator.cs and RotationFitter.cs comments).
//
// AxisUnit is returned in the SAME coordinate frame as Triangulator's
// output (camera-relative), NOT yet rotated into shot-relative backspin/
// sidespin/spiral terms. That labeling is a follow-on step using the rig's
// known mounting tilt (~16.15° — see App/SimulatorEngine.cs) once this
// magnitude+axis estimate has been validated against real footage.
//
// Kabsch only needs points to correspond by identity across two instants —
// it centers each point set on its own centroid internally, so the ball's
// own translation (flight path) doesn't need to be subtracted separately.
//
// Natural-texture tracking — reliability against real ball surfaces under
// real lighting is UNVALIDATED. See FeaturePointTracker.cs.
// ------------------------------------------------------------
using System.Collections.Generic;
using System.Drawing;
using Emgu.CV;
using SportSimulator.Tracking;

namespace SportSimulator.Vision
{
    public class Spin3DMeasurement
    {
        public bool Valid { get; set; }
        public float Rpm { get; set; }
        public Vec3 AxisUnit { get; set; }  // camera-relative — see class comment
        public int PointsUsed { get; set; } // matched-point count the fit used — rough confidence signal
    }

    public class Spin3DEstimator
    {
        // Mirrors RotationFitter's own minimum — no point calling Fit() below this.
        private const int MinMatchedPoints = 3;

        // Same staleness guard as SpinEstimator.cs: a gap this large (dropped
        // frame, coast period, camera switch) isn't a "near-consecutive" pair.
        private const float MaxDtSeconds = 0.05f;

        // No sport this rig actually outputs spin for gets remotely close to this —
        // soccer tops out around 300–600rpm, a tumbling field-goal kick less than
        // that (golf's much higher real spin is explicitly out of scope for this
        // middleware). A reading anywhere near this ceiling is de-facto a bad
        // point-correspondence fit, not real signal — natural-texture stereo
        // matching can mismatch when the ball has several similar-looking patches
        // close together (found via the mock's placeholder speckle texture:
        // template matching between cameras can't always tell identical-looking
        // dots apart, unlike a real ball's locally distinctive seams/panels — see
        // MockCameraManager's _speckleOffsets comment). Reject rather than feed a
        // physically-impossible spin into BallController's Magnus force — one bad
        // ~5000rpm reading is enough to send the ball into a visible orbit.
        //
        // 1500 (the first value tried here) turned out too loose: readings around
        // 1100–1474rpm with a flipping (+Y then -Y then +Y) axis got through as
        // "plausible" and still visibly perturbed the trajectory mid-flight — every
        // accepted reading is applied as a real Magnus force via BallController, so
        // even a same-cycle sign flip on a merely-large (not absurd) reading reads
        // as a sudden kink, not smooth orbiting. Soccer's real documented range
        // here is ~300–600rpm; capping close to that leaves headroom for genuine
        // variation without letting noise this size through.
        private const float MaxPlausibleRpm = 700f;

        private readonly FeaturePointTracker _tracker = new();
        private readonly Triangulator _triangulator;

        private Dictionary<int, Vec3>? _prevById;
        private long _prevTimestampUs;

        public Spin3DEstimator(Triangulator triangulator) => _triangulator = triangulator;

        /// <summary>
        /// Call once per stereo frame pair where both cameras detected the ball.
        /// leftFrame/rightFrame must already be rectified (see
        /// App/SimulatorEngine.cs RectifyFrame — the rectifier's undistort+
        /// rectify maps are what make row-constrained stereo matching valid).
        ///
        /// expectedDisparityPx should come from the ball's own just-triangulated
        /// position this same frame (TriangulatedPoint.Disparity from the primary
        /// Triangulator.TriangulateStereo call in SimulatorEngine.ProcessLoop) —
        /// feature points sit near the ball's surface, so their disparity is close
        /// to the ball center's. See FeaturePointTracker.FindStereoMatch for why
        /// this matters (this rig's disparity varies ~170–870px across a shot's
        /// depth range; a fixed search window can't cover that efficiently).
        /// </summary>
        public Spin3DMeasurement Update(Mat leftFrame, Mat rightFrame, PointF ballCenterLeft,
                                         float ballRadiusPx, float expectedDisparityPx, long timestampUs)
        {
            var tracked = _tracker.Update(leftFrame, ballCenterLeft, ballRadiusPx);

            var currById = new Dictionary<int, Vec3>();
            foreach (var pt in tracked)
            {
                var rightMatch = _tracker.FindStereoMatch(leftFrame, pt.Left, rightFrame, expectedDisparityPx);
                if (rightMatch == null) continue;

                var tri = _triangulator.TriangulateStereo(pt.Left, rightMatch.Value);
                if (tri.Tier == TriangulationTier.Monocular) continue; // stereo match rejected internally too

                currById[pt.Id] = new Vec3(tri.X, tri.Y, tri.Z);
            }

            var result = (_prevById == null)
                ? new Spin3DMeasurement { Valid = false }
                : Estimate(_prevById, currById, (timestampUs - _prevTimestampUs) / 1_000_000f);

            _prevById = currById;
            _prevTimestampUs = timestampUs;
            return result;
        }

        public void Reset()
        {
            _tracker.Reset();
            _prevById = null;
        }

        private static Spin3DMeasurement Estimate(Dictionary<int, Vec3> prev, Dictionary<int, Vec3> curr, float dt)
        {
            if (dt <= 0f || dt > MaxDtSeconds) return new Spin3DMeasurement { Valid = false };

            var setA = new List<Vec3>();
            var setB = new List<Vec3>();
            foreach (var kvp in prev)
            {
                if (curr.TryGetValue(kvp.Key, out var pCurr))
                {
                    setA.Add(kvp.Value);
                    setB.Add(pCurr);
                }
            }

            if (setA.Count < MinMatchedPoints) return new Spin3DMeasurement { Valid = false };

            var fit = RotationFitter.Fit(setA.ToArray(), setB.ToArray());
            if (!fit.Valid) return new Spin3DMeasurement { Valid = false };

            float rpm = fit.AngleDeg / dt / 6f; // deg/s ÷ 6 = rpm, same formula as SpinEstimator
            if (System.MathF.Abs(rpm) > MaxPlausibleRpm) return new Spin3DMeasurement { Valid = false };

            return new Spin3DMeasurement
            {
                Valid      = true,
                Rpm        = rpm,
                AxisUnit   = fit.AxisUnit,
                PointsUsed = setA.Count
            };
        }
    }
}
