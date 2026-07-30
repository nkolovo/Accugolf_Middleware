// ------------------------------------------------------------
// App/Program.cs
// ------------------------------------------------------------
using System;
using SportSimulator.App;
using SportSimulator.Vision;

#if NET48
// Separate from normal operation: dotnet run -f net48 -- --calibrate [output-path]
// Needs the real cameras (Spinnaker), not the mock — see CalibrationRunner.
if (args.Length > 0 && args[0] == "--calibrate")
{
    bool preview = Array.IndexOf(args, "--preview") >= 0;
    string calibOutputPath = "stereo_calibration.json";
    for (int i = 1; i < args.Length; i++)
    {
        if (args[i] != "--preview") { calibOutputPath = args[i]; break; }
    }
    CalibrationRunner.Run(calibOutputPath, preview);
    return;
}
#endif

var unityIp   = args.Length > 0 ? args[0] : "127.0.0.1";
var sendPort  = args.Length > 1 ? int.Parse(args[1]) : 7100;
var listenPort = args.Length > 2 ? int.Parse(args[2]) : 7101;

Console.WriteLine("=== SportSimulator ===");
Console.WriteLine($"Unity target : {unityIp}:{sendPort}");
Console.WriteLine($"Listen port  : {listenPort}");
Console.WriteLine("Press Ctrl+C to exit.\n");

#if NET48
using var engine = new SimulatorEngine();          // real Spinnaker cameras
#else
Console.WriteLine("[WARNING] Running without Spinnaker SDK — MockCameraManager active.");
var mock = new MockCameraManager();

// Default (30 m/s, 15°) is a full-power drive-distance kick that flies the ball
// past Triangulator's 10m stereo range within under a second — way beyond the
// real rig's actual working distance (~3m slant to the tee, per notes/14-...md).
// A real shot only needs to be tracked in a brief burst near the tee before Unity
// takes over the flight locally; this trajectory stays within stereo range for
// longer, giving the Kalman filter a realistic number of real measurements to
// converge on before the ball exits useful tracking range.
// Negative azimuth = left side of the net (positive azimuth -> positive world X ->
// positive Unity X -> Unity's GoalkeeperAI treats positive ballX as the right
// side, confirmed via its own "isLeftSide" log output on a straight, azimuth=0
// shot). Spin stays disabled for mock testing (see SimulatorEngine's
// _skipSpinEstimation), so this is a pure, curve-free aim change.
//
// Tracking no longer depends on capturing many real frames across the ball's
// whole visible flight (that was the old, since-replaced 8-sample least-squares
// seed's requirement). KalmanBallTracker now seeds velocity from a known rest
// position (the calibrated tee, effectively zero-noise) plus a SINGLE real
// detection the instant the ball is seen displaced from it — see its class
// comment. That single detection lands within a few cm of the tee regardless of
// launch angle or azimuth, well inside the sensor's frame — so steeper launch
// angles are no longer the tracking risk they used to be under the old
// architecture (see git history/notes/14-...md for that now-obsolete analysis).
// The real remaining constraint is just Triangulator.ToWorldFrame's correctness
// (fixed this session — see its class comment) and BallDetector no longer
// absorbing a resting ball into its background model (also fixed).
//
// launchAngleDeg 17.5 -> 25: requested to test a shot with enough arc to
// possibly clear the crossbar, not just reach mid-net height. Rough physics
// check (drag-free, ignoring Unity's own gravity/drag model which will differ
// somewhat): at 18 m/s / 25 degrees / 15 degrees azimuth, height at the ~11m
// goal line comes out around 2.9m — above the ~2.39m crossbar — so this should
// plausibly go over, matching what's being tested for.

mock.SetTrajectory(speedMps: 15f, launchAngleDeg: 7f, azimuthDeg: 16f);
using var engine = new SimulatorEngine(mock);
#endif

Console.CancelKeyPress += (_, e) => { e.Cancel = true; engine.Stop(); };

engine.Start(unityIp, sendPort, listenPort);