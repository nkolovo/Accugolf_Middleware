// ------------------------------------------------------------
// App/CalibrationRunner.cs
// ------------------------------------------------------------
// Interactive checkerboard-based stereo calibration tool. Connects to the
// REAL cameras (the same CameraManager the main simulator uses — net48
// only, needs the Spinnaker SDK and real hardware), lets the user position
// a printed checkerboard and capture frame pairs on demand, then runs
// StereoCalibrator.Calibrate() once enough pairs are collected and writes
// the result to a calibration JSON file.
//
// This is entirely separate from — and doesn't touch — whatever calibration
// AccuGolf's own golf software uses. Stereo calibration lives in a plain
// JSON file this middleware reads at startup (StereoCalibrationData.
// LoadFromFile), not in any camera-side persisted setting; nothing here
// changes the camera's own configuration beyond the current session, the
// same way running SpinView doesn't affect anything else that later
// reconnects to the camera.
//
// Run with: dotnet run -f net48 -- --calibrate [output-path]
// (defaults to stereo_calibration.json, matching SimulatorEngine.CalibPath)
// ------------------------------------------------------------
using System;
using System.Threading;
using SportSimulator.Vision;
using SportSimulator.Vision.Calibration;

namespace SportSimulator.App
{
    public static class CalibrationRunner
    {
        // StereoCalibrator.Calibrate() itself requires >= 15; this is just the
        // point at which we START prompting the user that they COULD stop —
        // more pairs, spread across varied positions/angles/distances, give a
        // better (lower RMS) result than exactly the minimum.
        private const int MinFramePairs = 15;

        public static void Run(string outputPath)
        {
            Console.WriteLine("=== Stereo Calibration ===");
            Console.WriteLine("Checkerboard: markhedleyjones.com's Checkerboard-A3-40mm-9x6.pdf");
            Console.WriteLine("(10x7 squares, giving 9x6 inner corners), printed on Letter paper");
            Console.WriteLine("fit-to-printable-area — measured square size 25.6mm (2.56cm), not");
            Console.WriteLine("the original 40mm, since Letter is smaller than the source A3 page.");
            Console.WriteLine("If you print a NEW/different copy, re-measure and update squareMm");
            Console.WriteLine("below — printer scaling shifts the real size, and a wrong value");
            Console.WriteLine("silently scales the whole calibration with no visible error.");
            Console.WriteLine("Hold it where BOTH cameras can see it — vary position, angle,");
            Console.WriteLine("and distance across captures for a good calibration.");
            Console.WriteLine($"Need at least {MinFramePairs} accepted pairs; more is better.");
            Console.WriteLine("Press ENTER to capture a pair, or 'q' + ENTER to finish early.\n");

            var cameras = new CameraManager();
            // 9x6 inner corners, 25.6mm squares -- measured directly off this
            // specific printed copy (Letter, fit-to-printable-area from the
            // source A3/40mm PDF; see the printed instructions above). Re-measure
            // and update this if a new copy is printed differently.
            var calibrator = new StereoCalibrator(cornersX: 9, cornersY: 6, squareMm: 25.6f);

            cameras.Initialize("generic");
            cameras.StartCapture();

            // Let both capture threads get a first frame flowing before the
            // very first capture request, so it doesn't fail on an empty queue
            // purely from startup timing.
            Thread.Sleep(500);

            int width = 0, height = 0;

            while (true)
            {
                Console.Write($"[{calibrator.FramePairsCollected}/{MinFramePairs}+] Press ENTER to capture, 'q' to finish: ");
                string? line = Console.ReadLine();
                if (line != null && line.Trim().Equals("q", StringComparison.OrdinalIgnoreCase))
                    break;

                if (!TryGetLatestPair(cameras, out var left, out var right))
                {
                    Console.WriteLine("  Couldn't get a frame from both cameras — try again.");
                    continue;
                }

                width = left.Width;
                height = left.Height;

                bool accepted = calibrator.AddFramePair(left.Data, right.Data, width, height);
                Console.WriteLine(accepted
                    ? $"  Checkerboard found in both cameras — pair {calibrator.FramePairsCollected} accepted."
                    : "  Checkerboard not found in one or both cameras — reposition and try again.");
            }

            cameras.Stop();
            cameras.Dispose();

            if (calibrator.FramePairsCollected < MinFramePairs)
            {
                Console.WriteLine($"\nOnly {calibrator.FramePairsCollected} pair(s) collected — " +
                                  $"StereoCalibrator needs at least {MinFramePairs}. Not calibrating.");
                return;
            }

            Console.WriteLine("\nRunning calibration...");
            double rms = calibrator.Calibrate(width, height, out var result);
            Console.WriteLine($"RMS reprojection error: {rms:F4}px " +
                               (rms < 1.0 ? "(good)" : "(high — consider recapturing with more/varied pairs)"));

            // Checkerboard calibration determines the cameras' own intrinsics
            // and their pose relative to EACH OTHER — it has no way to know how
            // the pair as a WHOLE is mounted relative to the world (height,
            // forward offset). That's a separate, rig-installation fact that
            // doesn't change just because the lenses got recalibrated. Carry
            // forward the confirmed on-site values rather than leaving these at
            // the class defaults (0,0) — which would silently disable
            // Triangulator's world-frame correction (see its class comment).
            result.CameraHeightM        = StereoCalibrationData.DefaultCameraHeightM;
            result.CameraForwardOffsetM = StereoCalibrationData.DefaultCameraForwardOffsetM;

            result.SaveToFile(outputPath);
            Console.WriteLine($"Saved to {outputPath}");
        }

        // Pulls whatever's already queued for each camera (an immediate,
        // non-blocking drain — a snapshot of everything that arrived since the
        // last capture), keeping only the LATEST frame per camera so a capture
        // reflects where the board is right now, not several frames back if
        // this loop happened to fall behind the camera's own ~200fps rate. If
        // the immediate drain didn't have both yet (e.g. right at startup),
        // falls back to waiting a bounded amount of time for whichever is
        // still missing, rather than either blocking forever or giving up too
        // early.
        private static bool TryGetLatestPair(CameraManager cameras, out CameraFrame left, out CameraFrame right)
        {
            CameraFrame? l = null, r = null;

            while (cameras.FrameQueue.TryTake(out var frame, 0))
            {
                if (frame.CameraIndex == 0) l = frame; else r = frame;
            }

            var deadline = DateTime.UtcNow.AddSeconds(2);
            while ((l == null || r == null) && DateTime.UtcNow < deadline)
            {
                if (!cameras.FrameQueue.TryTake(out var frame, 200)) continue;
                if (frame.CameraIndex == 0) l = frame; else r = frame;
            }

            left = l!; right = r!;
            return l != null && r != null;
        }
    }
}
