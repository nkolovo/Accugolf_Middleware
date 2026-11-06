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

        public static void Run(string outputPath, bool preview = false)
        {
            Console.WriteLine("=== Stereo Calibration ===");
            if (preview)
            {
                Console.WriteLine("Preview mode: each capture also saves left_preview.png /");
                Console.WriteLine("right_preview.png (overwritten each time) — green grid overlay");
                Console.WriteLine("if corners were found, plain frame if not. Use these to check");
                Console.WriteLine("focus/sharpness and framing directly, without opening SpinView.\n");
            }
            Console.WriteLine("Checkerboard: markhedleyjones.com's Checkerboard-A3-40mm-9x6.pdf");
            Console.WriteLine("(10x7 squares, giving 9x6 inner corners), printed on Letter paper");
            Console.WriteLine("fit-to-printable-area — measured square size 25.6mm (2.56cm), not");
            Console.WriteLine("the original 40mm, since Letter is smaller than the source A3 page.");
            Console.WriteLine("If you print a NEW/different copy, re-measure and update squareMm");
            Console.WriteLine("below — printer scaling shifts the real size, and a wrong value");
            Console.WriteLine("silently scales the whole calibration with no visible error.");
            Console.WriteLine("Hold it where BOTH cameras can see it — vary position, angle,");
            Console.WriteLine("and distance across captures for a good calibration.");
            Console.WriteLine("Keep the board RIGID and FLAT (taped to cardboard/a clipboard) —");
            Console.WriteLine("never bend it. The math assumes a perfectly flat grid; a bent board");
            Console.WriteLine("gets 'explained' as fake lens distortion and ruins the calibration.");
            Console.WriteLine("Tilting the whole flat board (30-45°) and holding it closer to the");
            Console.WriteLine("cameras (so it fills more of the frame) both help a lot.");
            Console.WriteLine($"Need at least {MinFramePairs} accepted pairs; more is better.");
            Console.WriteLine($"Every accepted pair is saved to {PointsPathFor(outputPath)} as you go —");
            Console.WriteLine("if the calibration math ever changes, re-run it from that file with");
            Console.WriteLine("--calibrate-from instead of recapturing.");
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
                if (accepted)
                    calibrator.SavePoints(PointsPathFor(outputPath), width, height);

                if (preview)
                {
                    calibrator.SavePreview(left.Data, right.Data, width, height,
                        "left_preview.png", "right_preview.png");
                    Console.WriteLine("  Saved left_preview.png / right_preview.png.");
                }
            }

            cameras.Stop();
            cameras.Dispose();

            if (calibrator.FramePairsCollected < MinFramePairs)
            {
                Console.WriteLine($"\nOnly {calibrator.FramePairsCollected} pair(s) collected — " +
                                  $"StereoCalibrator needs at least {MinFramePairs}. Not calibrating.");
                return;
            }

            CalibrateAndSave(calibrator, width, height, outputPath);
        }

        // Re-runs the calibration math on previously captured corners — no cameras,
        // no checkerboard, no recapturing.
        //   dotnet run -f net48 -- --calibrate-from stereo_calibration_points.json [output-path] [--focal N]
        public static void RunFromPoints(string pointsPath, string outputPath, double? focalPx = null)
        {
            Console.WriteLine($"=== Stereo Calibration (offline, from {pointsPath}) ===");
            var calibrator = StereoCalibrator.LoadPoints(pointsPath, out int width, out int height);
            Console.WriteLine($"Loaded {calibrator.FramePairsCollected} pairs ({width}x{height}).");

            if (calibrator.FramePairsCollected < MinFramePairs)
            {
                Console.WriteLine($"Need at least {MinFramePairs} pairs. Not calibrating.");
                return;
            }

            CalibrateAndSave(calibrator, width, height, outputPath, focalPx);
        }

        public static string PointsPathFor(string outputPath) =>
            System.IO.Path.ChangeExtension(outputPath, null) + "_points.json";

        private static void CalibrateAndSave(StereoCalibrator calibrator, int width, int height, string outputPath,
                                             double? focalPx = null)
        {
            Console.WriteLine("\nRunning calibration...");
            double rms = calibrator.Calibrate(width, height, out var result, focalPx);
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

            bool sane = PrintSanityReport(result);

            result.SaveToFile(outputPath);
            Console.WriteLine($"Saved to {outputPath}");
            Console.WriteLine($"Focal length was fixed at {result.K0[0]:F0}px, not solved: the checkerboard can't determine it. " +
                              "To change it: --calibrate-from <points> --focal N. A resting ball reads Y about 10cm above " +
                              "its radius either way, because detection misses its dim side.");
            if (!sane)
                Console.WriteLine("⚠ One or more sanity checks failed above — don't trust this calibration " +
                                  "for a ball test until they're resolved.");
        }

        // RMS alone isn't enough: a run reported 0.54px RMS while producing k3=2660
        // and a 53205px rectified focal length — garbage that only surfaced as zero
        // ball detections much later. These checks compare the result against
        // facts about this rig that the calibration never saw.
        private static bool PrintSanityReport(StereoCalibrationData r)
        {
            bool ok = true;
            void Check(bool pass, string msg)
            {
                Console.WriteLine($"  [{(pass ? "OK  " : "FAIL")}] {msg}");
                ok &= pass;
            }

            double f0 = r.K0[0], pf = r.P0[0];
            double baselineM = Math.Abs(r.P1[3] / r.P1[0]);
            double measuredM = StereoCalibrationData.DefaultBaselineM;
            double tLength = Math.Sqrt(r.T[0] * r.T[0] + r.T[1] * r.T[1] + r.T[2] * r.T[2]);

            Console.WriteLine("\nSanity checks:");
            Check(Math.Abs(r.T[2]) / tLength < 0.05,
                $"Cameras side by side, as mounted: T=({r.T[0] * 1000:F1}, {r.T[1] * 1000:F1}, {r.T[2] * 1000:F1})mm");
            Check(Math.Abs(baselineM - measuredM) / measuredM < 0.03,
                $"Baseline {baselineM * 1000:F1}mm vs tape-measured {measuredM * 1000:F1}mm (within 3%)");
            Check(Math.Abs(pf - f0) / f0 < 0.25,
                $"Rectified focal length {pf:F0}px close to calibrated {f0:F0}px (rectification didn't blow up)");
            Check(Math.Abs(r.D0[0]) < 1.5 && Math.Abs(r.D0[1]) < 5 && Math.Abs(r.D1[0]) < 1.5 && Math.Abs(r.D1[1]) < 5,
                $"Distortion plausible: cam0 k1={r.D0[0]:F3} k2={r.D0[1]:F3}, cam1 k1={r.D1[0]:F3} k2={r.D1[1]:F3} " +
                "(huge values usually mean a bent/non-flat board)");
            return ok;
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
