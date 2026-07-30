// ------------------------------------------------------------
// Vision/CameraManager.cs
// ------------------------------------------------------------
// Uses Spinnaker C# QuickSpin API throughout (cam.Property.Value style)
// per the official programmer's guide. Falls back to GenAPI node map
// for stream buffer tuning which QuickSpin doesn't expose directly.
// Disposal follows: EndAcquisition → DeInit → system.Dispose()
// ------------------------------------------------------------
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using SpinnakerNET;
using SpinnakerNET.GenApi;

namespace SportSimulator.Vision
{
    // CameraFrame is defined in Vision/CameraFrame.cs

    public class CameraManager : ICameraManager
    {
        private ManagedSystem? _system;
        private readonly List<ManagedCamera> _cameras = new();
        public BlockingCollection<CameraFrame> FrameQueue { get; } = new(128);
        private bool _running;

        // Per-profile exposure map: SportId → exposure µs
        // ⚠️ HARDWARE TODO — tune these for your specific lighting environment.
        // These are starting estimates only. Use SpinView to live-preview each
        // sport setup and adjust until the ball is clearly visible without
        // blooming (overexposure) or loss of detail (underexposure).
        // General rule: shorter exposure = sharper fast-moving objects, but
        // requires brighter lighting. For indoor simulators, err shorter.
        private readonly Dictionary<string, double> _exposureMap = new()
        {
            { "soccer",   2000.0 }, // ⚠️ tune: slower ball, more exposure OK
            { "hockey",    500.0 }, // ⚠️ tune: fast puck, keep short
            { "tennis",    800.0 }, // ⚠️ tune
            { "baseball",  600.0 }, // ⚠️ tune
            { "golf",      300.0 }, // ⚠️ tune: fastest ball — shortest exposure
            { "generic",  1000.0 }
        };

        // Known serial → CameraIndex mapping for the AccuGolf Hawkeye rig.
        // Spinnaker's raw enumeration order is NOT guaranteed to match physical
        // left/right placement — confirmed on-site (2026-07-22) that it doesn't:
        // enumeration put the RIGHT camera first. That matters because CameraIndex
        // 0 = LEFT is assumed throughout this codebase's stereo math (disparity
        // formula in Triangulator.cs, baseline sign in StereoCalibrationData.cs,
        // "Camera 0 at -baseline/2" in MockCameraManager.cs). Getting this backwards
        // makes every real detection look like it's behind the camera.
        //
        // ⚠️ HARDWARE TODO — update if a camera is ever swapped/replaced.
        private static readonly Dictionary<string, int> SerialToCameraIndex = new()
        {
            { "24182871", 0 }, // left
            { "24193779", 1 }, // right
        };

        public void Initialize(string activeSportId = "generic")
        {
            _system = new ManagedSystem();

            var logger = new SpinnakerLogHandler();
            _system.RegisterLoggingEventHandler(logger);
            _system.SetLoggingEventPriorityLevel(ManagedLoggingLevel.LOG_LEVEL_WARN);

            var camList = _system.GetCameras();
            if (camList.Count == 0) throw new Exception("No Spinnaker cameras detected.");

            // Init every camera first, then place it by serial number into the
            // slot the rest of the app expects — NOT by raw enumeration order.
            var bySlot = new SortedDictionary<int, ManagedCamera>();
            int nextFallbackSlot = 0;
            foreach (ManagedCamera cam in camList)
            {
                try
                {
                    cam.Init();
                    string serial = cam.DeviceSerialNumber.Value;

                    int slot;
                    if (SerialToCameraIndex.TryGetValue(serial, out var known))
                    {
                        slot = known;
                    }
                    else
                    {
                        // Unknown serial — different/replaced hardware, or the map
                        // above is stale. Fall back to enumeration order rather than
                        // fail outright, but warn loudly: left/right may be swapped.
                        slot = nextFallbackSlot;
                        Console.WriteLine($"[CameraManager] WARNING: serial {serial} not in SerialToCameraIndex — " +
                                           $"falling back to enumeration order for CameraIndex {slot}. Update the map above.");
                    }
                    nextFallbackSlot++;

                    while (bySlot.ContainsKey(slot)) slot++; // guard against a mapping collision

                    ConfigureCamera(cam, activeSportId);
                    SetStreamBuffers(cam, count: 20); // guide default is 10; raise for burst
                    EnableChunkData(cam);
                    cam.AcquisitionMode.Value = AcquisitionModeEnums.Continuous.ToString();
                    cam.BeginAcquisition();

                    bySlot[slot] = cam;
                    Console.WriteLine($"[CameraManager] Camera serial {serial} → CameraIndex {slot}.");
                }
                catch (SpinnakerException ex)
                {
                    Console.WriteLine($"[CameraManager] Camera init failed: {ex.Message}");
                }
            }

            foreach (var kvp in bySlot) _cameras.Add(kvp.Value);
            Console.WriteLine($"[CameraManager] {_cameras.Count} camera(s) ready.");
        }

        // Apply per-sport camera settings using QuickSpin API
        public void ApplyProfile(string sportId)
        {
            double exp = _exposureMap.TryGetValue(sportId, out var e) ? e : 1000.0;
            foreach (var cam in _cameras)
            {
                try
                {
                    // QuickSpin: exposure (guide §Setting Exposure Time)
                    cam.ExposureAuto.Value  = ExposureAutoEnums.Off.ToString();
                    cam.ExposureMode.Value  = ExposureModeEnums.Timed.ToString();
                    cam.ExposureTime.Value  = exp;

                    // QuickSpin: gain (guide §Setting Gain)
                    cam.GainAuto.Value = GainAutoEnums.Off.ToString();
                    cam.Gain.Value     = 0.0;

                    // QuickSpin: black level (guide §Setting Black Level)
                    cam.BlackLevelSelector.Value = BlackLevelSelectorEnums.All.ToString();
                    cam.BlackLevel.Value         = 1.0;
                }
                catch (SpinnakerException ex)
                {
                    Console.WriteLine($"[CameraManager] ApplyProfile warning: {ex.Message}");
                }
            }
        }

        private void ConfigureCamera(ManagedCamera cam, string sportId)
        {
            // Confirmed on-site 2026-07-22: both cameras are Blackfly S
            // BFS-PGE-04S2M, monochrome sensor (the "M" suffix) — Mono8 is the
            // native format, not a color-sensor fallback. If your simulator ever
            // needs color cues (e.g. a yellow tennis ball), you'd need a different
            // camera model — this one has no Bayer color data to fall back to.
            try { cam.PixelFormat.Value = PixelFormatEnums.Mono8.ToString(); }
            catch (SpinnakerException) { /* camera may not support Mono8 */ }

            // Force free-run acquisition regardless of the camera's persisted
            // trigger config. SpinView reported TriggerSource=Software on-site,
            // which is ambiguous by itself (TriggerSource is irrelevant if
            // TriggerMode=Off, but if TriggerMode=On, GetNextImage() would block
            // forever waiting for a software trigger this app never sends — the
            // continuous free-run architecture throughout this engine needs
            // TriggerMode explicitly Off, not just assumed).
            try
            {
                cam.TriggerSelector.Value = TriggerSelectorEnums.FrameStart.ToString();
                cam.TriggerMode.Value      = TriggerModeEnums.Off.ToString();
            }
            catch (SpinnakerException ex)
            {
                Console.WriteLine($"[CameraManager] TriggerMode warning: {ex.Message}");
            }

            ApplyProfileToCamera(cam, sportId);
        }

        private void ApplyProfileToCamera(ManagedCamera cam, string sportId)
        {
            double exp = _exposureMap.TryGetValue(sportId, out var e) ? e : 1000.0;
            cam.ExposureAuto.Value  = ExposureAutoEnums.Off.ToString();
            cam.ExposureMode.Value  = ExposureModeEnums.Timed.ToString();
            cam.ExposureTime.Value  = exp;
            cam.GainAuto.Value = GainAutoEnums.Off.ToString();
            cam.Gain.Value     = 0.0;
            cam.BlackLevelSelector.Value = BlackLevelSelectorEnums.All.ToString();
            cam.BlackLevel.Value         = 1.0;
        }

        // Increase software buffer count via GenAPI stream node map (guide §Setting Number of Image Buffers)
        private void SetStreamBuffers(ManagedCamera cam, int count)
        {
            try
            {
                INodeMap sMap    = cam.GetTLStreamNodeMap();
                IInteger bufNode = sMap?.GetNode<IInteger>("StreamDefaultBufferCount");
                // GetNode<T> returns null rather than throwing when a node isn't
                // exposed on this camera/GenTL producer combination — found live
                // on the real hardware (this dev environment has no way to catch
                // this, no Spinnaker SDK installed here): bufNode.Value = count
                // threw a NullReferenceException the existing catch below doesn't
                // catch (it's not a SpinnakerException), crashing the whole app
                // over an optional buffer-count tuning step. Treat a missing node
                // the same as the catch already does for a real Spinnaker error —
                // warn and move on, since this is a performance tweak (guide
                // default is 10; raise for burst), not something acquisition
                // actually requires.
                if (bufNode == null)
                {
                    Console.WriteLine("[CameraManager] Buffer config warning: StreamDefaultBufferCount not available on this camera/interface — using its default buffer count.");
                    return;
                }
                bufNode.Value = count;
            }
            catch (SpinnakerException ex)
            {
                Console.WriteLine($"[CameraManager] Buffer config warning: {ex.Message}");
            }
        }

        // Enable ExposureTime + Gain chunk data per image (guide §Chunk Data)
        private void EnableChunkData(ManagedCamera cam)
        {
            try
            {
                cam.ChunkSelector.Value  = ChunkSelectorEnums.ExposureTime.ToString();
                cam.ChunkEnable.Value    = true;
                cam.ChunkSelector.Value  = ChunkSelectorEnums.Gain.ToString();
                cam.ChunkEnable.Value    = true;
                cam.ChunkModeActive.Value = true;
            }
            catch (SpinnakerException ex)
            {
                Console.WriteLine($"[CameraManager] Chunk data warning: {ex.Message}");
            }
        }

        public void StartCapture()
        {
            _running = true;
            for (int i = 0; i < _cameras.Count; i++)
            {
                int captureIdx = i;
                var cam = _cameras[i];
                System.Threading.Tasks.Task.Factory.StartNew(
                    () => CaptureLoop(cam, captureIdx),
                    System.Threading.Tasks.TaskCreationOptions.LongRunning);
            }
        }

        private void CaptureLoop(ManagedCamera cam, int idx)
        {
            while (_running)
            {
                try
                {
                    // GetNextImage with 500ms timeout (guide §Grabbing Images)
                    using var raw = cam.GetNextImage(500);

                    // Always check ImageStatus before using data (guide §Grab Result)
                    if (raw.ImageStatus != ImageStatus.IMAGE_NO_ERROR)
                    {
                        Console.WriteLine($"[Camera {idx}] Bad frame: {raw.ImageStatus}");
                        continue;
                    }

                    // Pull chunk data for per-frame metadata
                    double expUs = 0, gainDb = 0;
                    try
                    {
                        expUs  = raw.ChunkData.ExposureTime;
                        gainDb = raw.ChunkData.Gain;
                    }
                    catch { /* chunk data may not be available every frame */ }

                    FrameQueue.Add(new CameraFrame
                    {
                        CameraIndex    = idx,
                        Data           = raw.ManagedData,
                        Width          = (int)raw.Width,
                        Height         = (int)raw.Height,
                        TimestampUs    = (long)(raw.TimeStamp / 1000),
                        ExposureTimeUs = expUs,
                        GainDb         = gainDb
                    });
                }
                catch (SpinnakerException) { /* timeout during shutdown — expected */ }
            }
        }

        public void Stop() => _running = false;

        public void Dispose()
        {
            Stop();
            // Guide: EndAcquisition → DeInit for each camera, then Dispose system
            foreach (var c in _cameras)
            {
                try { c.EndAcquisition(); } catch { }
                try { c.DeInit();         } catch { }
            }
            _system?.Dispose();
        }
    }

    // Minimal logging handler (guide §Logging — 5 levels: Error/Warning/Notice/Info/Debug)
    internal class SpinnakerLogHandler : ManagedLoggingEventHandler
    {
        public override void OnLogEvent(ManagedLoggingEventData e) =>
            Console.WriteLine($"[Spinnaker/{e.Level}] {e.Message}");
    }
}