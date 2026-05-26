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
    public class CameraFrame
    {
        public int CameraIndex { get; set; }
        public byte[] Data { get; set; } = Array.Empty<byte>();
        public int Width { get; set; }
        public int Height { get; set; }
        public long TimestampUs { get; set; }
        // Chunk data extras (enabled below)
        public double ExposureTimeUs { get; set; }
        public double GainDb { get; set; }
    }

    public class CameraManager : IDisposable
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

        public void Initialize(string activeSportId = "generic")
        {
            _system = new ManagedSystem();

            var logger = new SpinnakerLogHandler();
            _system.RegisterLoggingEvent(logger);
            _system.SetLoggingEventPriorityLevel(SpinnakerNET.LoggingLevel.Warning);

            var camList = _system.GetCameras();
            if (camList.Count == 0) throw new Exception("No Spinnaker cameras detected.");

            // Enumerate using foreach (IList<IManagedCamera>) per guide
            int idx = 0;
            foreach (ManagedCamera cam in camList)
            {
                try
                {
                    cam.Init();
                    ConfigureCamera(cam, activeSportId);
                    SetStreamBuffers(cam, count: 20); // guide default is 10; raise for burst
                    EnableChunkData(cam);
                    cam.AcquisitionMode.Value = AcquisitionModeEnums.Continuous.ToString();
                    cam.BeginAcquisition();
                    _cameras.Add(cam);
                    Console.WriteLine($"[CameraManager] Camera {idx} initialised.");
                    idx++;
                }
                catch (SpinnakerException ex)
                {
                    Console.WriteLine($"[CameraManager] Camera {idx} init failed: {ex.Message}");
                }
            }
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
            // ⚠️ HARDWARE TODO — confirm PixelFormatEnums.Mono8 is supported by
            // your FLIR model. Check SpinView → Format → Pixel Format.
            // Mono8 is preferred for speed (1 byte/px vs 3 for colour).
            // If your simulator uses colour cues for ball detection (e.g. yellow
            // tennis ball), switch to BayerRG8 and update BallDetector accordingly.
            try { cam.PixelFormat.Value = PixelFormatEnums.Mono8.ToString(); }
            catch (SpinnakerException) { /* camera may not support Mono8 */ }
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
                INodeMap sMap   = cam.GetStreamNodeMap();
                IInteger bufNode = sMap.GetNode<IInteger>("StreamDefaultBufferCount");
                bufNode.Value   = count;
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
                        Data           = raw.GetData(),
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
        public override void OnLogEvent(ManagedLoggingEvent e) =>
            Console.WriteLine($"[Spinnaker/{e.Priority}] {e.Message}");
    }
}