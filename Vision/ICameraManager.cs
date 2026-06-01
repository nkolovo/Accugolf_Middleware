// ------------------------------------------------------------
// Vision/ICameraManager.cs
// ------------------------------------------------------------
// Abstraction over the real Spinnaker-backed CameraManager.
// MockCameraManager implements this for unit tests on machines
// that don't have the Spinnaker SDK installed.
// ------------------------------------------------------------
using System;
using System.Collections.Concurrent;

namespace SportSimulator.Vision
{
    public interface ICameraManager : IDisposable
    {
        BlockingCollection<CameraFrame> FrameQueue { get; }
        void Initialize(string activeSportId = "generic");
        void ApplyProfile(string sportId);
        void StartCapture();
        void Stop();
    }
}
