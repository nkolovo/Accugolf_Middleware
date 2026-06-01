// ------------------------------------------------------------
// Vision/CameraFrame.cs
// ------------------------------------------------------------
// Moved out of CameraManager.cs so it is available on all
// target frameworks — CameraManager.cs is excluded on net10.0
// (no Spinnaker SDK), but ICameraManager, BallDetector, and
// MockCameraManager all reference CameraFrame.
// ------------------------------------------------------------
using System;

namespace SportSimulator.Vision
{
    public class CameraFrame
    {
        public int CameraIndex { get; set; }
        public byte[] Data { get; set; } = Array.Empty<byte>();
        public int Width { get; set; }
        public int Height { get; set; }
        public long TimestampUs { get; set; }
        // Chunk data extras (populated from Spinnaker chunk metadata)
        public double ExposureTimeUs { get; set; }
        public double GainDb { get; set; }
    }
}
