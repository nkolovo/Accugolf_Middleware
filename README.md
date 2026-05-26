# accugolfmiddleware
Apps that read, detect, model, and send ball/swing info to unity games

PROJECT STRUCTURE

SportSimulator/
  App/
    SimulatorEngine.cs
    Program.cs
  Models/
    BallData.cs
    SportProfile.cs
    ProfileSelectCommand.cs
  Profiles/
    Sports/ (Soccer.json, Hockey.json, Tennis.json, Baseball.json)
    SportProfileRegistry.cs
  Tracking/
    KalmanBallTracker.cs
  Transport/
    UdpTransport.cs
    PacketSerializer.cs
  Vision/
    Calibration/
      StereoCalibrationData.cs
      StereoCalibrator.cs
       StereoRectifier.cs
     BallDetector.cs
     CameraManager.cs
     Triangulator.cs