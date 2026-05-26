// ------------------------------------------------------------
// Profiles/SportProfileRegistry.cs
// ------------------------------------------------------------
using System.Collections.Generic;
using System.IO;
using System.Text.Json;
using SportSimulator.Models;

namespace SportSimulator.Profiles
{
    public class SportProfileRegistry
    {
        private readonly Dictionary<string, SportProfile> _profiles = new();

        public SportProfileRegistry(string profilesDir = "Profiles")
        {
            LoadBuiltins();
            if (Directory.Exists(profilesDir))
                foreach (var f in Directory.GetFiles(profilesDir, "*.json"))
                    LoadFromFile(f);
        }

        private void LoadBuiltins()
        {
            Register(new SportProfile
            {
                SportId = "soccer", DisplayName = "Soccer",
                DiameterMm = 220, MassGrams = 430, IsSphere = true,
                MinContourArea = 800, MaxContourArea = 8000,
                MinSpeedMps = 0, MaxSpeedMps = 60,
                OutputSpin = true, OutputLaunchAngle = true,
                ProcessNoise = 0.008f, MeasurementNoise = 0.09f,
                // Soccer: 10–30 m/s typical → 700mm crossed in 23–70ms → 3–8 frames.
                // +3 occlusion buffer for body/foot blocking the ball at impact.
                KalmanCoastFrames = 8
            });
            Register(new SportProfile
            {
                SportId = "hockey", DisplayName = "Ice Hockey",
                DiameterMm = 76, MassGrams = 170, IsSphere = false,
                MinContourArea = 100, MaxContourArea = 1200,
                MinSpeedMps = 0, MaxSpeedMps = 50,
                OutputSpin = false, OutputLaunchAngle = false, OutputTilt = true,
                ProcessNoise = 0.012f, MeasurementNoise = 0.12f,
                // Hockey: 20–40 m/s → 17–35ms → 2–4 frames.
                // Puck stays low so occlusion is rare; +2 buffer.
                KalmanCoastFrames = 5
            });
            Register(new SportProfile
            {
                SportId = "tennis", DisplayName = "Tennis",
                DiameterMm = 67, MassGrams = 58, IsSphere = true,
                MinContourArea = 80, MaxContourArea = 900,
                MinSpeedMps = 0, MaxSpeedMps = 80,
                OutputSpin = true, OutputLaunchAngle = true,
                ProcessNoise = 0.006f, MeasurementNoise = 0.08f,
                // Tennis: 30–60 m/s serve/groundstroke → 12–23ms → 1–3 frames.
                // Small ball, fast — over-projecting is worse than under.
                // +2 buffer only for racket-frame occlusion at contact.
                KalmanCoastFrames = 4
            });
            Register(new SportProfile
            {
                SportId = "baseball", DisplayName = "Baseball",
                DiameterMm = 74, MassGrams = 145, IsSphere = true,
                MinContourArea = 90, MaxContourArea = 1000,
                MinSpeedMps = 0, MaxSpeedMps = 50,
                OutputSpin = true, OutputLaunchAngle = true,
                ProcessNoise = 0.007f, MeasurementNoise = 0.09f,
                // Baseball: 35–50 m/s off the bat → 14–20ms → 2–3 frames.
                // Bat briefly occludes ball at contact; +2 buffer.
                KalmanCoastFrames = 4
            });
            Register(new SportProfile
            {
                SportId = "golf", DisplayName = "Golf",
                DiameterMm = 43, MassGrams = 46, IsSphere = true,
                MinContourArea = 30, MaxContourArea = 400,
                MinSpeedMps = 0, MaxSpeedMps = 90,
                OutputSpin = true, OutputLaunchAngle = true,
                ProcessNoise = 0.005f, MeasurementNoise = 0.07f,
                // Golf: 60–90 m/s driver → 700mm in 8–12ms → 1–2 frames.
                // Fastest sport here; coasting more than 3 frames risks
                // projecting the ball ~200mm past its real position.
                KalmanCoastFrames = 3
            });
        }

        private void LoadFromFile(string path)
        {
            var json = File.ReadAllText(path);
            var p = JsonSerializer.Deserialize<SportProfile>(json);
            if (p != null) Register(p);
        }

        public void Register(SportProfile p) => _profiles[p.SportId.ToLower()] = p;

        public SportProfile? Get(string sportId) =>
            _profiles.TryGetValue(sportId.ToLower(), out var p) ? p : null;

        public SportProfile GetOrDefault(string sportId) =>
            Get(sportId) ?? new SportProfile { SportId = "generic", DisplayName = "Generic",
                DiameterMm = 100, MassGrams = 100, MinContourArea = 50, MaxContourArea = 5000,
                MinSpeedMps = 0, MaxSpeedMps = 100 };
    }
}