// ------------------------------------------------------------
// Tests/SportProfileRegistryJsonTests.cs
// ------------------------------------------------------------
// Tests the JSON file-loading path of SportProfileRegistry.
// Each test creates a temp directory, writes JSON profile files,
// constructs the registry pointing at that dir, then cleans up.
// ------------------------------------------------------------
using FluentAssertions;
using SportSimulator.Models;
using SportSimulator.Profiles;
using System;
using System.IO;
using System.Text.Json;
using Xunit;

namespace SportSimulator.Tests
{
    public class SportProfileRegistryJsonTests : IDisposable
    {
        private readonly string _tempDir;

        public SportProfileRegistryJsonTests()
        {
            _tempDir = Path.Combine(Path.GetTempPath(), $"profiles_{Guid.NewGuid()}");
            Directory.CreateDirectory(_tempDir);
        }

        public void Dispose()
        {
            if (Directory.Exists(_tempDir))
                Directory.Delete(_tempDir, recursive: true);
        }

        private void WriteProfile(SportProfile p)
        {
            var json = JsonSerializer.Serialize(p, new JsonSerializerOptions { WriteIndented = true });
            File.WriteAllText(Path.Combine(_tempDir, $"{p.SportId}.json"), json);
        }

        // ── Loading a custom profile ─────────────────────────────────────────────

        [Fact]
        public void CustomProfile_LoadedFromJsonFile()
        {
            WriteProfile(new SportProfile
            {
                SportId     = "cricket",
                DisplayName = "Cricket",
                DiameterMm  = 71.3f,
                MassGrams   = 156f,
                IsSphere    = true,
                MinContourArea = 60, MaxContourArea = 700,
                ProcessNoise = 0.006f, MeasurementNoise = 0.08f,
                KalmanCoastFrames = 4
            });

            var reg = new SportProfileRegistry(_tempDir);
            var p   = reg.Get("cricket");

            p.Should().NotBeNull("custom profile should be loaded from the JSON file");
            p!.DisplayName.Should().Be("Cricket");
            p.DiameterMm.Should().BeApproximately(71.3f, 0.01f);
        }

        [Fact]
        public void CustomProfile_OverridesBuiltin()
        {
            // Writing a JSON file for "golf" should override the builtin golf profile.
            WriteProfile(new SportProfile
            {
                SportId    = "golf",
                DiameterMm = 999f,  // obviously wrong — just proves override worked
                MinContourArea = 1, MaxContourArea = 9999
            });

            var reg = new SportProfileRegistry(_tempDir);
            reg.Get("golf")!.DiameterMm.Should().BeApproximately(999f, 0.01f,
                "JSON file should override the builtin profile");
        }

        [Fact]
        public void MultipleJsonFiles_AllLoaded()
        {
            WriteProfile(new SportProfile { SportId = "squash",   DiameterMm = 44f });
            WriteProfile(new SportProfile { SportId = "lacrosse", DiameterMm = 64f });

            var reg = new SportProfileRegistry(_tempDir);

            reg.Get("squash").Should().NotBeNull();
            reg.Get("lacrosse").Should().NotBeNull();
        }

        // ── Builtin profiles still present ───────────────────────────────────────

        [Fact]
        public void BuiltinProfiles_StillPresentWhenDirHasCustomProfiles()
        {
            WriteProfile(new SportProfile { SportId = "cricket", DiameterMm = 71f });
            var reg = new SportProfileRegistry(_tempDir);

            // Builtins not overridden by the custom file should still be present
            reg.Get("soccer").Should().NotBeNull("soccer builtin should survive custom load");
            reg.Get("tennis").Should().NotBeNull("tennis builtin should survive custom load");
        }

        // ── Malformed / empty JSON ────────────────────────────────────────────────

        [Fact]
        public void MalformedJsonFile_IsSkipped_BuiltinsStillLoad()
        {
            // Invalid JSON should be silently skipped; builtins must still be present.
            // LoadFromFile wraps deserialization in a try/catch for exactly this case.
            File.WriteAllText(Path.Combine(_tempDir, "broken.json"), "{ not valid json }}}");

            var act = () => new SportProfileRegistry(_tempDir);
            act.Should().NotThrow("malformed JSON file should be silently skipped");

            var reg = new SportProfileRegistry(_tempDir);
            reg.Get("golf").Should().NotBeNull("builtins should be intact despite bad JSON file");
        }

        [Fact]
        public void EmptyJsonFile_IsSkipped_OrReturnsNull()
        {
            File.WriteAllText(Path.Combine(_tempDir, "empty.json"), "");

            // Empty file: JsonSerializer.Deserialize returns null, which is guarded by
            // `if (p != null) Register(p)` in LoadFromFile — should not throw.
            var act = () => new SportProfileRegistry(_tempDir);
            act.Should().NotThrow("empty JSON file should be silently skipped");
        }

        // ── Non-existent directory ────────────────────────────────────────────────

        [Fact]
        public void NonExistentDirectory_BuiltinsStillLoad()
        {
            // Standard test-suite practice (used by existing SportProfileRegistryTests)
            var reg = new SportProfileRegistry("totally_nonexistent_dir_xyz");
            reg.Get("soccer").Should().NotBeNull();
        }
    }
}
