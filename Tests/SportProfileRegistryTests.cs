// ------------------------------------------------------------
// Tests/SportProfileRegistryTests.cs
// ------------------------------------------------------------
using FluentAssertions;
using SportSimulator.Profiles;
using SportSimulator.Models;
using Xunit;

namespace SportSimulator.Tests
{
    public class SportProfileRegistryTests
    {
        // Pass a non-existent dir so no JSON files are loaded — builtins only
        private static SportProfileRegistry Registry() => new("nonexistent_dir");

        [Theory]
        [InlineData("soccer")]
        [InlineData("hockey")]
        [InlineData("tennis")]
        [InlineData("baseball")]
        [InlineData("golf")]
        public void BuiltinProfiles_ArePresent(string sportId)
        {
            var p = Registry().Get(sportId);
            p.Should().NotBeNull($"'{sportId}' should be a builtin profile");
        }

        [Theory]
        [InlineData("soccer")]
        [InlineData("SOCCER")]
        [InlineData("Soccer")]
        public void Lookup_IsCaseInsensitive(string id)
        {
            var p = Registry().Get(id);
            p.Should().NotBeNull("lookups should be case-insensitive");
            p!.SportId.Should().Be("soccer");
        }

        [Fact]
        public void GetOrDefault_UnknownSport_ReturnsGeneric()
        {
            var p = Registry().GetOrDefault("polo");
            p.SportId.Should().Be("generic");
        }

        [Fact]
        public void GolfProfile_HasSmallestBall()
        {
            var golf   = Registry().Get("golf")!;
            var soccer = Registry().Get("soccer")!;
            golf.DiameterMm.Should().BeLessThan(soccer.DiameterMm,
                "golf ball is smaller than soccer ball");
        }

        [Fact]
        public void GolfProfile_HasShortestKalmanCoastWindow()
        {
            // Golf is the fastest sport — coasting too long would project the
            // ball far past its real position.
            var reg = Registry();
            int golfCoast   = reg.Get("golf")!.KalmanCoastFrames;
            int soccerCoast = reg.Get("soccer")!.KalmanCoastFrames;
            golfCoast.Should().BeLessThan(soccerCoast,
                "golf needs the tightest coast window due to high speed");
        }

        [Fact]
        public void Register_OverridesExisting()
        {
            var reg = Registry();
            reg.Register(new SportProfile { SportId = "soccer", DisplayName = "Modified Soccer",
                DiameterMm = 999 });
            reg.Get("soccer")!.DiameterMm.Should().Be(999f);
        }

        [Fact]
        public void HockeyProfile_IsPuck_NotSphere()
        {
            var p = Registry().Get("hockey")!;
            p.IsSphere.Should().BeFalse("a hockey puck is not a sphere");
            p.OutputSpin.Should().BeFalse("puck spin is not meaningful in the same way");
            p.OutputTilt.Should().BeTrue("puck tilt angle is the relevant output");
        }
    }
}
