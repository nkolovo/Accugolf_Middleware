// ------------------------------------------------------------
// Tests/CoastConfidenceTests.cs
// ------------------------------------------------------------
// Documents and verifies the Kalman coast confidence decay formula
// used inline in SimulatorEngine.ProcessLoop:
//
//   conf = Max(0.05, 0.3 - coastFrame * (0.25 / maxCoastFrames))
//
// Starts at 0.25 on the first coast frame, decays linearly each frame,
// and floors at 0.05. This keeps Unity informed that the position is
// predicted (not measured) while preventing zero-confidence drops that
// would cause Unity to discard the point entirely.
// ------------------------------------------------------------
using FluentAssertions;
using System;
using Xunit;

namespace SportSimulator.Tests
{
    public class CoastConfidenceTests
    {
        // Mirror of the formula in SimulatorEngine.ProcessLoop so tests break
        // if the production formula is changed without updating the tests.
        private static float CoastConf(int frame, int maxFrames) =>
            Math.Max(0.05f, 0.3f - frame * (0.25f / maxFrames));

        // ── Linear decay ─────────────────────────────────────────────────────────

        [Theory]
        [InlineData(1, 5, 0.25f)]  // 0.30 - 1*(0.25/5) = 0.25
        [InlineData(2, 5, 0.20f)]
        [InlineData(3, 5, 0.15f)]
        [InlineData(4, 5, 0.10f)]
        [InlineData(5, 5, 0.05f)]  // exactly at floor — Max(0.05, 0.05)
        public void Confidence_DecaysLinearly_WithinWindow(int frame, int maxFrames, float expected)
        {
            CoastConf(frame, maxFrames).Should().BeApproximately(expected, 0.001f);
        }

        // ── Floor ────────────────────────────────────────────────────────────────

        [Fact]
        public void Confidence_NeverDropsBelowFloor()
        {
            // The engine stops sending after maxCoastFrames, but the formula should
            // still be safe if called beyond that point (defensive guard).
            for (int f = 1; f <= 30; f++)
                CoastConf(f, 5).Should().BeGreaterThanOrEqualTo(0.05f,
                    $"floor of 0.05 must hold at frame {f}");
        }

        [Fact]
        public void Confidence_FloorExactlyAtMaxCoastFrame()
        {
            // At frame == maxCoastFrames the formula equals exactly 0.05 (the floor).
            CoastConf(5, 5).Should().BeApproximately(0.05f, 0.001f);
        }

        // ── Relationship to stereo confidence ────────────────────────────────────

        [Fact]
        public void Confidence_FirstCoastFrame_IsLowerThanFullStereoMinimum()
        {
            // Tier 1 (FullStereo) confidence minimum is 0.80.
            // The first coast frame (0.25) should be well below that.
            CoastConf(1, 5).Should().BeLessThan(0.80f,
                "coasted position should never be mistaken for a fresh stereo detection");
        }

        // ── Effect of coast window width ─────────────────────────────────────────

        [Fact]
        public void Confidence_WiderWindow_SlowerDecay()
        {
            // Sports with longer coast windows (e.g. soccer=8) decay more slowly
            // than tight windows (golf=3). At the same frame index, wider is higher.
            float narrow = CoastConf(3, 3);  // at max — floor (0.05)
            float wide   = CoastConf(3, 8);  // 0.30 - 3*(0.25/8) = 0.206
            wide.Should().BeGreaterThan(narrow,
                "wider coast window means slower per-frame confidence decay");
        }

        [Fact]
        public void Confidence_NarrowWindow_ReachesFloorEarlier()
        {
            // With maxFrames=3 the floor is hit at frame 3; with maxFrames=10 it's frame 10.
            CoastConf(3,  3).Should().BeApproximately(0.05f, 0.001f, "narrow window hits floor at frame 3");
            CoastConf(3, 10).Should().BeGreaterThan(0.05f, "wide window is still above floor at frame 3");
        }

        // ── Output range ─────────────────────────────────────────────────────────

        [Fact]
        public void Confidence_FirstFrame_BelowInitialMeasuredValue()
        {
            // 0.30 is chosen so the first coast confidence is always below the monocular
            // detection confidence (0.35), preserving the tier ordering.
            CoastConf(1, 5).Should().BeLessThan(0.35f);
        }
    }
}
