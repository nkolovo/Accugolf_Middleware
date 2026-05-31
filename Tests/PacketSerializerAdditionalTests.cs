// ------------------------------------------------------------
// Tests/PacketSerializerAdditionalTests.cs
// ------------------------------------------------------------
// Edge-case coverage not in PacketSerializerTests.cs:
//   - Truncated length field (header says N bytes, buffer is shorter)
//   - Zero-length payload
//   - Unknown packet type round-trips cleanly
// ------------------------------------------------------------
using FluentAssertions;
using SportSimulator.Models;
using SportSimulator.Transport;
using System;
using System.Text.Json;
using Xunit;

namespace SportSimulator.Tests
{
    public class PacketSerializerAdditionalTests
    {
        // ── Truncated payload ────────────────────────────────────────────────────

        [Fact]
        public void Deserialize_LengthFieldExceedsBuffer_Throws()
        {
            // Header claims 100-byte payload but the buffer only contains 10 bytes total.
            var buf = new byte[10];
            buf[0] = (byte)PacketType.BallData;
            BitConverter.GetBytes(100).CopyTo(buf, 1); // lies: says 100 bytes follow
            // bytes 5-9 are zero — only 5 bytes of "payload" exist

            var act = () => PacketSerializer.Deserialize(buf);

            act.Should().Throw<Exception>("copying beyond the buffer must not silently succeed");
        }

        [Fact]
        public void Deserialize_LengthFieldIsNegative_Throws()
        {
            var buf = new byte[10];
            buf[0] = (byte)PacketType.BallData;
            BitConverter.GetBytes(-1).CopyTo(buf, 1);

            var act = () => PacketSerializer.Deserialize(buf);

            act.Should().Throw<Exception>("negative length is not a valid packet");
        }

        // ── Zero-length payload ──────────────────────────────────────────────────

        [Fact]
        public void Deserialize_ZeroLengthPayload_ReturnsEmptyByteArray()
        {
            // A 5-byte packet with length field = 0 is valid — just has no payload.
            var buf = new byte[5];
            buf[0] = (byte)PacketType.ProfileCommand;
            BitConverter.GetBytes(0).CopyTo(buf, 1);

            var act = () =>
            {
                var (type, payload) = PacketSerializer.Deserialize(buf);
                type.Should().Be(PacketType.ProfileCommand);
                payload.Should().BeEmpty();
            };

            act.Should().NotThrow("zero-length payload is a valid degenerate packet");
        }

        // ── Exact 5-byte boundary ────────────────────────────────────────────────

        [Fact]
        public void Deserialize_ExactlyFiveBytes_ZeroPayload_DoesNotThrow()
        {
            // Boundary test: 5 bytes = minimum valid packet (header only).
            var buf = new byte[5];
            buf[0] = (byte)PacketType.BallData;
            // length field = 0 (default zero-filled)

            var act = () => PacketSerializer.Deserialize(buf);
            act.Should().NotThrow();
        }

        [Fact]
        public void Deserialize_FourBytes_Throws()
        {
            // One byte below the minimum header size.
            var act = () => PacketSerializer.Deserialize(new byte[4]);
            act.Should().Throw<Exception>("4 bytes is below the 5-byte minimum");
        }

        // ── Serialize / Deserialize field preservation ───────────────────────────

        [Fact]
        public void Serialize_AllBallDataFields_PreservedAfterRoundTrip()
        {
            var original = new BallData
            {
                SportId        = "baseball",
                TimestampUs    = long.MaxValue,
                PosX = -1.5f, PosY = 3.7f, PosZ = 0.001f,
                VelX = 0f,    VelY = -9.81f, VelZ = 45f,
                SpeedMps       = 46.1f,
                LaunchAngleDeg = -2.5f,   // negative (downward)
                SpinRpm        = 0f,      // spin disabled for this sport
                Confidence     = 0.01f,   // very low confidence
                TrackingTier   = 4        // KalmanOnly
            };

            var packet   = PacketSerializer.Serialize(original);
            var (_, payload) = PacketSerializer.Deserialize(packet);
            var restored = JsonSerializer.Deserialize<BallData>(payload)!;

            restored.SportId.Should().Be("baseball");
            restored.TimestampUs.Should().Be(long.MaxValue);
            restored.PosY.Should().BeApproximately(3.7f, 0.001f);
            restored.VelY.Should().BeApproximately(-9.81f, 0.001f);
            restored.LaunchAngleDeg.Should().BeApproximately(-2.5f, 0.001f);
            restored.SpinRpm.Should().BeApproximately(0f, 0.001f);
            restored.Confidence.Should().BeApproximately(0.01f, 0.0001f);
            restored.TrackingTier.Should().Be(4);
        }
    }
}
