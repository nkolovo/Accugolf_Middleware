// ------------------------------------------------------------
// Tests/PacketSerializerTests.cs
// ------------------------------------------------------------
using FluentAssertions;
using SportSimulator.Models;
using SportSimulator.Transport;
using System.Text.Json;
using Xunit;

namespace SportSimulator.Tests
{
    public class PacketSerializerTests
    {
        private static BallData MakeBallData() => new()
        {
            SportId        = "soccer",
            TimestampUs    = 1_700_000_000_000_000L,
            PosX = 1.2f, PosY = 0.5f, PosZ = 3.4f,
            VelX = 10f,  VelY = 5f,   VelZ = 20f,
            SpeedMps       = 22.9f,
            LaunchAngleDeg = 12.5f,
            SpinRpm        = 1200f,
            Confidence     = 0.91f,
            TrackingTier   = 1
        };

        [Fact]
        public void Serialize_ProducesCorrectHeader()
        {
            var packet = PacketSerializer.Serialize(MakeBallData());

            packet[0].Should().Be((byte)PacketType.BallData, "first byte is packet type");
            int declaredLen = System.BitConverter.ToInt32(packet, 1);
            declaredLen.Should().Be(packet.Length - 5, "length field should equal payload length");
        }

        [Fact]
        public void Deserialize_RoundTrips_TypeAndPayload()
        {
            var original = MakeBallData();
            var packet   = PacketSerializer.Serialize(original);
            var (type, payload) = PacketSerializer.Deserialize(packet);

            type.Should().Be(PacketType.BallData);
            var restored = JsonSerializer.Deserialize<BallData>(payload)!;
            restored.SportId.Should().Be(original.SportId);
            restored.SpeedMps.Should().BeApproximately(original.SpeedMps, 0.01f);
            restored.Confidence.Should().BeApproximately(original.Confidence, 0.001f);
            restored.TrackingTier.Should().Be(original.TrackingTier);
        }

        [Fact]
        public void Deserialize_TooShortPacket_Throws()
        {
            var act = () => PacketSerializer.Deserialize(new byte[] { 0x01, 0x00 });
            act.Should().Throw<System.Exception>("packet under 5 bytes is invalid");
        }

        [Fact]
        public void ProfileCommand_RoundTrips()
        {
            var cmd     = new ProfileSelectCommand { SportId = "baseball" };
            var json    = System.Text.Json.JsonSerializer.SerializeToUtf8Bytes(cmd);
            var buf     = new byte[5 + json.Length];
            buf[0]      = (byte)PacketType.ProfileCommand;
            System.BitConverter.GetBytes(json.Length).CopyTo(buf, 1);
            json.CopyTo(buf, 5);

            var (type, payload) = PacketSerializer.Deserialize(buf);
            type.Should().Be(PacketType.ProfileCommand);

            var restored = PacketSerializer.ParseProfileCommand(payload)!;
            restored.SportId.Should().Be("baseball");
        }
    }
}
