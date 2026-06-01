// ------------------------------------------------------------
// Transport/PacketSerializer.cs
// ------------------------------------------------------------
using System;
using System.Text.Json;
using SportSimulator.Models;

namespace SportSimulator.Transport
{
    public enum PacketType : byte { BallData = 0x01, ProfileCommand = 0x10 }

    public static class PacketSerializer
    {
        // Wire format: [1 byte type][4 bytes length][N bytes JSON]
        public static byte[] Serialize(BallData data)
        {
            var json = JsonSerializer.SerializeToUtf8Bytes(data);
            return BuildPacket(PacketType.BallData, json);
        }

        public static (PacketType type, byte[] payload) Deserialize(byte[] raw)
        {
            if (raw.Length < 5) throw new Exception("Packet too short");
            var type = (PacketType)raw[0];
            int len = BitConverter.ToInt32(raw, 1);
            var payload = new byte[len];
            Array.Copy(raw, 5, payload, 0, len);
            return (type, payload);
        }

        public static ProfileSelectCommand? ParseProfileCommand(byte[] payload) =>
            JsonSerializer.Deserialize<ProfileSelectCommand>(payload);

        private static byte[] BuildPacket(PacketType type, byte[] payload)
        {
            var buf = new byte[5 + payload.Length];
            buf[0] = (byte)type;
            BitConverter.GetBytes(payload.Length).CopyTo(buf, 1);
            payload.CopyTo(buf, 5);
            return buf;
        }
    }
}