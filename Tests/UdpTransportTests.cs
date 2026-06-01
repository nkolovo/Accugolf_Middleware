// ------------------------------------------------------------
// Tests/UdpTransportTests.cs
// ------------------------------------------------------------
// All tests use loopback (127.0.0.1) so no network config is needed.
// Ports are chosen in the 50000+ ephemeral range and each test uses
// a unique pair so parallel test runs don't collide.
// ------------------------------------------------------------
using FluentAssertions;
using SportSimulator.Models;
using SportSimulator.Transport;
using System;
using System.Net;
using System.Net.Sockets;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
using Xunit;

namespace SportSimulator.Tests
{
    public class UdpTransportTests : IDisposable
    {
        // Each test allocates its own port pair from this base to avoid collisions.
        // Tests run in parallel within the class; offsets keep them isolated.
        private static int _portBase = 51000;
        private static int NextPort() => Interlocked.Add(ref _portBase, 2);

        public void Dispose() { }

        // ── BallData send ────────────────────────────────────────────────────────

        [Fact]
        public async Task Send_BallData_ArrivesOnLoopback()
        {
            int sendPort  = NextPort();
            int listenPort = sendPort + 1; // unused listen port for this test

            // Set up a raw UdpClient receiver BEFORE creating the transport
            // (so no packets are missed).
            using var receiver = new UdpClient(sendPort);
            receiver.Client.ReceiveTimeout = 2000;

            using var transport = new UdpTransport("127.0.0.1", sendPort, listenPort);

            var ballData = new BallData
            {
                SportId     = "golf",
                TimestampUs = 123456789L,
                PosX = 1f, PosY = 0.5f, PosZ = 3f,
                VelX = 5f, VelY = 2f,   VelZ = 60f,
                SpeedMps       = 60.3f,
                LaunchAngleDeg = 12f,
                SpinRpm        = 2800f,
                Confidence     = 0.95f,
                TrackingTier   = 1
            };

            transport.Send(ballData);

            var remote = new IPEndPoint(IPAddress.Any, 0);
            var raw    = await Task.Run(() => receiver.Receive(ref remote));

            raw.Should().NotBeEmpty("packet should arrive on loopback");
            raw[0].Should().Be((byte)PacketType.BallData, "first byte is packet type");

            var (type, payload) = PacketSerializer.Deserialize(raw);
            type.Should().Be(PacketType.BallData);

            var restored = JsonSerializer.Deserialize<BallData>(payload)!;
            restored.SportId.Should().Be("golf");
            restored.SpeedMps.Should().BeApproximately(60.3f, 0.01f);
            restored.TrackingTier.Should().Be(1);
        }

        [Fact]
        public async Task Send_MultipleBallData_AllArrive()
        {
            int sendPort   = NextPort();
            int listenPort = sendPort + 1;

            using var receiver = new UdpClient(sendPort);
            receiver.Client.ReceiveTimeout = 2000;

            using var transport = new UdpTransport("127.0.0.1", sendPort, listenPort);

            const int count = 5;
            for (int i = 0; i < count; i++)
                transport.Send(new BallData { SportId = "soccer", SpeedMps = i * 2f });

            int received = 0;
            var remote = new IPEndPoint(IPAddress.Any, 0);
            for (int i = 0; i < count; i++)
            {
                var raw = await Task.Run(() => receiver.Receive(ref remote));
                if (raw.Length >= 5) received++;
            }

            received.Should().Be(count, "every sent packet should arrive on loopback");
        }

        // ── ProfileCommand receive ───────────────────────────────────────────────

        [Fact]
        public async Task ProfileCommand_ReceivedEvent_FiresWithCorrectSportId()
        {
            int sendPort   = NextPort();
            int listenPort = sendPort + 1;

            using var transport = new UdpTransport("127.0.0.1", sendPort, listenPort);
            transport.StartListening();

            ProfileSelectCommand? received = null;
            var tcs = new TaskCompletionSource<ProfileSelectCommand>();
            transport.ProfileCommandReceived += cmd =>
            {
                received = cmd;
                tcs.TrySetResult(cmd);
            };

            // Send a ProfileCommand packet directly to the listen port
            var cmd    = new ProfileSelectCommand { SportId = "tennis" };
            var json   = JsonSerializer.SerializeToUtf8Bytes(cmd);
            var buf    = new byte[5 + json.Length];
            buf[0]     = (byte)PacketType.ProfileCommand;
            BitConverter.GetBytes(json.Length).CopyTo(buf, 1);
            json.CopyTo(buf, 5);

            using var sender = new UdpClient();
            sender.Send(buf, buf.Length, "127.0.0.1", listenPort);

            var completedTask = await Task.WhenAny(tcs.Task, Task.Delay(2000));
            completedTask.Should().Be(tcs.Task, "event should fire within 2 seconds");

            received!.SportId.Should().Be("tennis");
        }

        [Fact]
        public async Task ProfileCommand_EventDoesNotFireForBallDataPacket()
        {
            int sendPort   = NextPort();
            int listenPort = sendPort + 1;

            using var transport = new UdpTransport("127.0.0.1", sendPort, listenPort);
            transport.StartListening();

            bool eventFired = false;
            transport.ProfileCommandReceived += _ => eventFired = true;

            // Send a BallData packet (not a ProfileCommand) to the listen port
            var packet = PacketSerializer.Serialize(new BallData { SportId = "golf" });
            using var sender = new UdpClient();
            sender.Send(packet, packet.Length, "127.0.0.1", listenPort);

            // Wait briefly — event must NOT fire
            await Task.Delay(300);
            eventFired.Should().BeFalse("BallData packets should not trigger ProfileCommandReceived");
        }

        // ── Robustness ───────────────────────────────────────────────────────────

        [Fact]
        public async Task MalformedPacket_OnListenPort_DoesNotCrash()
        {
            int sendPort   = NextPort();
            int listenPort = sendPort + 1;

            using var transport = new UdpTransport("127.0.0.1", sendPort, listenPort);
            transport.StartListening();

            // Send garbage bytes — transport should log the error and keep running
            using var sender = new UdpClient();
            sender.Send(new byte[] { 0xFF, 0x01 }, 2, "127.0.0.1", listenPort);

            // Give the receive loop time to process
            await Task.Delay(300);

            // If we reach here without an exception, the test passes
            // Send a valid command afterward to confirm the loop is still alive
            var tcs = new TaskCompletionSource<bool>();
            transport.ProfileCommandReceived += _ => tcs.TrySetResult(true);

            var cmd  = new ProfileSelectCommand { SportId = "baseball" };
            var json = JsonSerializer.SerializeToUtf8Bytes(cmd);
            var buf  = new byte[5 + json.Length];
            buf[0]   = (byte)PacketType.ProfileCommand;
            BitConverter.GetBytes(json.Length).CopyTo(buf, 1);
            json.CopyTo(buf, 5);
            sender.Send(buf, buf.Length, "127.0.0.1", listenPort);

            var completed = await Task.WhenAny(tcs.Task, Task.Delay(2000));
            completed.Should().Be(tcs.Task, "transport should recover after a malformed packet");
        }
    }
}
