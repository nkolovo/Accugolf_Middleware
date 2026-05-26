// ------------------------------------------------------------
// Transport/UdpTransport.cs
// ------------------------------------------------------------
using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using SportSimulator.Models;

namespace SportSimulator.Transport
{
    public class UdpTransport : IDisposable
    {
        private readonly UdpClient _sender;
        private readonly UdpClient _receiver;
        private readonly IPEndPoint _unityEndpoint;
        private readonly CancellationTokenSource _cts = new();

        public event Action<ProfileSelectCommand>? ProfileCommandReceived;

        public UdpTransport(string unityIp, int sendPort, int listenPort)
        {
            _unityEndpoint = new IPEndPoint(IPAddress.Parse(unityIp), sendPort);
            _sender = new UdpClient();
            _receiver = new UdpClient(listenPort);
            Console.WriteLine($"[UDP] Sending to {unityIp}:{sendPort}, listening on :{listenPort}");
        }

        public void Send(BallData data)
        {
            var packet = PacketSerializer.Serialize(data);
            _sender.Send(packet, packet.Length, _unityEndpoint);
        }

        public void StartListening()
        {
            Task.Run(async () =>
            {
                while (!_cts.IsCancellationRequested)
                {
                    try
                    {
                        var result = await _receiver.ReceiveAsync(_cts.Token);
                        var (type, payload) = PacketSerializer.Deserialize(result.Buffer);
                        if (type == PacketType.ProfileCommand)
                        {
                            var cmd = PacketSerializer.ParseProfileCommand(payload);
                            if (cmd != null) ProfileCommandReceived?.Invoke(cmd);
                        }
                    }
                    catch (OperationCanceledException) { break; }
                    catch (Exception ex) { Console.WriteLine($"[UDP] Receive error: {ex.Message}"); }
                }
            }, _cts.Token);
        }

        public void Dispose() { _cts.Cancel(); _sender.Dispose(); _receiver.Dispose(); }
    }
}