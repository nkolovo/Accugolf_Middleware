// ------------------------------------------------------------
// App/Program.cs
// ------------------------------------------------------------
using System;
using SportSimulator.App;

var unityIp   = args.Length > 0 ? args[0] : "127.0.0.1";
var sendPort  = args.Length > 1 ? int.Parse(args[1]) : 7100;
var listenPort = args.Length > 2 ? int.Parse(args[2]) : 7101;

Console.WriteLine("=== SportSimulator ===");
Console.WriteLine($"Unity target : {unityIp}:{sendPort}");
Console.WriteLine($"Listen port  : {listenPort}");
Console.WriteLine("Press Ctrl+C to exit.\n");

using var engine = new SimulatorEngine();

Console.CancelKeyPress += (_, e) => { e.Cancel = true; engine.Stop(); };

engine.Start(unityIp, sendPort, listenPort);