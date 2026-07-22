# 14 — Session Recap: Hardware Bring-Up + 3D Spin (2026-07-22)

> Catch-up doc for a fresh Claude session (e.g. on another machine). Read this
> first, then dig into the referenced files as needed.

## Who/what

Nikolas is building this middleware to add multi-sport ball detection,
tracking, and spin to an AccuGolf Hawkeye 2-camera rig, streaming results to
Unity games over UDP. This session happened on-site at the actual hardware,
which resolved most of the previously-unknown physical setup, and then built
out real spin measurement.

**Games in scope:** soccer, hockey, baseball, football (field-goal kicks
only, not thrown spirals), possibly lacrosse. **Golf is explicitly
out of scope** — the AccuGolf Hawkeye already handles it natively; this
middleware is for everything else.

## Confirmed hardware facts (do not re-ask)

- **Spinnaker SDK**: `C:\Program Files\Teledyne\Spinnaker\bin64\vs2015\`, SDK 4.2.0.83. Single managed assembly `SpinnakerNET_v140.dll` (not two DLLs like older SDKs) — wired into [SportSimulator.csproj](../SportSimulator.csproj).
- **Camera model**: Blackfly S BFS-PGE-04S2M, **monochrome** (Mono8 native, no color/Bayer data available — matters if any sport ever wants color-based detection).
- **Resolution**: 720×540, **fixed/locked — no ROI windowing available**. This closes off "higher fps via a smaller ROI" as a path to real spin for fast-spinning sports (see Spin section below).
- **Frame rate**: confirmed 199.76fps at default (full-frame) settings.
- **Stereo baseline**: 19.5in center-to-center = 0.4953m.
- **Mounting geometry**: sensor 114in above floor, 33in horizontally forward of the tee/ball spot → tilt ≈16.15° off nadir (≈73.85° off horizontal). **No yaw** (confirmed via SpinView sightline check against the real shot line), no lateral offset.
- **Camera roles**: left = serial 24182871, right = serial 24193779. Spinnaker's raw enumeration order does NOT reliably match this — [CameraManager.cs](../Vision/CameraManager.cs) assigns `CameraIndex` by serial number (`SerialToCameraIndex` map) rather than trusting enumeration order. **Update that map if a camera is ever swapped.**
- **Trigger config**: SpinView reported `TriggerSource=Software`, ambiguous whether `TriggerMode` was On (which would silently starve the app of frames — nothing calls a software trigger). Fixed defensively: `CameraManager` now forces `TriggerMode=Off` explicitly regardless of the camera's persisted state.

## Real bugs found + fixed this session

- `net48` had **never actually compiled** before this session — multiple real API mismatches against the installed SDK (logging handler signature, `GetTLStreamNodeMap()` not `GetStreamNodeMap()`, `.ManagedData` not `.GetData()`), plus .NET Framework doesn't have `MathF`/`Math.Clamp` (added [Compat/NetFrameworkCompat.cs](../Compat/NetFrameworkCompat.cs) polyfill), plus a `record` type needing `IsExternalInit` (swapped for a tuple in [StereoCalibrator.cs](../Vision/Calibration/StereoCalibrator.cs)).
- **Camera left/right was backwards** — see camera roles above. Would have silently degraded all real detections to low-confidence monocular tracking.
- `StereoRectifier` was built at startup but **never actually applied to frames** — triangulation was running on unrectified coordinates. Fixed in [SimulatorEngine.cs](../App/SimulatorEngine.cs) (`RectifyFrame`), which is also a hard prerequisite for the 3D spin work (needs epipolar-rectified rows for stereo point matching).
- `fy = fx * (cy/cx)` in [StereoCalibrationData.cs](../Vision/Calibration/StereoCalibrationData.cs) was mathematically wrong for a square-pixel sensor (fy should just equal fx, independent of aspect ratio) — fixed, regression test added.
- `net48`'s `UdpClient` has no cancellable `ReceiveAsync(CancellationToken)` — [UdpTransport.cs](../Transport/UdpTransport.cs) now uses the standard close-the-socket cancellation pattern (works on both targets).
- `FindStereoMatch`'s original fixed 60px disparity search window was far too small for this rig's real geometry (~289px expected disparity at the ball spot, ranging ~170–870px across a shot's depth). Now searches around an *expected* disparity derived from the ball's own just-triangulated position each frame.
- Normalized correlation is degenerate (spurious high-confidence match) on a textureless patch — added a variance guard in [FeaturePointTracker.cs](../Vision/FeaturePointTracker.cs).

## Spin measurement — two tiers, know the difference

**Why two systems exist:** a single camera watching from nearly overhead can only see rotation about its own viewing axis. Backspin/topspin and a tumbling field-goal kick both rotate about a *horizontal* axis lying in the image plane — invisible to simple 2D correlation from this rig's mounting angle. Sidespin/curl (vertical axis) is the one case simple 2D correlation handles well here.

1. **[SpinEstimator.cs](../Vision/SpinEstimator.cs)** — single-camera 2D rotation correlation. Magnitude only, no axis. Reliable only where spin rate stays under ~90°/frame at 200fps (soccer's ~300–600rpm; NOT golf/baseball/tennis, which alias badly — and golf/tennis are out of scope anyway).
2. **[RotationFitter.cs](../Tracking/RotationFitter.cs) + [FeaturePointTracker.cs](../Vision/FeaturePointTracker.cs) + [Spin3DEstimator.cs](../Vision/Spin3DEstimator.cs)** — full 3D spin (magnitude AND axis), built this session. Natural-texture tracking (GFTT corners + optical flow + rectified stereo matching + Kabsch rigid-rotation fit), not marked/painted balls — chosen to avoid a practice-ball workflow change. Wired into `SimulatorEngine`; populates `BallData.SpinAxisX/Y/Z` for the first time. Takes priority over the 2D estimate when it produces a valid fit.

**Both are covered by unit tests against synthetic data (149 tests total, all passing) — this proves the math and wiring are correct. It does NOT prove real ball surfaces have enough natural texture contrast at 720×540 under real lighting for GFTT to track reliably. That's the biggest open unknown, only checkable against real footage.**

Ball choice matters: cameras are monochrome, so only grayscale contrast counts, not color. A classic black/white pentagon ball is the safer bet — a modern colorful ball can look flat/featureless in grayscale even if it looks richly patterned to the eye.

**Live validation added:** `SimulatorEngine` now prints `[Spin3D] rpm=... axis=(x,y,z) points=N` every time the 3D fit succeeds (unthrottled — whether this line appears at all is itself diagnostic), and a throttled `[Ball] pos=... speed=... tier=... spin=...` trajectory line (~10Hz) during tracking. To sanity-check axis output against real shots: a pure backspin shot should show `axis.X` dominant; a pure sidespin/curl shot should show `axis.Z` dominant with a smaller `axis.Y` (~±0.28, from the known 16.15° tilt) — not noise, expected geometry. `AxisUnit` is camera-relative, not yet rotated into labeled backspin/sidespin/spiral terms.

## Sport-specific spin decisions

- **Soccer**: real spin wanted and achievable (2D or 3D).
- **Football**: field-goal kicks only (no thrown spirals in scope) → backspin/tumble case, needs the 3D method.
- **Baseball**: spin/curve explicitly ignored — assume straight flight.
- **Hockey**: no spin needed (`OutputSpin = false` already in its profile) — pucks don't backspin meaningfully.
- **Lacrosse**: no solid spin-rate reference data; likely fine at 200fps if wanted, but low confidence — probably treat like baseball (ignore) unless proven otherwise.
- **Tennis**: dropped from scope entirely (that game won't be made).

## Still open / not done

1. **Real checkerboard calibration hasn't been run.** Everything currently runs on computed-from-measurements fallback defaults (baseline, resolution confirmed exactly; focal length only approximated from a floor-FOV measurement). No runnable calibration CLI exists yet — just the `StereoCalibrator` class. Offered to build a small harness; not done.
2. **Exposure tuning** — [CameraManager.cs](../Vision/CameraManager.cs)'s per-sport exposure map is still placeholder values; needs live SpinView tuning against real lighting.
3. **`KalmanCoastFrames`** ([SportProfileRegistry.cs](../Profiles/SportProfileRegistry.cs)) were tuned assuming 120fps; real fps is ~200, so the same frame counts now mean shorter real-world coast windows than originally intended. Not yet recomputed — open decision, not yet raised with Nikolas again since the calibration/spin work took priority.
4. **Unity-side readiness is unknown** — this repo only handles camera → detection → UDP send (`BallData`/`ProfileSelectCommand`, wire format in [PacketSerializer.cs](../Transport/PacketSerializer.cs), default ports 7100 send / 7101 listen). Whether the Unity soccer game already has a receiver for this is outside this repo and hasn't been confirmed.
5. **Must run the net48 build for real cameras** — [Program.cs](../App/Program.cs) picks real Spinnaker vs `MockCameraManager` based on compiled target framework, not a runtime flag. `dotnet run` can easily end up building `net10.0-windows` (mock/synthetic data) instead. Use `dotnet build -f net48` then run the `.exe` directly from `bin\...\net48\`, or run via Visual Studio targeting net48.

## Related

- [README (index)](README.md) · [00 Project Vision](00-project-vision.md)
- [11 Unity Integration](11-unity-integration.md) — the wire-protocol design this session actually implemented
- [12 Implementation Roadmap](12-implementation-roadmap.md)
