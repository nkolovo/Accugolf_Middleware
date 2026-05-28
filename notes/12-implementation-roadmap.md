# 12 — Implementation Roadmap

> Suggested order to build the middleware from scratch. Each milestone is small and demonstrably working.

> ⚠️ Speculative — confirm priorities with the user before executing.

## Milestone 0 — Pick the stack

Open questions to resolve **before writing code**:

1. **Language for the middleware**: C++ or C#?
   - C# pairs natural with Unity if going [Option B](11-unity-integration.md).
   - C++ pairs natural with OpenCV / max performance.
2. **Camera model + hardware**: which Blackfly S / Oryx / Forge does the user actually own?
3. **Trigger source**: IR break-beam? Microphone? Photodiode? Already wired or DIY?
4. **Mono vs color sensor**: mono recommended unless there's a reason for color.
5. **Single camera vs stereo**: spin from a single high-speed camera is hard; stereo simplifies depth + spin axis.
6. **OS**: Windows-only is the safe assumption.
7. **Build system**: CMake, Visual Studio sln, .NET SDK, etc.

## Milestone 1 — Hello camera

Goal: open a camera, grab one frame, save to disk.

- Compile / reference the SDK
- `System::GetInstance()` → `GetCameras()` → `Init()`
- Set `AcquisitionMode = SingleFrame`, `BeginAcquisition()`, `GetNextImage()`, `Save()`, `EndAcquisition()`
- See [02 Sys & Enum](02-system-and-enumeration.md), [03 Acquisition](03-image-acquisition.md)
- ✅ Done when: PNG of your office ceiling on disk.

## Milestone 2 — Tuned capture

Goal: ROI + Mono8 + short exposure + 200+ fps continuous to a counter.

- ROI to a band where the ball will fly (e.g. 1280×240)
- `PixelFormat = Mono8`, `ExposureTime = 200µs`, `Gain` to taste
- `AcquisitionMode = Continuous`
- Image-event handler that just increments a counter
- See [05 Tuning](05-exposure-and-tuning.md), [07 Events](07-events-and-callbacks.md)
- ✅ Done when: stable fps printout matches expected rate; no incomplete frames.

## Milestone 3 — Hardware trigger

Goal: frames arrive only when GPIO Line 0 pulses.

- Wire a button or function generator to Line 0 (5V opto-isolated input)
- Configure trigger per [04 Trigger](04-hardware-triggering.md)
- See chunks light up `LineStatusAll` when triggered (see [08 Chunks](08-chunk-data.md))
- ✅ Done when: pressing the button → exactly one frame; no untriggered frames.

## Milestone 4 — Burst capture

Goal: one trigger → N frames at full speed.

- `TriggerSelector = FrameBurstStart`
- `AcquisitionBurstFrameCount = 8` (or whatever)
- Capture into a per-burst buffer list
- Save burst as a sequence of files for offline CV iteration
- ✅ Done when: one button press produces 8 timestamped frames.

## Milestone 5 — Ball detection (offline)

Goal: from a recorded burst, detect the ball in each frame.

- Tooling: OpenCV (C++ or `OpenCVSharp` if C#)
- Background subtraction or bright-circle detection
- Returns `(x, y)` per frame
- ✅ Done when: ball detected in ≥ 90% of frames across 10 test bursts.

## Milestone 6 — Velocity + launch angle

Goal: from `(x, y, t)` tuples, fit a 2-D velocity vector.

- Use chunk timestamps (not host clock) → see [08 Chunks](08-chunk-data.md)
- Calibrate pixels-per-mm using a known reference at the tee
- Output: speed, launch angle, azimuth
- ✅ Done when: numbers are within 5% of TrackMan / GCQuad reference on a few test shots.

## Milestone 7 — Club + impact

Goal: detect club head in pre-impact frames.

- Different detector — club is faster, less circular, occluded
- Output: club speed, attack angle, club path, face angle (face angle is the hard one)

## Milestone 8 — Spin

Goal: ball spin and spin axis.

- Needs surface markings on ball OR very-high-speed pair-frame analysis
- Often a separate "spin camera" pointed straight at the ball, frame-synced via shared GPIO trigger
- This is the multi-month part — Trackman patents are mostly about this

## Milestone 9 — Unity payload

Goal: emit a shot record to Unity.

- Pick transport ([11 Unity](11-unity-integration.md))
- Define JSON / protobuf schema
- Unity-side: receive, animate ball flight
- ✅ Done when: real shot → flight in Unity within 1 second.

## Milestone 10 — Robustness

- Hot-plug recovery (see `AcquisitionMultipleCameraRecovery.cpp`)
- Graceful camera-busy / not-found handling
- Persisted user-set on camera (load on startup) — see [05 Tuning](05-exposure-and-tuning.md)
- Calibration UI for technician (live preview, ROI adjust)
- Logging + crash reports

## What to commit / leave out

- ✅ Commit: source code, CMake/csproj, calibration files, test bursts (small)
- ❌ Don't commit: Spinnaker SDK installer (vendor binary), recorded video (large)
- The `doc/` tree currently in the repo is the SDK reference — keep it (read-only).

## Related

- [README](README.md) · [00 Project Vision](00-project-vision.md)
- See also: every other note (they're all building blocks of this roadmap)
