# 11 — Unity Integration

> Spinnaker is C/C++/C#/.NET. Unity is C#. Three reasonable shapes for the bridge.

> ⚠️ This note is **design speculation** — the user hasn't picked an approach yet. Confirm before coding.

## Option A — Native middleware process + UDP/WebSocket

**Architecture:**
```
[ C++ or C# middleware ] ──UDP/JSON──▶ [ Unity game ]
       Spinnaker SDK                       UDP listener
       OpenCV
```

**Pros**
- Clean separation: middleware can be restarted without restarting Unity.
- Middleware can be heavy (OpenCV, CUDA, ML) without bloating the Unity build.
- Multiple Unity instances (or other clients) can subscribe.

**Cons**
- Serialization overhead per shot (small — single JSON / protobuf per shot).
- IPC plumbing.

**When**: Default choice. Pick this unless there's a specific reason.

## Option B — Spinnaker C# SDK directly in Unity

**Architecture:**
```
[ Unity game (C#) ]
   ├─ Spinnaker .NET binding (Managed SDK)
   ├─ OpenCV via OpenCVSharp / EmguCV
   └─ Game logic
```

**Pros**
- No IPC, no serialization, no two processes to manage.
- Easier to ship to customers (single installer).

**Cons**
- Spinnaker uses callbacks on its own threads — Unity main loop is single-threaded. Must marshal results back via `UnitySynchronizationContext` or a queue read on `Update()`.
- Heavy CV work in the same process can stall the renderer.
- Spinnaker is `net48` / WinForms-flavored — verify it works with Unity's runtime (Mono / IL2CPP).

**When**: Single-customer simulator boxes where simplicity wins.

## Option C — Native DLL + Unity P/Invoke

**Architecture:**
```
[ Unity (C#) ] ─P/Invoke─▶ [ native middleware.dll (C++) ]
                              Spinnaker SDK, OpenCV
```

**Pros**
- Performance-first. Heavy CV runs in native code.
- Spinnaker is happiest in C++.

**Cons**
- Marshalling pinned image buffers across the managed/native boundary is delicate.
- Crashes in native code take down Unity.

**When**: Performance is critical, team has native-dev experience.

## Recommended data payload (shot record)

After each detected shot, emit something like:

```json
{
  "shotId": "2026-05-25T22:10:33Z-001",
  "timestampNs": 18293749234,
  "ball": {
    "ballSpeed_mps": 71.2,
    "launchAngle_deg": 11.4,
    "azimuth_deg": -0.8,
    "totalSpin_rpm": 2730,
    "backSpin_rpm": 2680,
    "sideSpin_rpm": -540
  },
  "club": {
    "clubSpeed_mps": 49.6,
    "attackAngle_deg": -2.1,
    "clubPath_deg": 0.8,
    "faceAngle_deg": 0.4,
    "faceToPath_deg": -0.4,
    "impactPosX_mm": 2.3,
    "impactPosY_mm": -1.1
  },
  "quality": {
    "ballConfidence": 0.94,
    "clubConfidence": 0.81,
    "framesUsed": 6
  }
}
```

This roughly matches the data fields commercial launch monitors (TrackMan, GCQuad, Foresight) expose. Unity can render flight from `ballSpeed`, `launchAngle`, `azimuth`, and the three spins.

## Frame rate vs shot rate

The camera streams hundreds of fps continuously, but a shot **event** happens once every ~5 seconds. The middleware should:

1. **Free-run** the camera in a low-cost "armed" state (or trigger-only mode).
2. **On trigger** (break-beam / sound), capture a burst of N frames at full speed (see [04 Trigger](04-hardware-triggering.md), burst mode).
3. **Process** the burst into a shot record.
4. **Emit** one message to Unity.

Unity doesn't need raw frames — it needs the shot result.

## Optional: live preview to Unity

For technician calibration / aim setup, you might want a low-rate live frame stream too. Encode as JPEG and stream over WebSocket at ~30 fps. Don't pump raw frames through IPC — encoding pays for itself many times over.

## Related

- [README](README.md) · [00 Project Vision](00-project-vision.md)
- Next: [12 Implementation Roadmap](12-implementation-roadmap.md)
- See also: [03 Image Acquisition](03-image-acquisition.md), [04 Hardware Triggering](04-hardware-triggering.md), [07 Events & Callbacks](07-events-and-callbacks.md)
