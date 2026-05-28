# 00 — Project Vision

> **One-line:** Apps that read, detect, model, and send ball/swing info to Unity games.

## What it does (system-level)

```
   FLIR camera(s)
        │
        │ USB3 / GigE
        ▼
  ┌──────────────────────────────────┐
  │   Accugolf Middleware            │
  │                                  │
  │  1. Capture frames (Spinnaker)   │
  │  2. Detect ball + club           │
  │  3. Model trajectory / spin /    │
  │     swing path                   │
  │  4. Serialize data               │
  │  5. Send → Unity                 │
  └──────────────┬───────────────────┘
                 │ UDP / shared mem /
                 │ named pipe / WebSocket
                 ▼
            Unity golf game
```

## The four functional stages (from the README)

| Stage | Meaning | Likely tech |
|---|---|---|
| **Read** | Pull frames off the camera at high frame rate | Spinnaker SDK — see [03 Image Acquisition](03-image-acquisition.md) |
| **Detect** | Find the ball and club head in each frame | OpenCV / custom CV, possibly ML |
| **Model** | Compute ball flight (velocity, spin, launch angle), club path | Physics / fitting |
| **Send** | Push results to Unity in real time | UDP / shared memory — see [11 Unity Integration](11-unity-integration.md) |

## Why FLIR / Spinnaker (and not a webcam)

A golf ball off a driver leaves the clubface at ~70 m/s (160 mph). Capturing it without motion blur needs:

- **Sub-millisecond exposure** (e.g. 100 µs) → see [05 Exposure & Tuning](05-exposure-and-tuning.md)
- **Hardware trigger** synced to impact (IR break-beam or sound trigger) → see [04 Hardware Triggering](04-hardware-triggering.md)
- **High frame rate** (200+ fps typical for launch monitors) — needs USB3 / GigE bandwidth
- **Global shutter** (no rolling-shutter skew) — standard on Blackfly S / Oryx
- **Deterministic timing + per-frame metadata** → see [08 Chunk Data](08-chunk-data.md)

Consumer webcams have rolling shutters and auto-exposure that can't be controlled tightly enough.

## What's in the repo right now

```
Accugolf_Middleware/
├── README.md            # one-liner only
├── .gitattributes
└── doc/
    ├── C/               # Spinnaker C SDK docs
    ├── C++/             # Spinnaker C++ SDK docs (~2200 HTML files)
    ├── Managed/         # Spinnaker .NET (C#/VB) SDK docs (~660 files)
    ├── Spinnaker SDK C_ Getting Started.html
    └── FLIR Camera Getting Started.html  (redirect to flir.ca)
```

**No source code yet.** All the docs are vendor reference material the dev (likely Jalal) staged before starting implementation. The build language is not yet chosen — see [12 Roadmap](12-implementation-roadmap.md).

## Related

- [README (index)](README.md)
- Next: [01 Spinnaker SDK Overview](01-spinnaker-sdk-overview.md)
- See also: [12 Implementation Roadmap](12-implementation-roadmap.md), [11 Unity Integration](11-unity-integration.md)
