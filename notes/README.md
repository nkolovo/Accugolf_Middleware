# Accugolf Middleware — Notes Index

> Persistent context for future Claude sessions. Start here.

## What this project is

**Accugolf Middleware** is a planned application that uses **FLIR/Teledyne machine-vision cameras** to capture high-speed video of a golf ball and swing, run computer-vision detection/modelling, and stream the resulting ball-flight + swing data to **Unity games** (golf simulators).

The repo currently contains **no source code yet** — only the **FLIR Spinnaker SDK documentation** (the camera SDK that will be used) and this README. These notes capture everything Claude needs to know to help implement the middleware.

See: [00-project-vision.md](00-project-vision.md)

## The notes graph

```
                  ┌──────────────────────┐
                  │  00 Project Vision   │
                  └──────────┬───────────┘
                             │
                  ┌──────────▼───────────┐
                  │ 01 Spinnaker SDK     │
                  │    Overview          │
                  └──────────┬───────────┘
                             │
        ┌────────────────────┼────────────────────┐
        │                    │                    │
   ┌────▼─────┐         ┌────▼──────┐       ┌─────▼──────┐
   │ 02 Sys & │────────▶│ 03 Image  │──────▶│ 04 Hardware│
   │ Enumerate│         │ Acquire   │       │  Trigger   │
   └──────────┘         └────┬──────┘       └──────┬─────┘
                             │                     │
                  ┌──────────▼───────────┐         │
                  │ 05 Exposure / Tuning │◀────────┘
                  └──────────┬───────────┘
                             │
              ┌──────────────┼──────────────┐
              │              │              │
       ┌──────▼─────┐  ┌─────▼─────┐  ┌────▼──────┐
       │ 06 Pixel   │  │ 07 Events │  │ 08 Chunk  │
       │ & Bayer    │  │ Callbacks │  │   Data    │
       └──────┬─────┘  └───────────┘  └───────────┘
              │
       ┌──────▼──────┐
       │ 09 Errors   │
       └──────┬──────┘
              │
       ┌──────▼─────────┐    ┌─────────────────────┐
       │ 10 Examples    │◀──▶│ 13 Doc-Tree Map     │
       │   Cheatsheet   │    │ (where in /doc)     │
       └──────┬─────────┘    └─────────────────────┘
              │
       ┌──────▼────────────┐
       │ 11 Unity Integ.   │
       └──────┬────────────┘
              │
       ┌──────▼────────────┐
       │ 12 Roadmap        │
       └───────────────────┘
```

## All notes

| # | Note | Purpose |
|---|---|---|
| 00 | [Project Vision](00-project-vision.md) | What Accugolf Middleware is and why |
| 01 | [Spinnaker SDK Overview](01-spinnaker-sdk-overview.md) | The camera SDK: what it is, language bindings, GenICam |
| 02 | [System & Enumeration](02-system-and-enumeration.md) | Connect to cameras, list devices, init |
| 03 | [Image Acquisition](03-image-acquisition.md) | Begin/end acquisition, GetNextImage, image lifecycle |
| 04 | [Hardware Triggering](04-hardware-triggering.md) | GPIO triggers — critical for golf swing sync |
| 05 | [Exposure & Camera Tuning](05-exposure-and-tuning.md) | Short exposures for fast ball, gain, gamma, WB |
| 06 | [Pixel Formats & Bayer Data](06-pixel-formats-and-bayer.md) | Raw pixel layout, conversion |
| 07 | [Events & Callbacks](07-events-and-callbacks.md) | Interface + device events |
| 08 | [Chunk Data](08-chunk-data.md) | Per-frame metadata embedded in image |
| 09 | [Error Handling](09-error-handling.md) | Exceptions and ImageStatus codes |
| 10 | [Examples Cheatsheet](10-examples-cheatsheet.md) | Which SDK example does what |
| 11 | [Unity Integration](11-unity-integration.md) | How to ship data from middleware → Unity |
| 12 | [Implementation Roadmap](12-implementation-roadmap.md) | Suggested build order |
| 13 | [Doc-Tree Map](13-sdk-doc-map.md) | Where to find specific reference pages in /doc |

## How these notes are linked

- Each note has a **Related** section at the bottom with backlinks.
- Inline links in body text use plain markdown `[text](file.md)`.
- This file is the hub — every note links back here.

## Key facts to remember

- **SDK version**: Spinnaker SDK 4.2.0.83 (Feb 2025), Teledyne FLIR
- **Camera interfaces**: USB3 Vision and GigE Vision (industrial machine vision, not consumer cameras)
- **Languages supported**: C, C++, C#/.NET, VB.NET, Python (docs cover the first three)
- **Likely model class**: Blackfly S, Forge, Oryx (high-speed global-shutter cameras suited to golf)
- **Build target OS**: Windows (Spinnaker is Windows-first; macOS/Linux supported but most features Windows-only)
