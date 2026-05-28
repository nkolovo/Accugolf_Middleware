# 08 — Chunk Data

> Camera-side metadata appended to each frame: exposure time used, gain applied, GPIO line states, frame counter, timestamp. Critical for accurate offline analysis.

## Image structure with chunks enabled

```
┌─────────────────┐
│     Leader      │
├─────────────────┤
│   Image Data    │
├─────────────────┤
│  Chunk: gain    │
│  Chunk: expo    │
│  Chunk: ts      │
│  Chunk: …       │
├─────────────────┤
│     Trailer     │
└─────────────────┘
```

## Enabling chunks

Two steps:
1. Turn on **chunk mode**
2. Enable each individual chunk you care about

```cpp
// 1. Master switch
cam->ChunkModeActive.SetValue(true);

// 2. Enable specific chunks (selector + enable)
const auto enableChunk = [&](ChunkSelectorEnums sel) {
    cam->ChunkSelector.SetValue(sel);
    cam->ChunkEnable.SetValue(true);
};

enableChunk(ChunkSelectorEnums::ChunkSelector_ExposureTime);
enableChunk(ChunkSelectorEnums::ChunkSelector_Gain);
enableChunk(ChunkSelectorEnums::ChunkSelector_Timestamp);
enableChunk(ChunkSelectorEnums::ChunkSelector_FrameID);
enableChunk(ChunkSelectorEnums::ChunkSelector_LineStatusAll);
```

GenAPI flavor:
```cpp
CEnumerationPtr csel = nodeMap.GetNode("ChunkSelector");
csel->SetIntValue(csel->GetEntryByName("ExposureTime")->GetValue());
CBooleanPtr cen = nodeMap.GetNode("ChunkEnable");
cen->SetValue(true);

CBooleanPtr active = nodeMap.GetNode("ChunkModeActive");
active->SetValue(true);
```

## Reading chunks off a frame

```cpp
ImagePtr img = cam->GetNextImage(1000);
const ChunkData& chunks = img->GetChunkData();

float64_t expUsedUs = chunks.GetExposureTime();   // µs
float64_t gainUsed  = chunks.GetGain();
int64_t   ts        = chunks.GetTimestamp();       // ns since camera epoch
int64_t   frameId   = chunks.GetFrameID();
int64_t   lines     = chunks.GetLineStatusAll();  // bitmask of GPIO states
```

`ChunkData` is a thin accessor over the frame buffer — no copy.

## Why this matters for golf

| Chunk | Why it matters |
|---|---|
| `ExposureTime` | If exposure was auto-adjusted, you need this to compute light/motion blur |
| `Timestamp` | Camera-clock timestamp of the frame — use this for velocity calc (don't trust host clock) |
| `FrameID` | Detect dropped frames (gaps in the sequence) |
| `LineStatusAll` | Was the IR break-beam high when this frame was exposed? Confirms trigger source. |
| `Gain` | Same reason as ExposureTime |
| `ImageCRC` | Detect on-wire corruption |

## Per-line GPIO chunks

If you have multiple GPIO inputs (e.g. one for tee sensor, one for swing sensor), `LineStatusAll` gives a bitmask — line 0 in bit 0, line 1 in bit 1, etc.

You can also enable the per-line chunk:
```cpp
cam->ChunkSelector.SetValue(ChunkSelectorEnums::ChunkSelector_Line0LineStatus);
cam->ChunkEnable.SetValue(true);
```

## Chunks vs `ImagePtr` properties

| Source | What it gives | Trustworthy? |
|---|---|---|
| `img->GetTimeStamp()` | Buffer-level timestamp | yes |
| `img->GetFrameID()` | Buffer-level frame ID | yes |
| `chunks.GetTimestamp()` | Camera-internal timestamp | yes — and matches the exact frame even after buffer-level reordering |
| `chunks.GetExposureTime()` | Actual exposure used | **only way to know** — `cam->ExposureTime.GetValue()` reads the **current setting**, not the value used for this specific frame |

The chunk values are authoritative for offline analysis. Use them.

## Caveats

- Each enabled chunk **adds bytes per frame** — keep the list lean.
- Chunks must be re-enabled after `UserSetLoad` if they weren't part of the saved set.
- Not all chunks are supported on all cameras — query availability via `IsAvailable(chunkSelector->GetEntryByName(...))`.

## Related

- [README](README.md) · [03 Image Acquisition](03-image-acquisition.md)
- Next: [09 Error Handling](09-error-handling.md)
- See also: [04 Hardware Triggering](04-hardware-triggering.md) (LineStatus correlates to triggers), [10 Examples Cheatsheet](10-examples-cheatsheet.md) (`ChunkData.cpp`)
