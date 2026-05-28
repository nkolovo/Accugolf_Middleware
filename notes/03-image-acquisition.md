# 03 — Image Acquisition

> Pulling frames off the camera: `BeginAcquisition` → loop `GetNextImage` → `Release` → `EndAcquisition`.

## Acquisition modes (the GenICam standard)

Set `AcquisitionMode` before `BeginAcquisition()`:

| Mode | Behavior |
|---|---|
| `Continuous` | Streams frames until `EndAcquisition()`. **Default for live mode.** |
| `SingleFrame` | Captures exactly one frame |
| `MultiFrame` | Captures `AcquisitionFrameCount` frames then stops |

```cpp
CEnumerationPtr acqMode = nodeMap.GetNode("AcquisitionMode");
acqMode->SetIntValue(acqMode->GetEntryByName("Continuous")->GetValue());
```

## The capture loop (C++)

```cpp
cam->BeginAcquisition();

for (int i = 0; i < N; ++i) {
    // Blocks until next image arrives, or timeout (default ~infinite)
    ImagePtr pImage = cam->GetNextImage(1000);   // 1000 ms timeout

    if (pImage->IsIncomplete()) {
        std::cout << "incomplete: " << pImage->GetImageStatus() << "\n";
    } else {
        // ✅ use the image
        size_t w = pImage->GetWidth();
        size_t h = pImage->GetHeight();
        void*  data = pImage->GetData();
        // … push into CV pipeline …
    }

    pImage->Release();   // ← REQUIRED. Returns the buffer to the pool.
}

cam->EndAcquisition();
```

**Critical:** every `GetNextImage()` must be matched by `Release()`. Buffers come from a fixed-size pool; not releasing them starves the camera and you'll start dropping frames.

## Image buffer pool

Driver-side software buffers (default = 10). Bump this if your CV pipeline can be momentarily slow:

```cpp
INodeMap& sMap = cam->GetTLStreamNodeMap();
CIntegerPtr count = sMap.GetNode("StreamBufferCountManual");
count->SetValue(20);
```

Also set the count mode to manual first:
```cpp
CEnumerationPtr mode = sMap.GetNode("StreamBufferCountMode");
mode->SetIntValue(mode->GetEntryByName("Manual")->GetValue());
```

For lossy live streams you can choose `NewestOnly`:
```cpp
CEnumerationPtr handling = sMap.GetNode("StreamBufferHandlingMode");
handling->SetIntValue(handling->GetEntryByName("NewestOnly")->GetValue());
```

## ImagePtr — what it actually owns

`ImagePtr` is a smart pointer to an `Image`. It can either:

1. Point to a **camera buffer** (returned by `GetNextImage`) — must be `Release()`d
2. Point to a **standalone image** (returned by `Image::Create(...)` or `ImageProcessor::Convert(...)`) — owned by the smart pointer, auto-freed

```cpp
// ✅ correct: assign, then use
ImagePtr goodImage;
goodImage = Image::Create(w, h, 0, 0, PixelFormat_BayerRG8, dataBuf);

// ❌ wrong: dereferencing before assignment
ImagePtr illegalImage;
illegalImage->Create(...);   // crash — null pointer
```

You can hold multiple `ImagePtr`s to the same underlying object — reference counted.

## Image-event-driven acquisition (no polling)

Instead of looping on `GetNextImage`, register an `ImageEventHandler` and the SDK pushes frames to you:

```cpp
class GolfBallHandler : public ImageEventHandler {
public:
    void OnImageEvent(ImagePtr image) override {
        // called on the SDK's image-event thread
        if (!image->IsIncomplete()) { /* … */ }
        // do NOT call Release() inside OnImageEvent — SDK does that
    }
};

GolfBallHandler handler;
cam->RegisterEventHandler(handler);
cam->BeginAcquisition();
// … run until stop signal …
cam->EndAcquisition();
cam->UnregisterEventHandler(handler);
```

This is the right pattern for the middleware — it avoids a busy-wait loop and decouples camera timing from your CV pipeline.

See [07 Events & Callbacks](07-events-and-callbacks.md) for full pattern.

## Saving images

```cpp
// pImage->Save("frame_001.jpg");                          // implicit format
pImage->Save("frame_001.png", ImageFileFormat::PNG);       // explicit
pImage->Save("frame_001.raw", ImageFileFormat::RAW);
```

For video, see the `SaveToVideo` example (uses `SpinVideo`).

## Per-frame metadata (timestamps etc.)

Available from every `ImagePtr`:
- `GetTimeStamp()` — camera-side, nanoseconds
- `GetFrameID()` — monotonically increasing
- `GetID()` — buffer ID

For **embedded** metadata (exposure used for THIS frame, gain, GPIO line states), see [08 Chunk Data](08-chunk-data.md).

## Related

- [README](README.md) · [02 System & Enumeration](02-system-and-enumeration.md)
- Next: [04 Hardware Triggering](04-hardware-triggering.md)
- See also: [05 Exposure & Tuning](05-exposure-and-tuning.md), [06 Pixel Formats](06-pixel-formats-and-bayer.md), [07 Events](07-events-and-callbacks.md), [08 Chunk Data](08-chunk-data.md), [09 Error Handling](09-error-handling.md)
