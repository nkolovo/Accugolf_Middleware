# 09 — Error Handling

> Two distinct error surfaces: API exceptions (config / connection issues) and per-image status flags (transmission integrity).

## API exceptions

C++ uses `Spinnaker::Exception`. Always wrap the camera lifecycle in try/catch:

```cpp
try {
    cam->Init();
    // … configure, acquire …
}
catch (Spinnaker::Exception& e) {
    std::cerr << "Spinnaker error: " << e.what() << "\n";
    // e.GetError() returns the Spinnaker error code (enum)
    // e.GetFullErrorMessage() — extended description
}
```

`Spinnaker::Exception` extends `std::exception`. Useful members:
- `GetError()` — returns a `Spinnaker::Error` enum value
- `GetErrorMessage()` — short
- `GetFullErrorMessage()` — long, with file/line
- `GetFileName()`, `GetLineNumber()`, `GetFunctionName()`

C# equivalent: `SpinnakerException` in `SpinnakerNET`.

## Common exception causes

| Symptom | Cause |
|---|---|
| "Camera is not initialized" | Forgot `Init()` (or called after `DeInit()`) |
| "Buffer was incomplete" (thrown inside Save) | Buffer arrived corrupted — see `ImageStatus` |
| "Node not available" | Feature doesn't exist on this camera/mode; check `IsAvailable` first |
| "Camera is already in use" | Another process (SpinView, another instance) has the camera open |
| GigE timeout / heartbeat | Disable heartbeat in debug, see [02 Sys & Enum](02-system-and-enumeration.md) |
| "PgrLwfDriver not loaded" | On Windows, install or pick a different stream mode |

## Image status (per-frame integrity)

`GetNextImage()` doesn't throw on a bad frame — you must check status:

```cpp
ImagePtr img = cam->GetNextImage(1000);

if (img->IsIncomplete()) {
    ImageStatus s = img->GetImageStatus();
    log(s);
    img->Release();
    continue;
}
```

`ImageStatus` enum (from the docs):

| Value | Meaning |
|---|---|
| `IMAGE_NO_ERROR` | All good |
| `IMAGE_CRC_CHECK_FAILED` | On-wire CRC mismatch (corrupted) |
| `IMAGE_INSUFFICIENT_SIZE` | Buffer smaller than expected |
| `IMAGE_MISSING_PACKETS` | GigE: packets lost in transit |
| `IMAGE_LEADER_BUFFER_SIZE_INCONSISTENT` | Leader incomplete |
| `IMAGE_TRAILER_BUFFER_SIZE_INCONSISTENT` | Trailer incomplete |
| `IMAGE_PACKETID_INCONSISTENT` | Packet IDs out of order |
| `IMAGE_DATA_INCOMPLETE` | Frame truncated |
| `IMAGE_UNKNOWN_ERROR` | Catch-all |

## When you'll see incomplete frames

| Cause | Fix |
|---|---|
| GigE network drops | Use jumbo frames (9000 MTU), check switch, dedicate a NIC, set socket buffers |
| USB3 bus saturation | Lower frame rate, reduce ROI, use Mono8 |
| Slow consumer (your CV) | Bump `StreamBufferCountManual`, use a worker queue ([07 Events](07-events-and-callbacks.md)) |
| Cable too long / damaged | USB3 spec is 3 m max active |

The `BufferHandling.cpp` example shows handling-mode tuning for high-loss scenarios.

## Always release buffers — even bad ones

```cpp
ImagePtr img = cam->GetNextImage(1000);
try {
    // process
}
catch (...) {
    img->Release();
    throw;
}
img->Release();
```

Or use RAII:

```cpp
struct ImageGuard {
    ImagePtr img;
    ~ImageGuard() { if (img) img->Release(); }
};
```

## GenAPI null-pointer / type-mismatch errors

Mis-spelling a node name returns a smart pointer that converts to false on access, but a wrong cast (e.g. `CIntegerPtr` on a float node) throws. Guard with both:

```cpp
CFloatPtr p = nodeMap.GetNode("ExposureTime");
if (!p || !IsReadable(p)) { /* graceful skip */ }
```

## Logging level for production

Set `Error` or `Warning` in production. `Info` and `Debug` write a LOT (`C:\ProgramData\Spinnaker\Logs`). See [07 Events & Callbacks](07-events-and-callbacks.md) for the logging callback pattern.

## Related

- [README](README.md) · [03 Image Acquisition](03-image-acquisition.md)
- Next: [10 Examples Cheatsheet](10-examples-cheatsheet.md)
- See also: [02 System & Enumeration](02-system-and-enumeration.md), [07 Events & Callbacks](07-events-and-callbacks.md)
