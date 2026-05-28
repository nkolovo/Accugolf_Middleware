# 10 — Examples Cheatsheet

> The SDK ships ~25 C++ examples and equivalents in C and C#. Which one to crib from for each problem.

## All C++ examples (under `doc/C++/html/examples.html`)

```
Acquisition.cpp                          Enumerate + grab images (start here)
AcquisitionMultipleCameraRecovery.cpp    Multi-cam with hot-plug recovery
AcquisitionMultipleCamerasWriteToFile.cpp  Multi-cam → write raw to disk
AcquisitionMultipleThread.cpp            Multi-cam, one thread per cam
AcquisitionUserBuffer.cpp                Bring your own pinned buffer (zero-copy)
BufferHandling.cpp                       Buffer pool, handling modes (NewestOnly etc.)
Compression.cpp                          On-camera image compression (Oryx)
CounterAndTimer.cpp                      On-camera counters/timers
Exposure.cpp                             Manual exposure config + clamping
FileAccess_Quickspin.cpp                 Upload/download files (firmware, LUTs)
GigEVisionPerformance.cpp                Tune GigE for throughput
ImageFormatControl.cpp                   ROI, pixel format, binning
ImageFormatControl_QuickSpin.cpp         Same, QuickSpin flavor
Inference.cpp                            On-camera ML inference (some Oryx/Forge)
LogicBlock.cpp                           Programmable in-camera logic
NodeMapInfo.cpp                          Dump full nodemap (debugging)
NodeMapInfo_QuickSpin.cpp                Same, QuickSpin flavor
Polarization.cpp                         Polarized sensor handling
SaveToVideo.cpp                          Write H.264/MJPEG/Uncompressed
SpinSimpleGUI_DirectShow.cpp             DirectShow GUI wrapper
StereoAcquisition.cpp                    Stereo camera pair (Bumblebee X)
StereoAcquisition_QuickSpin.cpp          Same, QuickSpin flavor
StereoGPIO.cpp                           Stereo + external GPIO sync
Trigger.cpp                              Software + hardware triggering
Trigger_QuickSpin.cpp                    Same, QuickSpin flavor
```

## Quick lookup: "I need to…"

| Goal | Reach for |
|---|---|
| Basic capture loop | `Acquisition.cpp` |
| Multiple cameras at once | `AcquisitionMultipleCamerasWriteToFile.cpp` or `AcquisitionMultipleThread.cpp` |
| Hot-plug + reconnect | `AcquisitionMultipleCameraRecovery.cpp` + `EnumerationEvents.cpp` |
| Hardware trigger (golf!) | `Trigger.cpp` ([04 Trigger notes](04-hardware-triggering.md)) |
| Tune exposure | `Exposure.cpp` ([05 Tuning notes](05-exposure-and-tuning.md)) |
| ROI crop / pixel format | `ImageFormatControl.cpp` |
| Per-frame metadata | `ChunkData.cpp` ([08 Chunk notes](08-chunk-data.md)) |
| Push-style image events | `ImageEvents.cpp` ([07 Events notes](07-events-and-callbacks.md)) |
| Dump every feature for debugging | `NodeMapInfo.cpp` |
| Record video | `SaveToVideo.cpp` |
| Stereo pair (depth / triangulate ball) | `StereoAcquisition.cpp` + `StereoGPIO.cpp` |
| Custom GPIO logic (compound triggers) | `LogicBlock.cpp` + `CounterAndTimer.cpp` |
| On-camera ML for ball detection | `Inference.cpp` (model on-camera, results streamed via chunks) |

## Where to find the rendered source

Each example is rendered as HTML at:
```
doc/C++/html/_<example_lowercase>_8cpp-example.html
```

For example: `Trigger.cpp` → `doc/C++/html/_trigger_8cpp-example.html`

C# (Managed) equivalents:
```
doc/Managed/html/_<example>_8cs-example.html
```

## Live source (GitHub)

The HTML pages reference the public GitHub repo for the canonical, current versions:

> **https://github.com/Teledyne-MV/Spinnaker-Examples**

When implementing the middleware, clone that repo for compileable copies rather than scraping the HTML in this repo's `doc/` tree.

## Related

- [README](README.md) · [09 Error Handling](09-error-handling.md)
- Next: [11 Unity Integration](11-unity-integration.md)
- See also: [13 Doc-Tree Map](13-sdk-doc-map.md)
