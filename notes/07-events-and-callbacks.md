# 07 — Events & Callbacks

> Three event classes in Spinnaker: interface events (hot-plug), device events (exposure-end, etc.), image events (each frame). Implement a handler, register it, get callbacks.

## The three handler base classes (C++)

| Base class | Fires when | Register on |
|---|---|---|
| `InterfaceEventHandler` | Camera plug/unplug | `InterfacePtr` |
| `DeviceEventHandler` | Camera-side event (ExposureEnd, etc.) | `CameraPtr` |
| `ImageEventHandler` | A frame arrives | `CameraPtr` |

Each defines one virtual method to override.

## Interface events — hot-plug detection

```cpp
class IfaceHandler : public InterfaceEventHandler {
public:
    void OnDeviceArrival(CameraPtr cam) override {
        // serial available via TL device nodemap
    }
    void OnDeviceRemoval(CameraPtr cam) override {
        std::cout << "removed: "
                  << cam->TLDevice.DeviceSerialNumber.ToString() << "\n";
    }
};

IfaceHandler ih;
InterfaceList ifaces = system->GetInterfaces();
ifaces.GetByIndex(0)->RegisterEventHandler(ih);
```

Use case for middleware: auto-reconnect, log, surface to UI.

## Device events — ExposureEnd, EventTest, etc.

Two steps:
1. Tell the camera **which** event to emit
2. Register a handler

```cpp
// 1. select event + enable notification
CEnumerationPtr sel  = nodeMap.GetNode("EventSelector");
sel->SetIntValue(sel->GetEntryByName("ExposureEnd")->GetValue());

CEnumerationPtr notif = nodeMap.GetNode("EventNotification");
notif->SetIntValue(notif->GetEntryByName("On")->GetValue());

// 2. register handler
class ExpHandler : public DeviceEventHandler {
public:
    void OnDeviceEvent(Spinnaker::GenICam::gcstring eventName) override {
        std::cout << "event=" << eventName
                  << " id="   << GetDeviceEventId() << "\n";
    }
};

ExpHandler eh;
cam->RegisterEvent(eh);
```

ExposureEnd is **useful for golf** — you can fire a strobe LED off of this event, or use it as a timing anchor for sub-frame measurement.

## Image events — the preferred capture pattern

Push-based alternative to looping `GetNextImage()`:

```cpp
class GolfCapture : public ImageEventHandler {
    BallDetector& detector;
public:
    GolfCapture(BallDetector& d) : detector(d) {}

    void OnImageEvent(ImagePtr image) override {
        if (image->IsIncomplete()) return;
        detector.process(image);     // your CV pipeline
        // DO NOT call Release() — SDK manages buffer for image events
    }
};

GolfCapture gc(detector);
cam->RegisterEventHandler(gc);
cam->BeginAcquisition();
// … wait for stop signal …
cam->EndAcquisition();
cam->UnregisterEventHandler(gc);
```

### Threading reality check

`OnImageEvent` is called on **the SDK's image-event thread**. Implications:

- If your CV pipeline is slow, the SDK back-pressures — you'll start dropping frames.
- **Don't do heavy work inside the handler.** Hand the `ImagePtr` to a worker queue (`ConcurrentQueue<ImagePtr>` or boost lock-free) and process on another thread.
- If you hold an `ImagePtr` past the handler's return, the SDK keeps the buffer locked — bump `StreamBufferCountManual` to compensate (see [03 Acquisition](03-image-acquisition.md)).

## C# equivalents

Inherit from:
- `ManagedInterfaceEventHandler`
- `ManagedDeviceEventHandler`
- `ManagedImageEventHandler`

Same `OnDeviceArrival` / `OnDeviceEvent` / `OnImageEvent` virtuals.

## Logging events (a fourth, system-wide kind)

```cpp
class LogCB : public LoggingEvent {
    void OnLogEvent(LoggingEventDataPtr d) override { /* … */ }
};

LogCB lcb;
system->RegisterLoggingEvent((LoggingEvent&)lcb);
system->SetLoggingEventPriorityLevel(SPINNAKER_LOG_LEVEL_NOTICE);
```

Log levels (high → low): Error, Warning, Notice, Info, Debug. Higher level includes all above.

By default SpinView writes to `C:\ProgramData\Spinnaker\Logs`.

## Cleanup

Always `Unregister…` before `DeInit()` / process exit, or the SDK may call into freed memory.

```cpp
cam->UnregisterEventHandler(gc);
cam->UnregisterEvent(eh);
iface->UnregisterEventHandler(ih);
```

## Related

- [README](README.md) · [03 Image Acquisition](03-image-acquisition.md)
- Next: [08 Chunk Data](08-chunk-data.md)
- See also: [09 Error Handling](09-error-handling.md), [10 Examples Cheatsheet](10-examples-cheatsheet.md) (`ImageEvents.cpp`, `DeviceEvents.cpp`, `EnumerationEvents.cpp`, `Logging.cpp`)
