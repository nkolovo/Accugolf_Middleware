# 02 — System & Enumeration

> The bootstrap dance: get the `System` singleton, list interfaces, list cameras, initialize them.

## The lifecycle

```
GetInstance() → GetInterfaces() / GetCameras() → Init() → … work … → DeInit() → Clear() → ReleaseInstance()
```

**Forgetting `ReleaseInstance()` will leak the camera handle until power-cycle.** Wrap everything in try/finally (C#) or RAII guards (C++).

## Minimal C++ enumeration

```cpp
#include "Spinnaker.h"
using namespace Spinnaker;

int main()
{
    // 1. System singleton (process-wide)
    SystemPtr system = System::GetInstance();

    // 2. Discover cameras across all interfaces
    CameraList camList = system->GetCameras();
    unsigned int numCameras = camList.GetSize();

    std::cout << numCameras << " camera(s) found.\n";

    // 3. Init each camera before use
    for (unsigned int i = 0; i < numCameras; ++i) {
        CameraPtr pCam = camList.GetByIndex(i);
        pCam->Init();

        // Read serial number from the TL device nodemap
        INodeMap& tlMap = pCam->GetTLDeviceNodeMap();
        CStringPtr ptrSerial = tlMap.GetNode("DeviceSerialNumber");
        if (IsReadable(ptrSerial))
            std::cout << "  cam " << i << " serial = "
                      << ptrSerial->GetValue() << "\n";

        // (use camera …)

        pCam->DeInit();
    }

    // 4. Cleanup
    camList.Clear();
    system->ReleaseInstance();
}
```

> **Multiple cameras must be instantiated one at a time** (per the docs). Loop and Init() sequentially, don't try to parallelize that step.

## Filtering by serial number

The middleware will almost certainly need to bind to **a specific physical camera** (left/right for stereo, or "the ball cam" vs "the swing cam"). Use `CameraList::GetBySerial()`:

```cpp
CameraPtr pCam = camList.GetBySerial("20123456");
```

Stash serials in a config file rather than hard-coding.

## Interface enumeration (rarely needed)

Useful only for diagnostics or hot-plug logic:

```cpp
InterfaceList ifaceList = system->GetInterfaces();
for (unsigned int i = 0; i < ifaceList.GetSize(); ++i) {
    InterfacePtr iface = ifaceList.GetByIndex(i);
    CameraList camsOnIface = iface->GetCameras();
    // … inspect iface->GetTLNodeMap() for InterfaceID, etc.
}
```

For hot-plug detection use [interface events](07-events-and-callbacks.md) instead of polling.

## Checking node readability before use

Always guard nodemap reads/writes — features can be missing on a given camera model or in a given mode:

```cpp
CEnumerationPtr ptr = nodeMap.GetNode("TriggerSource");
if (!IsReadable(ptr) || !IsWritable(ptr)) {
    // log + skip, don't crash
}
```

`IsReadable`, `IsWritable`, `IsAvailable`, `IsImplemented` are GenAPI free functions — they accept any node smart pointer.

## GigE-specific: heartbeat

GigE Vision cameras have a heartbeat. While **debugging** (stepping in the IDE) you should disable it so the camera doesn't time out:

```cpp
CBooleanPtr hb = nodeMap.GetNode("GevGVCPHeartbeatDisable");
if (IsWritable(hb)) hb->SetValue(true);  // disable for debug
```

**Re-enable in production.** If the app crashes with heartbeat disabled, the camera stays "locked" until power-cycled. See full pattern in `Acquisition.cpp` (`ConfigureGVCPHeartbeat`).

## Related

- [README (index)](README.md) · [01 Spinnaker SDK Overview](01-spinnaker-sdk-overview.md)
- Next: [03 Image Acquisition](03-image-acquisition.md)
- See also: [07 Events & Callbacks](07-events-and-callbacks.md) (hot-plug), [09 Error Handling](09-error-handling.md)
