# 01 — Spinnaker SDK Overview

> Teledyne FLIR's machine-vision camera SDK. Wraps the **GenICam** standard for USB3 Vision and GigE Vision industrial cameras.

## What it is

- **Vendor**: Teledyne FLIR Integrated Imaging Solutions
- **Version (this repo)**: 4.2.0.83 (Feb 27, 2025)
- **Interfaces supported**: USB3 Vision 1.0, GigE Vision
- **Language bindings**: C, C++, C#/.NET (managed), VB.NET, Python
- **OS support**: Windows 7/8.1/10/11 primary; macOS/Linux secondary
- **Compilers** (C++ on Windows): Visual Studio 2015–2022

## Architecture (two big parts)

```
┌───────────────────────────────────────────────┐
│                Spinnaker SDK                  │
├───────────────────┬───────────────────────────┤
│  Image            │  Camera Configuration     │
│  Acquisition      │                           │
│  (buffer mgmt,    │  ┌──────────────────────┐ │
│   grabbing)       │  │  QuickSpin API       │ │
│                   │  │  (typed wrapper —    │ │
│                   │  │   easy, autocomplete)│ │
│                   │  └──────────┬───────────┘ │
│                   │             │ wraps       │
│                   │  ┌──────────▼───────────┐ │
│                   │  │  GenAPI (GenICam)    │ │
│                   │  │  (string-keyed nodes)│ │
│                   │  └──────────────────────┘ │
└───────────────────┴───────────────────────────┘
```

### QuickSpin vs GenAPI — when to use which

| API | Example | Use when |
|---|---|---|
| **QuickSpin** | `cam->Gain.SetValue(10.5);` | The setting is one of the common ones (everything in `camera.h`). Type-safe, IDE autocomplete. |
| **GenAPI** | `nodeMap.GetNode("Gain")->SetValue(10.5);` | Setting isn't exposed by QuickSpin, or you need generic code that works across feature names. |

QuickSpin is built on top of GenAPI — it's just a typed wrapper. Most middleware code should prefer QuickSpin and fall back to GenAPI for the long tail.

## GenICam: the standard underneath

Every supported camera ships with an **XML description** of its features (Gain, ExposureTime, Width, TriggerSource, …). The SDK loads that XML at runtime to build a **NodeMap** — a dictionary of named nodes you read/write.

- Cached locally at `C:\ProgramData\Spinnaker\XML` (binary form for speed)
- Three nodemaps per camera:
  - `cam->GetNodeMap()` — main GenICam features (most settings)
  - `cam->GetTLDeviceNodeMap()` — transport-layer device info (serial #, model)
  - `cam->GetTLStreamNodeMap()` — stream-layer info (buffer counts, packet stats)

See [02 System & Enumeration](02-system-and-enumeration.md) for how nodemaps are obtained.

## Header files / namespaces (C++)

```cpp
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
```

In C# the equivalent is `using SpinnakerNET;` and `using SpinnakerNET.GenApi;`.

## Key types you'll see everywhere

| Type | Role |
|---|---|
| `SystemPtr` | Singleton: entry point. `System::GetInstance()` |
| `InterfaceList` / `InterfacePtr` | A physical interface (USB controller / NIC) |
| `CameraList` / `CameraPtr` | A discovered camera |
| `INodeMap` | Dictionary of features |
| `CEnumerationPtr`, `CIntegerPtr`, `CFloatPtr`, `CBooleanPtr` | Typed GenAPI node smart pointers |
| `ImagePtr` | Smart pointer to a captured frame |
| `ImageProcessor` | Pixel format converter (Bayer→RGB/Mono, etc.) |

All `*Ptr` types are reference-counted smart pointers — never `new`/`delete` manually.

## Reference docs in this repo

| Want | Open |
|---|---|
| C++ getting-started | `doc/C++/html/index.html` |
| C++ Programmer's Guide | `doc/C++/html/C-Plus-Plus-ProgrammerGuide.html` |
| C++ examples list | `doc/C++/html/examples.html` |
| C# / .NET docs | `doc/Managed/html/index.html` |
| Plain C docs | `doc/C/` |

See [13 Doc-Tree Map](13-sdk-doc-map.md) for more pointers.

## Related

- [README (index)](README.md) · [00 Project Vision](00-project-vision.md)
- Next: [02 System & Enumeration](02-system-and-enumeration.md)
- See also: [10 Examples Cheatsheet](10-examples-cheatsheet.md), [13 Doc-Tree Map](13-sdk-doc-map.md)
