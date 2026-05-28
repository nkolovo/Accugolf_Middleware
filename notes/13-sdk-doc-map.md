# 13 — Doc-Tree Map

> Where to find specific reference material in the (massive) `doc/` tree. ~3000 HTML files total.

## Top of tree

```
doc/
├── FLIR Camera Getting Started.html      # redirect to flir.ca/support-center
├── Spinnaker SDK C_ Getting Started.html # short overview, links to C docs
├── C/                                    # plain-C SDK reference
│   └── html/
├── C++/                                  # C++ SDK — primary reference
│   └── html/  (~2200 files)
└── Managed/                              # .NET (C# / VB) SDK
    └── html/  (~660 files)
```

## C++ entry points (doc/C++/html/)

| File | What it is |
|---|---|
| `index.html` | Getting Started landing |
| `C-Plus-Plus-ProgrammerGuide.html` | **Best high-level guide** — read this in full |
| `_programmer_guide.html` | Same content, different URL slug |
| `examples.html` | List of all C++ examples (see [10 Cheatsheet](10-examples-cheatsheet.md)) |
| `SpinView-Getting-Started.html` | Using the SpinView GUI (camera tuning tool) |
| `_streaming_driver_information.html` | Stream modes (Teledyne GigE, LWF, Socket) |
| `_networking_best_practices.html` | GigE network tuning |

## Specific feature docs (doc/C++/html/, search by name)

| Feature | File pattern |
|---|---|
| Any class | `class_spinnaker_1_1_<lowercased_class>.html` (e.g. `class_spinnaker_1_1_image.html`) |
| Any struct | `struct_<…>.html` |
| Header file | `<header>_8h.html` (e.g. `Camera.h` → `_camera_8h.html`) |
| Example source | `_<example>_8cpp-example.html` (e.g. `Trigger.cpp` → `_trigger_8cpp-example.html`) |
| Function group / module | `group___<group>.html` |

### Naming convention quirks

Doxygen (the doc generator) escapes characters in filenames:
- Uppercase letters get an underscore prefix → `Image` becomes `_image`
- `::` becomes `_1_1`
- Headers' `.h` becomes `_8h`

Examples:
- `Spinnaker::Camera` class → `class_spinnaker_1_1_camera.html`
- `Spinnaker::GenApi::INodeMap` → `class_spinnaker_1_1_gen_api_1_1_i_node_map.html`

You almost always find pages faster with a filesystem `grep` than by guessing slugs:

```bash
grep -l "class Spinnaker::Camera " doc/C++/html/*.html | head
```

## Managed (.NET / C#) entry points (doc/Managed/html/)

| File | What it is |
|---|---|
| `index.html` | Landing |
| `C-Sharp-ProgrammerGuide.html` | C# programmer's guide |
| Examples | `_<example>__c_sharp_8cs-example.html` |

C# example list (excerpt):
```
Acquisition_CSharp.cs            ImageEvents_CSharp.cs
Exposure_CSharp.cs               ImageFormatControl_CSharp.cs
Trigger_CSharp.cs                NodeMapCallback_CSharp.cs
ChunkData_CSharp.cs              SaveToVideo_CSharp.cs
Sequencer_CSharp.cs              Logging_CSharp.cs
Inference_CSharp.cs              EnumerationEvents_CSharp.cs
```

## C (plain) entry points (doc/C/)

Smaller tree. Useful if you ever need pure-C bindings (e.g. wrapping for a language without C++ FFI).

## Things NOT in this `doc/` tree

The SDK reference is **only** for the SDK. Things you'll have to look up elsewhere:

- **Camera model datasheets / TRMs** (pin colors, sensor specs, max fps tables) → [flir.com](https://www.flir.com/support-center/iis/) under "Machine Vision"
- **GenICam spec** itself → [emva.org](https://www.emva.org/standards-technology/genicam/)
- **GigE Vision spec** → AIA / Vision Standards
- **SpinView GUI** is shipped separately by FLIR — install the full SDK to get it

## How to convert the HTML docs to readable text (for AI / grep)

The HTML is Doxygen-output with lots of JavaScript noise. Quick extractor:

```python
from html.parser import HTMLParser

class TextExtractor(HTMLParser):
    def __init__(self):
        super().__init__()
        self.text = []; self.skip = False
    def handle_starttag(self, tag, attrs):
        if tag in ('script', 'style'): self.skip = True
    def handle_endtag(self, tag):
        if tag in ('script', 'style'): self.skip = False
    def handle_data(self, data):
        if not self.skip and data.strip():
            self.text.append(data.strip())

with open('doc/C++/html/some_page.html', errors='ignore') as f:
    p = TextExtractor(); p.feed(f.read())
    print('\n'.join(p.text))
```

## Related

- [README](README.md) · [01 Spinnaker SDK Overview](01-spinnaker-sdk-overview.md)
- See also: [10 Examples Cheatsheet](10-examples-cheatsheet.md)
