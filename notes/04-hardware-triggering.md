# 04 — Hardware Triggering

> **Critical for golf.** A free-running camera can't reliably catch the millisecond of impact. You need an external trigger (IR break-beam, mic, photodiode) wired to a GPIO line that fires exposure.

## Trigger types

| `TriggerSource` value | Meaning |
|---|---|
| `Software` | Fired by API call (`TriggerSoftware.Execute()`) — useful for testing |
| `Line0`, `Line1`, `Line2`, `Line3` | One of the camera's GPIO pins |
| `Counter0End`, etc. | Internal counter (advanced) |

## Trigger configuration order matters

You **must** turn `TriggerMode` **off** before changing source/selector/activation. Then turn it back on. The Spinnaker `Trigger.cpp` example explicitly enforces this order.

```cpp
// 1. Disable
CEnumerationPtr mode = nodeMap.GetNode("TriggerMode");
mode->SetIntValue(mode->GetEntryByName("Off")->GetValue());

// 2. Select WHICH event you're triggering (start of a frame)
CEnumerationPtr sel = nodeMap.GetNode("TriggerSelector");
sel->SetIntValue(sel->GetEntryByName("FrameStart")->GetValue());

// 3. Source — hardware Line 0
CEnumerationPtr src = nodeMap.GetNode("TriggerSource");
src->SetIntValue(src->GetEntryByName("Line0")->GetValue());

// 4. Edge — rising
CEnumerationPtr act = nodeMap.GetNode("TriggerActivation");
act->SetIntValue(act->GetEntryByName("RisingEdge")->GetValue());

// 5. Re-enable
mode->SetIntValue(mode->GetEntryByName("On")->GetValue());

// Now BeginAcquisition() — frames are gated by Line0 pulses
cam->BeginAcquisition();
```

QuickSpin equivalent (cleaner):
```cpp
cam->TriggerMode.SetValue(TriggerModeEnums::TriggerMode_Off);
cam->TriggerSelector.SetValue(TriggerSelectorEnums::TriggerSelector_FrameStart);
cam->TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Line0);
cam->TriggerActivation.SetValue(TriggerActivationEnums::TriggerActivation_RisingEdge);
cam->TriggerMode.SetValue(TriggerModeEnums::TriggerMode_On);
```

## Software trigger fallback (testing)

For dev/test without the trigger wiring:

```cpp
// during config:
src->SetIntValue(src->GetEntryByName("Software")->GetValue());

// during acquisition loop:
CCommandPtr fire = nodeMap.GetNode("TriggerSoftware");
fire->Execute();
ImagePtr img = cam->GetNextImage(1000);
```

## GPIO physical lines (FLIR cameras)

Typical Blackfly S layout:

| Line | Wire color | Default function |
|---|---|---|
| Line 0 | Black | Opto-isolated input (trigger in) |
| Line 1 | White | Opto-isolated output (flash/strobe out) |
| Line 2 | Red | Bidirectional GPIO |
| Line 3 | Yellow | Bidirectional GPIO |

Always check the **camera-specific Technical Reference Manual** for pinout — varies by model. The Spinnaker docs only describe the API; pin numbers/colors come from the hardware datasheet.

## Strobe output (firing a flash)

Mirror image of trigger input. Configure a line as output, point an event at it:

```cpp
cam->LineSelector.SetValue(LineSelectorEnums::LineSelector_Line1);
cam->LineMode.SetValue(LineModeEnums::LineMode_Output);
cam->LineSource.SetValue(LineSourceEnums::LineSource_ExposureActive);  // strobe while exposing
```

Useful if you're using a short-burst LED flash to freeze ball motion further without bumping ISO/gain.

## Golf-specific design notes

1. **Choose your trigger source.** IR break-beam at the tee is the classic launch-monitor approach. A microphone with peak detection also works (cracks faster than break-beam).
2. **Trigger jitter matters.** If your CV pipeline measures ball velocity by Δposition / Δtime, any jitter in trigger-to-exposure delay is a velocity error. Use the same trigger to fire **all** cameras in a multi-cam rig (gang the input wire).
3. **Burst mode > single-frame.** Configure `TriggerSelector = FrameBurstStart` and `AcquisitionBurstFrameCount = N` so one trigger pulse → N frames at full rate. That's how you get a sequence of the ball mid-flight.
4. **Frame drops on trigger overrun.** Per the example comment: *"if the application/user software triggers faster than frame time, the trigger may be dropped/skipped by the camera."* For multiple frames per trigger use multi-frame / burst mode rather than rapid retriggering.

## Trigger overlap (max throughput)

To allow a new exposure to start while the previous frame is still reading out:

```cpp
CEnumerationPtr overlap = nodeMap.GetNode("TriggerOverlap");
overlap->SetIntValue(overlap->GetEntryByName("ReadOut")->GetValue());
```

Important for high-rate burst capture.

## Cleanup

When done, set `TriggerMode` back to `Off` before `DeInit()` — otherwise the next process to open the camera inherits a triggered camera that won't free-run.

## Related

- [README](README.md) · [03 Image Acquisition](03-image-acquisition.md)
- Next: [05 Exposure & Tuning](05-exposure-and-tuning.md)
- See also: [08 Chunk Data](08-chunk-data.md) (per-frame timestamp), [10 Examples Cheatsheet](10-examples-cheatsheet.md) (`Trigger.cpp`)
