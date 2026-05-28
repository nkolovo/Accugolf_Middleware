# 05 — Exposure & Camera Tuning

> Freezing a 160 mph golf ball needs sub-millisecond exposure. Here are the knobs.

## Exposure time

ExposureTime is in **microseconds**.

```cpp
// QuickSpin
cam->ExposureAuto.SetValue(ExposureAutoEnums::ExposureAuto_Off);
cam->ExposureMode.SetValue(ExposureModeEnums::ExposureMode_Timed);
cam->ExposureTime.SetValue(100);     // 100 µs — very short, golf-appropriate
```

```cpp
// GenAPI
CEnumerationPtr auto_ = nodeMap.GetNode("ExposureAuto");
auto_->SetIntValue(auto_->GetEntryByName("Off")->GetValue());

CEnumerationPtr mode = nodeMap.GetNode("ExposureMode");
mode->SetIntValue(mode->GetEntryByName("Timed")->GetValue());

CFloatPtr et = nodeMap.GetNode("ExposureTime");
et->SetValue(100);
```

### Picking an exposure for golf

| Ball speed | Pixels of blur in 1 ms | Recommended exposure |
|---|---|---|
| 30 m/s (driver chip) | ~30 px @ 1 mm/px | ≤500 µs |
| 70 m/s (driver) | ~70 px @ 1 mm/px | ≤100 µs |
| 80 m/s (tour driver) | ~80 px @ 1 mm/px | ≤50 µs |

Rule of thumb: keep motion blur under one ball diameter / 10 = ~4 mm.

### Clamping to the camera's range

Always read the min/max before setting — varies by model and current pixel format:

```cpp
CFloatPtr et = nodeMap.GetNode("ExposureTime");
double mn = et->GetMin();
double mx = et->GetMax();
et->SetValue(std::clamp(desired_us, mn, mx));
```

The `Exposure.cpp` SDK example does exactly this.

## Gain (dB)

Short exposures mean less light → bump gain. Trade-off: noise.

```cpp
cam->GainAuto.SetValue(GainAutoEnums::GainAuto_Off);
cam->Gain.SetValue(10.5);   // dB
```

Same min/max clamping pattern applies.

## Gamma

```cpp
cam->Gamma.SetValue(1.5);
```

Default ~1.0. For CV pipelines, **leave gamma at 1.0** (or disable: `GammaEnable = false`) — you want linear pixel response for accurate detection.

## Black Level (DC offset)

```cpp
cam->BlackLevelSelector.SetValue(BlackLevelSelectorEnums::BlackLevelSelector_All);
cam->BlackLevel.SetValue(1.5);   // percent
```

## White Balance (color cameras only)

```cpp
cam->BalanceWhiteAuto.SetValue(BalanceWhiteAutoEnums::BalanceWhiteAuto_Off);
cam->BalanceRatioSelector.SetValue(BalanceRatioSelectorEnums::BalanceRatioSelector_Blue);
// then write balance via BalanceRatio node
```

For golf, **mono cameras** are usually better — more light per pixel, no demosaic, simpler CV. Only use color if you need to differentiate club types or read shaft markings.

## ROI (region of interest) — speed booster

If you only care about a strip of the image (e.g. ~200 px tall band where the ball flies), crop on-sensor. Smaller frames → higher max frame rate → less data over USB/GigE.

```cpp
cam->Width.SetValue(1280);
cam->Height.SetValue(240);
cam->OffsetX.SetValue(0);
cam->OffsetY.SetValue(420);    // center vertically
```

Width/Height/Offset must be **multiples of the sensor's increment** (often 4 or 8). Read `GetInc()` on the node before setting.

## Acquisition Frame Rate

For free-run (non-triggered) capture:

```cpp
CBooleanPtr fpsEnable = nodeMap.GetNode("AcquisitionFrameRateEnable");
fpsEnable->SetValue(true);
cam->AcquisitionFrameRate.SetValue(500.0);  // 500 fps
```

In hardware-triggered mode (the golf case), frame rate is **dictated by trigger pulses** — this setting is ignored / capped.

## Pixel Format

Affects both bit depth and max frame rate:

| Format | Bits | Note |
|---|---|---|
| `Mono8` | 8 | Fastest, smallest bandwidth |
| `Mono12p` / `Mono16` | 12/16 | Higher dynamic range |
| `BayerRG8` | 8 (raw color) | Cheapest color path, demosaic in CPU |
| `BGR8` / `RGB8` | 24 | Demosaiced on-camera (slower) |

```cpp
cam->PixelFormat.SetValue(PixelFormatEnums::PixelFormat_Mono8);
```

For golf middleware: **start with Mono8**.

See [06 Pixel Formats](06-pixel-formats-and-bayer.md) for raw layout details.

## User Sets — saving config to camera

Once you have a tuned configuration, save it as a user set so the camera boots into it:

```cpp
cam->UserSetSelector.SetValue(UserSetSelectorEnums::UserSetSelector_UserSet0);
cam->UserSetSave.Execute();
// later:
cam->UserSetDefault.SetValue(UserSetDefaultEnums::UserSetDefault_UserSet0);
```

This is great for production: the technician tunes once, the middleware just calls `UserSetLoad` on startup.

## Related

- [README](README.md) · [04 Hardware Triggering](04-hardware-triggering.md)
- Next: [06 Pixel Formats & Bayer Data](06-pixel-formats-and-bayer.md)
- See also: [03 Image Acquisition](03-image-acquisition.md), [10 Examples Cheatsheet](10-examples-cheatsheet.md) (`Exposure.cpp`, `ImageFormatControl.cpp`)
