# 06 — Pixel Formats & Bayer Data

> How camera bytes map to pixels, and how to convert between formats.

## Common pixel formats

| Format | Bytes/pixel | Channels | Notes |
|---|---|---|---|
| `Mono8` | 1 | 1 | 8-bit grayscale — golf default |
| `Mono16` | 2 | 1 | 16-bit grayscale (high dynamic range) |
| `Mono12p` | 1.5 | 1 | 12-bit packed (3 bytes per 2 pixels) |
| `BayerRG8` | 1 | raw | RGGB tile, demosaic in software |
| `BayerGB8` / `BayerGR8` / `BayerBG8` | 1 | raw | Other tile orderings |
| `RGB8` | 3 | 3 | On-camera demosaiced color |
| `BGR8` | 3 | 3 | OpenCV-native order |
| `YCbCr8` | 3 | 3 | YUV |

Set with `cam->PixelFormat.SetValue(...)` (see [05 Tuning](05-exposure-and-tuning.md)).

## Reading raw bytes

```cpp
ImagePtr img = cam->GetNextImage(1000);
unsigned char* data = (unsigned char*)img->GetData();

size_t w = img->GetWidth();
size_t h = img->GetHeight();
size_t stride = img->GetStride();     // bytes per row (≥ w * bytesPerPixel)
size_t bpp = img->GetBitsPerPixel();
PixelFormatEnums fmt = img->GetPixelFormat();
```

`stride` ≠ `width × bytesPerPixel` in general — rows can be padded for alignment. Always use stride when indexing rows.

## Bayer layout (BayerRG8 example)

Each pixel has only one color sample. The tile name is the **top-left 2×2 pattern**:

```
BayerRG8:           BayerGB8:
  R G R G R G         G B G B G B
  G B G B G B         R G R G R G
  R G R G R G         G B G B G B
```

`PixelColorFilter` node on the camera tells you which one to expect.

```cpp
// from the docs, BayerRG8:
// data[0]     = row 0, col 0 = R
// data[1]     = row 0, col 1 = G
// data[width] = row 1, col 0 = G
// data[width+1] = row 1, col 1 = B
```

## Demosaicing with the SDK

`ImageProcessor` converts in-place to any output format:

```cpp
ImageProcessor proc;
proc.SetColorProcessing(ColorProcessingAlgorithm::HQ_LINEAR);  // or BILINEAR / EDGE_SENSING

ImagePtr converted = proc.Convert(rawBayerImage, PixelFormat_BGR8);
```

For middleware: **prefer doing demosaic in OpenCV** (`cv::cvtColor(src, dst, cv::COLOR_BayerRG2BGR)`) if you're already using OpenCV — fewer copies, same quality.

## Format ↔ frame-rate trade-off

| Choice | Bandwidth | Max FPS (typical) |
|---|---|---|
| `Mono8` 1280×720 | 0.9 MB/frame | high |
| `RGB8` 1280×720 | 2.6 MB/frame | ~⅓ of Mono8 |
| `BayerRG8` 1280×720 | 0.9 MB/frame | high (same as Mono8 — single channel on the wire) |

For golf: **Mono8** for speed, or **BayerRG8 + OpenCV demosaic** if you need color.

## Loading an image from disk (for offline tests)

```cpp
int w = 1280, h = 1024;
unsigned char* buf = (unsigned char*)malloc(w * h);

FILE* in = fopen("ball.raw", "rb");
fread(buf, 1, w * h, in);
fclose(in);

ImagePtr loaded = Image::Create(w, h, 0, 0, PixelFormat_Mono8, buf);

ImageProcessor proc;
ImagePtr mono = proc.Convert(loaded, PixelFormat_Mono8);
mono->Save("out.jpg");
```

Useful pattern: **record raw frames during a real range session, then iterate on detection offline.**

## Related

- [README](README.md) · [05 Exposure & Tuning](05-exposure-and-tuning.md)
- Next: [07 Events & Callbacks](07-events-and-callbacks.md)
- See also: [10 Examples Cheatsheet](10-examples-cheatsheet.md) (`ImageFormatControl.cpp`)
