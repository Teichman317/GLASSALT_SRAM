This is a Bendix/King KI-525A Horizontal Situation Indicator (HSI) simulation project. The goal is to render a functional HSI instrument face on a 480x480 pixel display driven by an STM32F769ZI microcontroller using its LTDC (LCD-TFT Display Controller) and Chrom-ART (DMA2D) hardware accelerator.

## Reference Instrument

The KI-525A is a panel-mounted HSI used in general aviation. It combines a heading indicator and a VOR/ILS course deviation indicator into a single instrument. The reference photograph is `52d137-2335_2.webp` (a Mid-Continent Instruments HSI, front-on view). All layer geometry was derived from measurements of this reference image and iteratively refined.

## Architecture Overview

The instrument is rendered by compositing independent bitmap layers, each representing a moving or static element. Layers are loaded from SD card into RAM at startup and composited in real-time using the DMA2D. Each layer can be rotated or translated independently to reflect live avionics data (heading, selected course, course deviation, selected heading).

### Layer Stack (bottom to top, compositing order)

1. **Background** — Static black 480x480 square with 8 white tick marks at 45° intervals (compass rose index marks). Never moves.
2. **CDI Dots Disk** — Black opaque disk (diameter 306px) with 5 white course deviation dots spaced 25px apart horizontally through center. Rotates with selected course.
3. **Compass Card Washer** — Black/white washer-shaped ring (outer diameter 412px, inner diameter 296px) with 72 tick marks at 5° intervals, numeric labels every 30°, and cardinal letters N/S/E/W. Rotates with aircraft heading.
4. **Heading Bug** — Hollow orange/red triangle sitting in the tick mark area of the compass card. Rotates independently to indicate selected heading.
5. **Course Pointer** — Yellow shaft with arrowhead (TO end) running from north to south with a gap in the center for the CDI bar. Rotates with selected course.
6. **CDI Bar** — Yellow vertical bar (176px long) that sits in the course pointer's center gap. Translates perpendicular to the course pointer to indicate course deviation.
7. **Lubber Line + Aircraft Symbol** — Red lubber triangle at north (heading reference), red south tick, and orange aircraft cross symbol at image center. Static overlay, always on top.

## Master Generation Script

**File:** `hsi_generate_layers.py`

Single Python script that generates all layers as 480x480 RGBA PNGs and exports them to optimized binary formats. All geometric parameters are tweakable constants at the top of the file. Run with `python3 hsi_generate_layers.py` to regenerate everything.

### Key Geometry (all measurements in pixels, center at 240,240)

- Canvas: 480x480
- Compass card outer radius: 206 (diameter 412)
- Compass card inner radius: 148 (diameter 296, 5px inside E/W letters)
- Major tick length: 24px (every 10°), width: 3px
- Minor tick length: 14px (every 5°), width: 2px
- Number radius: 162 (center of text placement)
- Inner edge of major ticks: radius 182 (OUTER_RADIUS - MAJOR_TICK_LEN) — this is the key alignment radius for the course pointer tips and lubber triangle tip
- Background tick inner radius: 208, length: 12px (half of major), 8 ticks at 45° intervals

### Color Palette

- Compass card: black (0,0,0) background, white (255,255,255) markings
- Lubber line / heading bug: red/orange (223, 28, 21)
- Aircraft symbol: orange (250, 98, 57)
- Course pointer / CDI bar: yellow-green (223, 223, 101)
- CDI dots: white (255, 255, 255) on black (0, 0, 0) disk
- Background: solid black (0, 0, 0)

### Text Orientation

Compass card labels on the bottom half (angles 90°–270°) are rotated 180° so all text reads from outside looking inward. This is handled by the `draw_text_centered()` function which checks `if 90 < angle_deg < 270` and adds 180° to the rotation.

### Compass Card Labels

```
0°: "N", 30°: "3", 60°: "6", 90°: "E", 120°: "12", 150°: "15",
180°: "S", 210°: "21", 240°: "24", 270°: "W", 300°: "30", 330°: "33"
```

## Binary Export Formats

Three pixel formats are used, chosen to minimize RAM while matching the DMA2D's native blending capabilities:

### L8 (8-bit luminance, no alpha) — 1 byte/pixel
Used for: **Background** (fully opaque, black and white only)
Format: Raw pixel data, row-major, top-to-bottom. Each byte is a luminance value 0x00 (black) to 0xFF (white). No header.

### AL44 (4-bit alpha + 4-bit luminance) — 1 byte/pixel
Used for: **Compass Card**, **CDI Dots Disk** (need alpha transparency, black and white only)
Format: Raw pixel data, row-major, top-to-bottom. High nibble = alpha (0x0=transparent, 0xF=opaque), low nibble = luminance (0x0=black, 0xF=white). No header.

### ARGB1555 (1-bit alpha, 5-5-5 RGB) — 2 bytes/pixel, little-endian
Used for: **Course Pointer**, **CDI Bar**, **Heading Bug**, **Lubber/Aircraft** (need alpha + color)
Format: These four layers are exported as **cropped sprites** to save RAM. Each .bin file has an 8-byte header followed by pixel data for the bounding box only:

```
Header (8 bytes, little-endian):
  uint16_t offset_x;   // X position of top-left corner in the 480x480 canvas
  uint16_t offset_y;   // Y position of top-left corner in the 480x480 canvas
  uint16_t width;       // Width of the cropped sprite in pixels
  uint16_t height;      // Height of the cropped sprite in pixels

Pixel data (width * height * 2 bytes):
  Row-major, top-to-bottom, little-endian uint16_t per pixel
  Bit 15: alpha (1=opaque, 0=transparent)
  Bits 14-10: red (5 bits)
  Bits 9-5: green (5 bits)
  Bits 4-0: blue (5 bits)
```

## Binary File Inventory

| File | Format | Dimensions | Offset | Size | Notes |
|---|---|---|---|---|---|
| background.bin | L8 | 480x480 | (0,0) | 230,400 bytes | Full frame, no header |
| compass_card.bin | AL44 | 480x480 | (0,0) | 230,400 bytes | Full frame, no header |
| cdi_dots.bin | AL44 | 480x480 | (0,0) | 230,400 bytes | Full frame, no header |
| course_pointer.bin | ARGB1555 | 25x365 | (228,58) | 18,258 bytes | 8-byte header + pixels |
| cdi_bar.bin | ARGB1555 | 5x177 | (238,152) | 1,778 bytes | 8-byte header + pixels |
| heading_bug.bin | ARGB1555 | 21x27 | (230,34) | 1,142 bytes | 8-byte header + pixels |
| lubber_aircraft.bin | ARGB1555 | 55x447 | (213,0) | 49,178 bytes | 8-byte header + pixels |
| **Total layers** | | | | **761,556 bytes** | **743.7 KB** |

## Target Hardware

- **MCU:** STM32F769ZI (ARM Cortex-M7)
- **External SDRAM:** 8 Mbit (1,048,576 bytes = 1 MB)
- **Internal SRAM:** 512 KB (524,288 bytes)
- **Total available RAM:** 1,572,864 bytes (1,536 KB)
- **Display:** 480x480 pixels
- **Peripherals used:** LTDC (LCD controller), DMA2D / Chrom-ART (hardware blitter/blender)
- **Storage:** SD card (layers loaded at startup)

## Memory Budget

| Item | Size |
|---|---|
| All 7 layer binaries | 761,556 bytes (743.7 KB) |
| Scratch framebuffer (RGB565) | 460,800 bytes (450.0 KB) |
| **Total RAM needed** | **1,222,356 bytes (1,193.7 KB)** |
| **Total RAM available** | **1,572,864 bytes (1,536.0 KB)** |
| **Headroom** | **350,508 bytes (342.3 KB)** |

The memory fits with 342 KB to spare for application code, stack, DMA buffers, and other runtime needs.

## Runtime Compositing Strategy

At startup, all 7 binary files are read from SD card into RAM. The DMA2D composites the layers into a single RGB565 framebuffer for the LTDC to scan out. The compositing order matches the layer stack described above.

For layers that rotate (compass card, CDI dots, course pointer, heading bug), rotation must be performed in software or by pre-rendering rotated versions. The DMA2D itself does not support rotation — it handles rectangular blits and alpha blending. The cropped sprites (course pointer, CDI bar, heading bug, lubber/aircraft) are blitted to their offset positions using DMA2D memory-to-memory with blending mode.

The CDI bar translates perpendicular to the course pointer direction. In the default (0°) orientation it slides left/right. When the course pointer is rotated, the CDI bar's translation axis rotates with it.

## PNG Files (for reference/preview only, not used at runtime)

- `compass_card_washer.png` — Compass card layer (RGBA)
- `layer_course_pointer.png` — Course pointer layer (RGBA)
- `layer_cdi_bar.png` — CDI bar layer (RGBA)
- `layer_cdi_dots.png` — CDI dots disk layer (RGBA)
- `layer_heading_bug.png` — Heading bug layer (RGBA)
- `layer_lubber_aircraft.png` — Lubber/aircraft layer (RGBA)
- `layer_background.png` — Background layer (RGBA)
- `composite_all_layers_preview.png` — All layers composited on checkerboard
- `*_preview.png` — Individual layer previews on checkerboard
- `52d137-2335_2.webp` — Original reference photograph

## Excel Summary

`hsi_layer_summary.xlsx` — Complete schedule of all binary files with pixel format, dimensions, offsets, file sizes, motion behavior, and memory budget breakdown.

## Possible Future Additions

- TO/FROM flag indicator
- NAV/GS warning flags
- Glideslope deviation indicator (vertical CDI on right side)
- Double buffering (would require a second 450 KB framebuffer — currently 342 KB headroom is not enough, would need further optimization)
