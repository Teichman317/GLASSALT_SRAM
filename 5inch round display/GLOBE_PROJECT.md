# 3D Spinning Globe — Project Notes
**Target displays:** 5" Round LCD QT050YMM-N41 (1080×1080, HX8399-C, 4-lane MIPI DSI)  
**Primary host (final):** Raspberry Pi 4 Model B → direct DSI  
**Interim host:** Raspberry Pi 4 Model B → HDMI-MIPI-2KV05 converter board  
**PC simulation:** Windows 11, MSYS2/MinGW, SDL2 + OpenGL (desktop GL, not ES)  
**Language:** ANSI C  
**Future target (separate effort):** STM32F767ZI — no GPU, requires software renderer, out of scope here

---

## 1. Project Phases

| Phase | Host | Display path | Status |
|-------|------|--------------|--------|
| 1 | Windows PC (MSYS2) | SDL2 window, desktop OpenGL | First — validate look |
| 2 | Pi 4B | HDMI → HDMI-MIPI-2KV05 → panel | Quick hardware check |
| 3 | Pi 4B | Native DSI via custom adapter board + hx8399 kernel driver | Final goal |

---

## 2. Panel Quick Reference (QT050YMM-N41)

| Parameter | Value |
|-----------|-------|
| Resolution | 1080 × 1080 px (square) |
| Active area | 127.008 × 127.008 mm (circular mask) |
| Driver IC | HX8399-C |
| Interface | 4-lane MIPI DSI |
| IOVCC | 1.8 V (typ) |
| VSP | +5 V |
| VSN | −5 V |
| Backlight | 38.4 V forward, 20 mA, ~0.77 W |
| Refresh rate | 60 Hz max (panel + IC hard limit — 120 Hz not achievable) |
| Connector | Hirose DF40C-50DP-0.4V (50-pin, 0.4 mm pitch) |

### Key panel pins (50-pin connector)
| Pin(s) | Signal | Notes |
|--------|--------|-------|
| 1,2,7,8,13,14,19,20,25,26,31,37,42 | GND | |
| 3,4 | LAN2_P/N | MIPI data lane 2 |
| 5,6 | LAN2_N/P | |
| 9,10 | LAN1_P/N | MIPI data lane 1 |
| 11,12 | LAN1_N/P | |
| 15,16 | CLK_P/N | MIPI clock lane |
| 17,18 | ID_PIN1/2 | ID pins, pd GND / pu 1.8V |
| 21,22 | LAN0_P/N | MIPI data lane 0 |
| 23,24 | IOVCC | 1.8 V digital supply |
| 27,28 | LAN3_P/N | MIPI data lane 3 |
| 29,30 | LAN3_N/P | |
| 32,33 | NC | |
| 34,35,36 | NC | |
| 38,36 | VDD(+5) | VSP analog rail |
| 32,30 | VDD(−5) | VSN analog rail |
| 44 | LEDPWM | Backlight PWM output |
| 46 | TE | Tearing effect out |
| 48 | RESET | Active low, ≥10 µs pulse |
| 12,10 | LEDA | LED anode |
| 6,4 | LEDK | LED cathode |

### Power-on sequence (critical — follow this order)
1. IOVCC rises
2. VSP rises (>1 ms after IOVCC)
3. VSN rises (>1 ms after VSP)
4. RESET pulse (≥10 µs low)
5. MIPI signal starts (>50 ms after RESET)
6. Backlight on (>200 ms after MIPI signal start)

Power-off is reverse order with different hold times — see datasheet section 6.0.

---

## 3. HDMI-MIPI-2KV05 Converter Board

Shenzhen GL Electronics, preliminary spec V0 (2025-06-05).

| Parameter | Value |
|-----------|-------|
| Input | Mini HDMI 1.4 |
| Power | +5V via Micro USB |
| MIPI output | Up to 2 ports, 4 lanes each, 1 clock lane |
| FPC output | 40-pin, 0.5 mm pitch, bottom contact |
| Supported res | 1080×1080, 1080×1920 |
| Board size | 49 × 47 × 4.3 mm |
| MIPI standard | DSI 1.02, D-PHY 1.2, DCS 1.02 |

The 40-pin FPC carries two MIPI ports (N0 and N1). For a single-port panel like the QT050YMM-N41, use the **N1 port** (pins 20–30 per the red-box callout in the datasheet — MIPI_N1_D0±, MIPI_N1_CLK±, MIPI_N1_D1±, MIPI_N1_D2±). Also note pins 32–33 (1V8), 34 (VGL), 35 (VGH), 36 (3V3), 37–38 (LED−), 39–40 (LED+).

**The converter handles HDMI→MIPI translation entirely in hardware.** The Pi just sees a normal HDMI monitor. No kernel driver or DSI configuration needed for Phase 2.

---

## 4. Pi 4B HDMI Custom Resolution (Phase 2)

The Pi 4B's HDMI output doesn't know about 1080×1080 natively. Add a custom mode to `/boot/firmware/config.txt` (Pi OS Bullseye/Bookworm) or `/boot/config.txt` (older):

```ini
hdmi_group=2
hdmi_mode=87
hdmi_cvt=1080 1080 60 1 0 0 0
hdmi_drive=2
```

Verify after boot:
```bash
tvservice -s          # should report 1080x1080
fbset -s              # framebuffer size
ls /dev/dri/          # expect card0, renderD128
```

---

## 5. Phase 1 — PC Simulation (MSYS2/MinGW, SDL2 + OpenGL)

### 5.1 Install dependencies

```bash
pacman -S mingw-w64-x86_64-SDL2 \
          mingw-w64-x86_64-glew \
          mingw-w64-x86_64-libpng \
          mingw-w64-x86_64-gcc \
          make
```

GLEW is used here to load OpenGL extensions on Windows. On the Pi we switch to OpenGL ES 2.0 + EGL and drop GLEW.

### 5.2 Texture — NASA Blue Marble

Download a free equirectangular Earth texture (public domain):
```
https://visibleearth.nasa.gov/images/57735/the-blue-marble-land-surface-ocean-color-sea-ice-and-clouds
```
A 4096×2048 PNG is a good balance of quality vs. memory. Name it `earth.png` and place in the project directory alongside the executable.

Alternatively a 2048×1024 version works fine on screen and loads faster.

### 5.3 Source file structure

```
globe/
├── main.c          — SDL2 init, window (1080×1080), event loop
├── globe.c         — sphere mesh generation, draw call
├── globe.h
├── shader.c        — load/compile/link GLSL shaders
├── shader.h
├── texture.c       — PNG load via libpng → GL texture
├── texture.h
├── vert.glsl       — vertex shader
├── frag.glsl       — fragment shader
├── earth.png       — equirectangular texture
└── Makefile
```

### 5.4 Sphere mesh

Generate a UV sphere at startup (not from file). Typical parameters:
- 64 stacks × 64 slices = 4096 vertices — smooth enough at 1080px
- Interleaved VBO: `[x, y, z, nx, ny, nz, u, v]` per vertex
- Index buffer for triangle strip or indexed triangles

```c
/* Pseudocode — fills out arrays you allocate */
for (int stack = 0; stack <= STACKS; stack++) {
    float phi = M_PI * stack / STACKS;          /* 0 .. PI */
    for (int slice = 0; slice <= SLICES; slice++) {
        float theta = 2.0f * M_PI * slice / SLICES;  /* 0 .. 2PI */
        float x = sinf(phi) * cosf(theta);
        float y = cosf(phi);
        float z = sinf(phi) * sinf(theta);
        /* normal = position (unit sphere) */
        float u = (float)slice / SLICES;
        float v = (float)stack / STACKS;
        /* push x,y,z,x,y,z,u,v */
    }
}
```

### 5.5 Shaders (desktop GLSL — PC only)

**vert.glsl**
```glsl
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNorm;
layout(location = 2) in vec2 aUV;

uniform mat4 uMVP;
uniform mat4 uModel;

out vec3 vNorm;
out vec2 vUV;

void main() {
    vNorm = mat3(uModel) * aNorm;
    vUV   = aUV;
    gl_Position = uMVP * vec4(aPos, 1.0);
}
```

**frag.glsl**
```glsl
#version 330 core
in vec3 vNorm;
in vec2 vUV;

uniform sampler2D uTex;
uniform vec3 uLightDir;   /* normalised, world space */

out vec4 FragColor;

void main() {
    vec3 n    = normalize(vNorm);
    float diff = max(dot(n, normalize(uLightDir)), 0.0);
    vec4 col  = texture(uTex, vUV);
    FragColor = vec4(col.rgb * (0.15 + 0.85 * diff), 1.0);
}
```

The 0.15 ambient term keeps the night side from going fully black. Adjust to taste.

### 5.6 Makefile (MSYS2/MinGW)

```makefile
CC      = gcc
CFLAGS  = -Wall -O2 -std=c99
LIBS    = -lSDL2 -lGLEW -lopengl32 -lpng -lz -lm
TARGET  = globe.exe
SRCS    = main.c globe.c shader.c texture.c

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) $(SRCS) -o $(TARGET) $(LIBS)

clean:
	rm -f $(TARGET)
```

### 5.7 SDL2 window — circular mask

The display is physically round but the framebuffer is square (1080×1080). On the PC sim you can optionally draw a circular clip in software (draw a black annulus over the corners) to preview the round look accurately. Simple approach: after rendering the globe, draw a fullscreen quad with a circle-clip fragment shader that discards fragments outside `length(uv - 0.5) > 0.5`.

On the Pi with the actual panel the hardware already masks to the circle — no software clip needed.

### 5.8 Rotation

Simple spin: accumulate angle each frame based on elapsed milliseconds.

```c
float angle = 0.0f;
Uint32 last = SDL_GetTicks();

/* in loop */
Uint32 now  = SDL_GetTicks();
float  dt   = (now - last) / 1000.0f;
last = now;
angle += 20.0f * dt;   /* degrees per second — adjust freely */
if (angle > 360.0f) angle -= 360.0f;
```

Build model matrix: `Ry(angle)`. Apply tilt if desired: `Rx(-23.5°)` for Earth's axial tilt.

---

## 6. Phase 3 — Direct DSI (Custom Adapter Board)

This is the goal hardware path. Notes from prior investigation:

### 6.1 Adapter board requirements

| Component | Purpose | Notes |
|-----------|---------|-------|
| TPS65132 | VSP (+5V) / VSN (−5V) bias rails | I2C programmable, single-supply input |
| Boost converter | Backlight LED string (~38.4V, 20mA) | e.g. TPS61040 or similar; PWM dim via LEDPWM |
| Pi DSI FFC | 15-pin, 1mm pitch | Pi 4B DSI0 or DSI1 connector |
| Panel FFC | Hirose DF40C-50DP-0.4V to board | 0.4mm pitch, fragile — handle with care |
| RESET GPIO | Active-low reset to panel pin 48 | Any free Pi GPIO, pulled high |
| Level shift | 3.3V GPIO → 1.8V RESET if needed | Panel IOVCC is 1.8V |

### 6.2 DSI impedance requirements

MIPI D-PHY differential pairs must be routed as 100Ω differential / 50Ω single-ended controlled impedance. Keep pairs matched length, avoid vias, minimize stub length at connector.

### 6.3 Pi kernel driver

The `hx8399` driver exists in mainline kernel. Check:
```bash
grep -r hx8399 /boot/overlays/   # may already have a dtbo
modinfo hx8399                   # if module is built
```

A device tree overlay is required. Minimum overlay parameters:
- `compatible = "himax,hx8399c"` (check exact string against kernel version)
- Clock rate, lane count (4), panel size, timing values from section 5.0 of the datasheet
- RESET GPIO
- Power regulators (reference TPS65132 regulator node)

Timing values from datasheet section 5.0 to plug into the overlay:

| Parameter | Value |
|-----------|-------|
| H active | 1080 px |
| H front porch | 16 DCK |
| H sync width | 25 DCK |
| H back porch | 25 DCK |
| V active | 1080 lines (528+8xNL where NL=66.5, typ) |
| V front porch | 2 lines |
| V sync width | 2 lines |
| V back porch | 2 lines |
| Refresh | 60 Hz |

### 6.4 OpenGL ES on Pi (Phase 3 code changes)

When porting from Phase 1 (desktop GL) to Phase 3 (Pi, OpenGL ES 2.0):

- Replace GLEW with EGL + `libGLESv2`
- Change shader `#version 330 core` → `#version 100` (GLSL ES 1.0) or `#version 300 es`
- Replace `in`/`out` with `attribute`/`varying` if targeting ES 1.0 shaders
- Replace `glGenVertexArrays` / `glBindVertexArray` with manual attribute binding (no VAOs in ES 2.0)
- EGL surface replaces SDL2's GL context on the Pi bare-metal path (no X11)
- SDL2 can still be used on the Pi if X11/Wayland is present; on Lite without desktop, use DRM/KMS + EGL directly

Install on Pi:
```bash
sudo apt install libgles2-mesa-dev libegl1-mesa-dev libgbm-dev libdrm-dev libpng-dev
```

---

## 7. STM32F767ZI — Future Separate Effort

The F767ZI has no GPU. OpenGL does not apply. Options for a display on that platform:

- **LTDC peripheral** — parallel RGB interface, drives a framebuffer directly; no DSI support on F767
- **SPI/parallel display** — small lower-res panels (e.g. ST7789, ILI9341)
- **Software renderer** — CPU-side scanline rasterizer for simple 3D; very limited at 1080×1080
- **External GPU** — e.g. Renderscript over SPI, or a dedicated display controller IC

The QT050YMM-N41 (MIPI DSI, 1080×1080) is **not compatible** with the STM32F767ZI directly — the F767 has no MIPI DSI output. A bridge IC (e.g. SSD2828) could convert parallel RGB to DSI but this is a complex separate project.

---

## 8. SD Card Sizing

| Item | Approx |
|------|--------|
| Pi OS Lite base | ~2.5 GB |
| Dev libs (Mesa, EGL, libpng, SDL2, gcc) | ~500 MB |
| Earth texture (4K PNG) | ~80 MB |
| Kernel headers (for Phase 3 DT work) | ~400 MB |
| Headroom | rest |
| **Recommended card** | **32 GB** |

8 GB will be tight once dev tools and kernel headers are installed. 32 GB is comfortable.

---

## 9. Reference Files (this directory)

| File | Contents |
|------|----------|
| `QT050YMM-N41.pdf` | Panel datasheet — electrical, timing, mechanical, pinout |
| `HDMI-MIPI-2KV05.pdf` | Converter board datasheet — FPC pinout, features |
| `GLOBE_PROJECT.md` | This file |

---

*Last updated: 2026-04-02*
