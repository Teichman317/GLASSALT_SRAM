#!/usr/bin/env python3
# ------------------------------------------------------------
# make_strip_bin.py
#   Convert a PNG image directly to raw RGB565 little-endian .bin
#   Works for any size — strip, dial, pointer, etc.
# ------------------------------------------------------------

import struct
import sys
from pathlib import Path

try:
    from PIL import Image
except ImportError:
    print("ERROR: Pillow not installed. Run: pip install Pillow")
    sys.exit(1)

# ---------- CONFIG ----------
INPUT_PNG  = "100sWheel28X567.png"
OUTPUT_BIN = "100sWheel.bin"
# ----------------------------

def rgb888_to_rgb565(r, g, b):
    """Convert 8-bit RGB to 16-bit RGB565"""
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)

def main():
    if not Path(INPUT_PNG).exists():
        print(f"ERROR: File not found: {INPUT_PNG}")
        sys.exit(1)

    img = Image.open(INPUT_PNG).convert("RGB")
    w, h = img.size
    print(f"Image: {INPUT_PNG} → {w}x{h} ({w*h} pixels)")

    pixels = img.load()
    with open(OUTPUT_BIN, "wb") as f:
        for y in range(h):
            for x in range(w):
                r, g, b = pixels[x, y]
                val = rgb888_to_rgb565(r, g, b)
                f.write(struct.pack("<H", val))

    size = Path(OUTPUT_BIN).stat().st_size
    print(f"SUCCESS: {OUTPUT_BIN} → {size:,} bytes ({w}x{h}x2)")

    # Verify
    with open(OUTPUT_BIN, "rb") as f:
        first = f.read(32)
        print("First 16 bytes:", " ".join(f"{b:02X}" for b in first))

if __name__ == "__main__":
    main()
