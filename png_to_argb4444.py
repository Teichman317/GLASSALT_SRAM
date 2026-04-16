#!/usr/bin/env python3
"""Convert a PNG (with alpha) to raw ARGB4444 little-endian .bin.

Usage:
    python png_to_argb4444.py pointer_source.png pointerargb4444.bin

The output size and pixel layout must match what altimeter_host.c expects:
    PTR_WIDTH  = 63
    PTR_HEIGHT = 240
This script will refuse to write if the dimensions don't match the
existing target .bin (when present), so you can't accidentally ship a
resized image that breaks the rendering geometry.
"""
import struct, sys
from pathlib import Path
from PIL import Image

EXPECTED = {"pointerargb4444.bin": (63, 240)}

def main():
    if len(sys.argv) != 3:
        print(__doc__); sys.exit(1)
    src, dst = sys.argv[1], sys.argv[2]
    img = Image.open(src).convert("RGBA")
    w, h = img.size
    exp = EXPECTED.get(Path(dst).name)
    if exp and (w, h) != exp:
        print(f"ERROR: {src} is {w}x{h}, expected {exp[0]}x{exp[1]} for {dst}")
        print(f"Resize the PNG to {exp[0]}x{exp[1]} before converting.")
        sys.exit(1)
    pixels = img.load()
    with open(dst, "wb") as f:
        for y in range(h):
            for x in range(w):
                r, g, b, a = pixels[x, y]
                p = ((a >> 4) << 12) | ((r >> 4) << 8) | ((g >> 4) << 4) | (b >> 4)
                f.write(struct.pack("<H", p))
    print(f"Wrote {dst} ({w}x{h}, {w*h*2} bytes ARGB4444)")

if __name__ == "__main__":
    main()
