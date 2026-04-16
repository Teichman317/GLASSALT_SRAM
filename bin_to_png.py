#!/usr/bin/env python3
"""Convert raw RGB565 / ARGB4444 .bin pixel dumps back to PNG for editing.

Usage:
    python bin_to_png.py pointerargb4444.bin 63 240 argb4444 pointer.png
    python bin_to_png.py image.bin 480 480 rgb565 dial.png
"""
import struct, sys
from pathlib import Path
from PIL import Image

def argb4444_to_rgba(p):
    a = ((p >> 12) & 0xF) * 17
    r = ((p >>  8) & 0xF) * 17
    g = ((p >>  4) & 0xF) * 17
    b = ( p        & 0xF) * 17
    return (r, g, b, a)

def rgb565_to_rgba(p):
    r = ((p >> 11) & 0x1F) << 3; r |= r >> 5
    g = ((p >>  5) & 0x3F) << 2; g |= g >> 6
    b = ( p        & 0x1F) << 3; b |= b >> 5
    return (r, g, b, 255)

def main():
    if len(sys.argv) != 6:
        print(__doc__); sys.exit(1)
    src, w, h, fmt, dst = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), sys.argv[4].lower(), sys.argv[5]
    expected = w * h * 2
    raw = Path(src).read_bytes()
    if len(raw) != expected:
        print(f"ERROR: {src} is {len(raw)} bytes, expected {expected} for {w}x{h} 16bpp"); sys.exit(1)
    decode = {"argb4444": argb4444_to_rgba, "rgb565": rgb565_to_rgba}.get(fmt)
    if not decode: print(f"ERROR: unknown format '{fmt}'"); sys.exit(1)
    img = Image.new("RGBA", (w, h))
    pixels = img.load()
    for y in range(h):
        for x in range(w):
            (p,) = struct.unpack_from("<H", raw, (y * w + x) * 2)
            pixels[x, y] = decode(p)
    img.save(dst)
    print(f"Wrote {dst} ({w}x{h} RGBA)")

if __name__ == "__main__":
    main()
