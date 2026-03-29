#!/usr/bin/env python3
"""
HSI Background Layer Generator
Generates the layered images for a simulated Horizontal Situation Indicator.
Target display: 480x480, final export format ARGB (TBD: ARGB4444 or ARGB1555)

Layer architecture (bottom to top):
  1. Compass Card Washer  - rotates with aircraft heading
  2. Course Selector Pointer - rotates with selected course
  3. Lubber Line + Aircraft Symbol - FIXED (static overlay)

All layers are 480x480 RGBA PNGs with transparent backgrounds.

Usage: python3 hsi_generate_layers.py
  Generates all layer PNGs plus a composite preview.
"""

import numpy as np
from PIL import Image, ImageDraw, ImageFont
import math
import os
import struct

# ==============================================================
# MASTER PARAMETERS - TWEAK THESE
# ==============================================================

SIZE = 480
CX, CY = SIZE // 2, SIZE // 2

# --- Compass Card ---
OUTER_RADIUS = 206          # outer edge of tick marks (diameter = 412)
INNER_RADIUS = 148          # inner transparent hole (5px inside E/W letters)
MAJOR_TICK_LEN = 24         # length of 10-degree ticks
MINOR_TICK_LEN = 14         # length of 5-degree ticks
MAJOR_TICK_WIDTH = 3        # pixel width of major ticks
MINOR_TICK_WIDTH = 2        # pixel width of minor ticks
NUMBER_RADIUS = OUTER_RADIUS - MAJOR_TICK_LEN - 20  # center of number text
NUM_FONT_SIZE = 20          # font size for numbers (3, 6, 12, 15, etc.)
LETTER_FONT_SIZE = 24       # font size for N, S, E, W
CARD_BG_COLOR = (0, 0, 0, 255)       # black opaque
CARD_FG_COLOR = (255, 255, 255, 255) # white opaque

# --- Lubber Line ---
LUBBER_COLOR = (223, 28, 21, 255)    # red/orange
LUBBER_BASE_HALF_W = 18             # half-width at base (wide end)
LUBBER_TIP_HALF_W = 2               # half-width at tip (narrow end)
LUBBER_TIP_RADIUS = OUTER_RADIUS - MAJOR_TICK_LEN  # tip aligns with inner end of major tick

# --- Aircraft Symbol ---
AIRCRAFT_COLOR = (250, 98, 57, 255)  # orange
AIRCRAFT_LINE_HALF_LEN = 27         # half-length of both main lines (vert & horiz equal)
AIRCRAFT_LINE_WIDTH = 3             # pixel width of all lines
AIRCRAFT_TAIL_OFFSET = 5            # tail line is this many px from south end of vert line
# tail horizontal line is 1/3 the length of the main lines

# --- Course Pointer ---
POINTER_COLOR = (223, 223, 101, 255) # yellow/green
POINTER_SHAFT_HALF_W = 2            # shaft half-width
POINTER_ARROW_HALF_W = 12           # arrowhead half-width at base
POINTER_ARROW_HEIGHT = 20           # arrowhead height (base to tip)
POINTER_TAIL_HALF_W = 12            # tail line half-width
POINTER_TAIL_WIDTH = 3              # tail line thickness
# Both arrowhead tip and tail line stop at inner edge of major ticks
POINTER_LIMIT_RADIUS = OUTER_RADIUS - MAJOR_TICK_LEN  # = 182

# --- CDI Bar ---
CDI_COLOR = POINTER_COLOR               # same yellow as pointer
CDI_HALF_LEN = POINTER_LIMIT_RADIUS // 2 - 3  # gap half (91) minus 3 = 88 each side → 176px total
CDI_HALF_W = POINTER_SHAFT_HALF_W       # same width as pointer shaft

# --- CDI Dots ---
CDI_DOTS_COUNT = 5                          # total dots (center + 2 each side)
CDI_DOT_SPACING = 25                        # pixels between dot centers
CDI_DOT_RADIUS = 3                          # radius of each dot
CDI_DOT_COLOR = (255, 255, 255, 255)        # white
CDI_DISK_RADIUS = INNER_RADIUS + 5          # = 153, slightly larger than compass inner hole
CDI_DISK_BG = (0, 0, 0, 255)               # black opaque

# --- Heading Bug ---
HBUG_COLOR = LUBBER_COLOR                   # same orange/red as lubber line
HBUG_TIP_RADIUS = OUTER_RADIUS             # tip touches outer edge of tick marks (206)
HBUG_BASE_RADIUS = POINTER_LIMIT_RADIUS - 2 # base 2px inside inner end of major ticks (180)
HBUG_BASE_HALF_W = 10                       # half-width of triangle base
HBUG_LINE_WIDTH = POINTER_SHAFT_HALF_W * 2  # match pointer shaft thickness (4)

# --- Background ---
BG_COLOR = (0, 0, 0, 255)                  # solid black, no transparency
BG_TICK_COUNT = 8                           # 8 ticks evenly spaced (every 45°)
BG_TICK_INNER_RADIUS = 208                  # innermost pixel radius from center
BG_TICK_LEN = MAJOR_TICK_LEN // 2           # half the length of major graduations (12)
BG_TICK_WIDTH = MAJOR_TICK_WIDTH            # same thickness as major graduations (3)
BG_TICK_COLOR = CARD_FG_COLOR               # white

# ==============================================================
# FONT SETUP
# ==============================================================
FONT_PATHS = [
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
    "/usr/share/fonts/truetype/freefont/FreeSansBold.ttf",
    "C:/Windows/Fonts/arialbd.ttf",
    "C:/Windows/Fonts/calibrib.ttf",
]

def get_font(size):
    for fp in FONT_PATHS:
        if os.path.exists(fp):
            return ImageFont.truetype(fp, size)
    return ImageFont.load_default()

# ==============================================================
# COMPASS CARD LABELS
# ==============================================================
LABELS = {
    0: "N", 30: "3", 60: "6", 90: "E",
    120: "12", 150: "15", 180: "S", 210: "21",
    240: "24", 270: "W", 300: "30", 330: "33"
}

# ==============================================================
# DRAWING HELPERS
# ==============================================================

def draw_tick(draw_ctx, angle_deg, r_inner, r_outer, width, color):
    angle_rad = math.radians(angle_deg)
    sin_a = math.sin(angle_rad)
    cos_a = math.cos(angle_rad)
    x1 = CX + r_inner * sin_a
    y1 = CY - r_inner * cos_a
    x2 = CX + r_outer * sin_a
    y2 = CY - r_outer * cos_a
    draw_ctx.line([(x1, y1), (x2, y2)], fill=color, width=width)


def draw_text_centered(img, draw_ctx, angle_deg, radius, text, font, color):
    angle_rad = math.radians(angle_deg)
    tx = CX + radius * math.sin(angle_rad)
    ty = CY - radius * math.cos(angle_rad)

    bbox = draw_ctx.textbbox((0, 0), text, font=font)
    tw = bbox[2] - bbox[0]
    th = bbox[3] - bbox[1]

    pad = max(tw, th) * 2 + 8
    txt_img = Image.new('RGBA', (pad, pad), (0, 0, 0, 0))
    txt_draw = ImageDraw.Draw(txt_img)
    txt_x = (pad - tw) // 2 - bbox[0]
    txt_y = (pad - th) // 2 - bbox[1]
    txt_draw.text((txt_x, txt_y), text, fill=color, font=font)

    rot_angle = -angle_deg
    if 90 < angle_deg < 270:
        rot_angle = -angle_deg + 180

    rotated = txt_img.rotate(rot_angle, expand=True, resample=Image.BICUBIC)
    rw, rh = rotated.size
    paste_x = int(tx - rw / 2)
    paste_y = int(ty - rh / 2)
    img.paste(rotated, (paste_x, paste_y), rotated)


def make_checkerboard(size, block=16):
    checker = np.zeros((size, size, 3), dtype=np.uint8)
    for y in range(size):
        for x in range(size):
            if (x // block + y // block) % 2 == 0:
                checker[y, x] = [180, 180, 180]
            else:
                checker[y, x] = [220, 220, 220]
    return Image.fromarray(checker).convert('RGBA')


# ==============================================================
# GENERATE COMPASS CARD WASHER
# ==============================================================
def generate_compass_card(output_dir):
    img = Image.new('RGBA', (SIZE, SIZE), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)

    # Black opaque ring
    draw.ellipse([CX - OUTER_RADIUS, CY - OUTER_RADIUS,
                  CX + OUTER_RADIUS, CY + OUTER_RADIUS], fill=CARD_BG_COLOR)
    draw.ellipse([CX - INNER_RADIUS, CY - INNER_RADIUS,
                  CX + INNER_RADIUS, CY + INNER_RADIUS], fill=(0, 0, 0, 0))

    num_font = get_font(NUM_FONT_SIZE)
    letter_font = get_font(LETTER_FONT_SIZE)

    major_inner = OUTER_RADIUS - MAJOR_TICK_LEN
    minor_inner = OUTER_RADIUS - MINOR_TICK_LEN

    # Tick marks
    for deg in range(0, 360, 5):
        if deg % 10 == 0:
            draw_tick(draw, deg, major_inner, OUTER_RADIUS,
                      MAJOR_TICK_WIDTH, CARD_FG_COLOR)
        else:
            draw_tick(draw, deg, minor_inner, OUTER_RADIUS,
                      MINOR_TICK_WIDTH, CARD_FG_COLOR)

    # Labels
    for deg, label in LABELS.items():
        font = letter_font if label in ("N", "S", "E", "W") else num_font
        draw_text_centered(img, draw, deg, NUMBER_RADIUS, label, font, CARD_FG_COLOR)

    # Clean circular edges with numpy
    arr = np.array(img)
    for y in range(SIZE):
        for x in range(SIZE):
            dist = math.sqrt((x - CX) ** 2 + (y - CY) ** 2)
            if dist > OUTER_RADIUS or dist < INNER_RADIUS:
                arr[y, x, 3] = 0
    img = Image.fromarray(arr)

    img.save(os.path.join(output_dir, "compass_card_washer.png"))
    return img


# ==============================================================
# GENERATE LUBBER LINE + AIRCRAFT LAYER
# ==============================================================
def generate_lubber_aircraft(output_dir):
    img = Image.new('RGBA', (SIZE, SIZE), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)

    # North lubber triangle (pointing down toward center)
    # Tip aligns with inner end of major tick marks
    # Base extends outward, clipped by canvas boundary
    lubber_height = 58  # keep original triangle height
    n_tip_y = CY - LUBBER_TIP_RADIUS
    n_top_y = n_tip_y - lubber_height  # base extends outward (may go < 0)
    draw.polygon([
        (CX - LUBBER_BASE_HALF_W, max(0, n_top_y)),
        (CX + LUBBER_BASE_HALF_W, max(0, n_top_y)),
        (CX + LUBBER_TIP_HALF_W, n_tip_y),
        (CX - LUBBER_TIP_HALF_W, n_tip_y),
    ], fill=LUBBER_COLOR)

    # South lubber mark - thin rectangle same as a major tick mark
    # Starts at outer diameter, extends inward by MAJOR_TICK_LEN
    s_outer_y = CY + OUTER_RADIUS
    s_inner_y = CY + OUTER_RADIUS - MAJOR_TICK_LEN
    draw.rectangle([
        CX - MAJOR_TICK_WIDTH // 2 - 1, s_inner_y,
        CX + MAJOR_TICK_WIDTH // 2 + 1, s_outer_y,
    ], fill=LUBBER_COLOR)

    # Aircraft symbol - two equal lines crossing at exact image center,
    # plus a shorter tail line near the south end of the vertical
    half = AIRCRAFT_LINE_HALF_LEN
    w = AIRCRAFT_LINE_WIDTH
    tail_half = half // 3  # tail is 1/3 the length of main lines
    tail_y = CY + half - AIRCRAFT_TAIL_OFFSET  # 5px from south end

    # Vertical line (fuselage) - centered at (CX, CY)
    draw.line([(CX, CY - half), (CX, CY + half)],
              fill=AIRCRAFT_COLOR, width=w)
    # Horizontal line (wings) - centered at (CX, CY)
    draw.line([(CX - half, CY), (CX + half, CY)],
              fill=AIRCRAFT_COLOR, width=w)
    # Tail horizontal line - 1/3 length, centered on vertical, near south end
    draw.line([(CX - tail_half, tail_y), (CX + tail_half, tail_y)],
              fill=AIRCRAFT_COLOR, width=w)

    img.save(os.path.join(output_dir, "layer_lubber_aircraft.png"))
    return img


# ==============================================================
# GENERATE COURSE POINTER LAYER
# ==============================================================
def generate_course_pointer(output_dir):
    img = Image.new('RGBA', (SIZE, SIZE), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)

    # Shaft runs from inner edge of major ticks (north) to same (south)
    # with a gap in the center (half the total length) for the CDI bar
    shaft_top = CY - POINTER_LIMIT_RADIUS
    shaft_bot = CY + POINTER_LIMIT_RADIUS
    gap_half = POINTER_LIMIT_RADIUS // 2  # half the gap width

    # North shaft segment (from arrowhead down to gap)
    draw.rectangle([
        CX - POINTER_SHAFT_HALF_W, shaft_top,
        CX + POINTER_SHAFT_HALF_W, CY - gap_half,
    ], fill=POINTER_COLOR)

    # South shaft segment (from gap down to tail)
    draw.rectangle([
        CX - POINTER_SHAFT_HALF_W, CY + gap_half,
        CX + POINTER_SHAFT_HALF_W, shaft_bot,
    ], fill=POINTER_COLOR)

    # Arrowhead (north/TO end) - tip at inner edge of major ticks
    arrow_tip_y = CY - POINTER_LIMIT_RADIUS
    arrow_base_y = arrow_tip_y + POINTER_ARROW_HEIGHT
    draw.polygon([
        (CX, arrow_tip_y),
        (CX - POINTER_ARROW_HALF_W, arrow_base_y),
        (CX + POINTER_ARROW_HALF_W, arrow_base_y),
    ], fill=POINTER_COLOR)

    # Tail (south/FROM end) - shaft only, no crossbar

    img.save(os.path.join(output_dir, "layer_course_pointer.png"))
    return img


# ==============================================================
# GENERATE CDI BAR LAYER
# ==============================================================
def generate_cdi_bar(output_dir):
    img = Image.new('RGBA', (SIZE, SIZE), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)

    # Vertical bar centered in image, fits inside the pointer gap
    draw.rectangle([
        CX - CDI_HALF_W, CY - CDI_HALF_LEN,
        CX + CDI_HALF_W, CY + CDI_HALF_LEN,
    ], fill=CDI_COLOR)

    img.save(os.path.join(output_dir, "layer_cdi_bar.png"))
    return img


# ==============================================================
# GENERATE CDI DOTS LAYER
# ==============================================================
def generate_cdi_dots(output_dir):
    img = Image.new('RGBA', (SIZE, SIZE), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)

    # Black opaque disk, slightly larger than compass card inner hole
    draw.ellipse([
        CX - CDI_DISK_RADIUS, CY - CDI_DISK_RADIUS,
        CX + CDI_DISK_RADIUS, CY + CDI_DISK_RADIUS,
    ], fill=CDI_DISK_BG)

    # 5 white dots equally spaced horizontally through center
    half_count = CDI_DOTS_COUNT // 2  # 2 dots each side of center
    for i in range(-half_count, half_count + 1):
        dx = i * CDI_DOT_SPACING
        draw.ellipse([
            CX + dx - CDI_DOT_RADIUS, CY - CDI_DOT_RADIUS,
            CX + dx + CDI_DOT_RADIUS, CY + CDI_DOT_RADIUS,
        ], fill=CDI_DOT_COLOR)

    # Clip to circular disk
    arr = np.array(img)
    for y in range(SIZE):
        for x in range(SIZE):
            dist = math.sqrt((x - CX) ** 2 + (y - CY) ** 2)
            if dist > CDI_DISK_RADIUS:
                arr[y, x, 3] = 0
    img = Image.fromarray(arr)

    img.save(os.path.join(output_dir, "layer_cdi_dots.png"))
    return img


# ==============================================================
# GENERATE HEADING BUG LAYER
# ==============================================================
def generate_heading_bug(output_dir):
    img = Image.new('RGBA', (SIZE, SIZE), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)

    # Hollow triangle, tip pointing outward (north), base closer to center
    tip_y = CY - HBUG_TIP_RADIUS          # tip at outer edge of ticks
    base_y = CY - HBUG_BASE_RADIUS        # base 2px inside major tick inner edge

    draw.polygon([
        (CX, tip_y),                        # tip (top center)
        (CX - HBUG_BASE_HALF_W, base_y),   # base left
        (CX + HBUG_BASE_HALF_W, base_y),   # base right
    ], outline=HBUG_COLOR, width=HBUG_LINE_WIDTH)

    img.save(os.path.join(output_dir, "layer_heading_bug.png"))
    return img


# ==============================================================
# GENERATE BACKGROUND LAYER
# ==============================================================
def generate_background(output_dir):
    img = Image.new('RGBA', (SIZE, SIZE), BG_COLOR)
    draw = ImageDraw.Draw(img)

    # 8 tick marks evenly spaced (every 45°), starting at north (0°)
    bg_tick_outer = BG_TICK_INNER_RADIUS + BG_TICK_LEN
    for i in range(BG_TICK_COUNT):
        angle_deg = i * (360 / BG_TICK_COUNT)  # 0, 45, 90, 135, ...
        draw_tick(draw, angle_deg, BG_TICK_INNER_RADIUS, bg_tick_outer,
                  BG_TICK_WIDTH, BG_TICK_COLOR)

    img.save(os.path.join(output_dir, "layer_background.png"))
    return img


# ==============================================================
# BINARY EXPORT HELPERS
# ==============================================================

def rgba_to_rgb565(r, g, b):
    """Convert 8-bit RGB to RGB565 (16-bit, no alpha)."""
    r, g, b = int(r), int(g), int(b)
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)

def rgba_to_argb1555(r, g, b, a):
    """Convert 8-bit RGBA to ARGB1555 (16-bit, 1-bit alpha)."""
    r, g, b, a = int(r), int(g), int(b), int(a)
    alpha = 1 if a >= 128 else 0
    return (alpha << 15) | ((r >> 3) << 10) | ((g >> 3) << 5) | (b >> 3)

def rgba_to_al44(a, lum):
    """Convert 8-bit alpha + luminance to AL44 (8-bit, 4-bit each)."""
    a, lum = int(a), int(lum)
    return ((a >> 4) << 4) | (lum >> 4)

def rgba_to_l8(lum):
    """Convert 8-bit luminance to L8 (8-bit grayscale)."""
    return int(lum) & 0xFF

def export_rgb565(img, filepath):
    """Export RGBA image as raw RGB565 binary (little-endian)."""
    arr = np.array(img)
    with open(filepath, 'wb') as f:
        for y in range(arr.shape[0]):
            for x in range(arr.shape[1]):
                r, g, b = arr[y, x, 0], arr[y, x, 1], arr[y, x, 2]
                val = rgba_to_rgb565(r, g, b)
                f.write(struct.pack('<H', val))
    return os.path.getsize(filepath)

def export_argb1555(img, filepath):
    """Export RGBA image as raw ARGB1555 binary (little-endian)."""
    arr = np.array(img)
    with open(filepath, 'wb') as f:
        for y in range(arr.shape[0]):
            for x in range(arr.shape[1]):
                r, g, b, a = arr[y, x, 0], arr[y, x, 1], arr[y, x, 2], arr[y, x, 3]
                val = rgba_to_argb1555(r, g, b, a)
                f.write(struct.pack('<H', val))
    return os.path.getsize(filepath)

def export_al44(img, filepath):
    """Export RGBA image as raw AL44 binary (1 byte per pixel)."""
    arr = np.array(img)
    with open(filepath, 'wb') as f:
        for y in range(arr.shape[0]):
            for x in range(arr.shape[1]):
                r, g, b, a = arr[y, x, 0], arr[y, x, 1], arr[y, x, 2], arr[y, x, 3]
                lum = int(0.299 * r + 0.587 * g + 0.114 * b)
                val = rgba_to_al44(a, lum)
                f.write(struct.pack('B', val))
    return os.path.getsize(filepath)

def export_l8(img, filepath):
    """Export RGBA image as raw L8 binary (1 byte per pixel, grayscale)."""
    arr = np.array(img)
    with open(filepath, 'wb') as f:
        for y in range(arr.shape[0]):
            for x in range(arr.shape[1]):
                r, g, b = arr[y, x, 0], arr[y, x, 1], arr[y, x, 2]
                lum = int(0.299 * r + 0.587 * g + 0.114 * b)
                f.write(struct.pack('B', rgba_to_l8(lum)))
    return os.path.getsize(filepath)

def export_argb1555_cropped(img, filepath):
    """Export RGBA image as cropped ARGB1555 sprite with 8-byte header.
    Header (little-endian): offset_x(u16), offset_y(u16), width(u16), height(u16)
    Followed by raw ARGB1555 pixel data for the bounding box only.
    Returns (file_size, offset_x, offset_y, crop_w, crop_h).
    """
    arr = np.array(img)
    # Find bounding box of non-transparent pixels
    alpha = arr[:, :, 3]
    rows = np.any(alpha > 0, axis=1)
    cols = np.any(alpha > 0, axis=0)
    if not np.any(rows):
        # Fully transparent - write empty sprite
        with open(filepath, 'wb') as f:
            f.write(struct.pack('<HHHH', 0, 0, 0, 0))
        return (os.path.getsize(filepath), 0, 0, 0, 0)
    y_min, y_max = np.where(rows)[0][[0, -1]]
    x_min, x_max = np.where(cols)[0][[0, -1]]
    crop_w = x_max - x_min + 1
    crop_h = y_max - y_min + 1
    cropped = arr[y_min:y_max+1, x_min:x_max+1]
    with open(filepath, 'wb') as f:
        # 8-byte header
        f.write(struct.pack('<HHHH', x_min, y_min, crop_w, crop_h))
        # Pixel data
        for y in range(crop_h):
            for x in range(crop_w):
                r, g, b, a = cropped[y, x, 0], cropped[y, x, 1], cropped[y, x, 2], cropped[y, x, 3]
                val = rgba_to_argb1555(r, g, b, a)
                f.write(struct.pack('<H', val))
    return (os.path.getsize(filepath), int(x_min), int(y_min), int(crop_w), int(crop_h))


# ==============================================================
# MAIN
# ==============================================================
if __name__ == "__main__":
    output_dir = os.path.dirname(os.path.abspath(__file__))

    print("Generating background layer...")
    background = generate_background(output_dir)

    print("Generating compass card washer...")
    card = generate_compass_card(output_dir)

    print("Generating lubber line + aircraft layer...")
    lubber = generate_lubber_aircraft(output_dir)

    print("Generating course pointer layer...")
    pointer = generate_course_pointer(output_dir)

    print("Generating CDI bar layer...")
    cdi = generate_cdi_bar(output_dir)

    print("Generating CDI dots layer...")
    dots = generate_cdi_dots(output_dir)

    print("Generating heading bug layer...")
    hbug = generate_heading_bug(output_dir)

    # Composite preview
    # Layer order (bottom to top): dots disk, compass card, heading bug, pointer, cdi bar, lubber/aircraft
    # Rotate heading bug 90° CW for preview (show at 3 o'clock)
    hbug_rotated = hbug.rotate(-90, resample=Image.BICUBIC, center=(CX, CY))

    print("Generating composite preview...")
    checker = make_checkerboard(SIZE)
    composite = Image.alpha_composite(checker, background)
    composite = Image.alpha_composite(composite, dots)
    composite = Image.alpha_composite(composite, card)
    composite = Image.alpha_composite(composite, hbug_rotated)
    composite = Image.alpha_composite(composite, pointer)
    composite = Image.alpha_composite(composite, cdi)
    composite = Image.alpha_composite(composite, lubber)
    composite.save(os.path.join(output_dir, "composite_all_layers_preview.png"))

    # Individual previews
    for name, layer in [("compass_card_washer", card),
                        ("layer_lubber_aircraft", lubber),
                        ("layer_course_pointer", pointer),
                        ("layer_cdi_bar", cdi),
                        ("layer_cdi_dots", dots),
                        ("layer_heading_bug", hbug),
                        ("layer_background", background)]:
        prev = Image.alpha_composite(checker, layer)
        prev.save(os.path.join(output_dir, f"{name}_preview.png"))

    # ==============================================================
    # BINARY EXPORT
    # ==============================================================
    print("\nExporting binary files...")

    bin_exports = []

    # Background - L8 (grayscale, no alpha)
    sz = export_l8(background, os.path.join(output_dir, "background.bin"))
    bin_exports.append(("background.bin", "L8", 8, sz, "Background", "Static", False,
                        "Black square with 8 white tick marks at 45° intervals"))

    # Compass card - AL44 (4-bit alpha + 4-bit luminance)
    sz = export_al44(card, os.path.join(output_dir, "compass_card.bin"))
    bin_exports.append(("compass_card.bin", "AL44", 8, sz, "Compass Card Washer", "Rotates with heading", True,
                        "Black/white washer with ticks, numbers, and cardinal letters"))

    # CDI dots - AL44
    sz = export_al44(dots, os.path.join(output_dir, "cdi_dots.bin"))
    bin_exports.append(("cdi_dots.bin", "AL44", 8, sz, "CDI Dots Disk", "Rotates with course", True,
                        "Black disk with 5 white dots, 25px spacing"))

    # Course pointer - ARGB1555 cropped sprite
    sz, ox, oy, cw, ch = export_argb1555_cropped(pointer, os.path.join(output_dir, "course_pointer.bin"))
    bin_exports.append(("course_pointer.bin", "ARGB1555", 16, sz, "Course Pointer", "Rotates with course", True,
                        f"Cropped sprite {cw}x{ch} at ({ox},{oy}), yellow shaft+arrowhead, center gap for CDI"))
    print(f"    course_pointer: cropped to {cw}x{ch} at offset ({ox},{oy})")

    # CDI bar - ARGB1555 cropped sprite
    sz, ox, oy, cw, ch = export_argb1555_cropped(cdi, os.path.join(output_dir, "cdi_bar.bin"))
    bin_exports.append(("cdi_bar.bin", "ARGB1555", 16, sz, "CDI Bar", "Translates L/R of course", True,
                        f"Cropped sprite {cw}x{ch} at ({ox},{oy}), yellow bar slides for deviation"))
    print(f"    cdi_bar: cropped to {cw}x{ch} at offset ({ox},{oy})")

    # Heading bug - ARGB1555 cropped sprite
    sz, ox, oy, cw, ch = export_argb1555_cropped(hbug, os.path.join(output_dir, "heading_bug.bin"))
    bin_exports.append(("heading_bug.bin", "ARGB1555", 16, sz, "Heading Bug", "Rotates independently", True,
                        f"Cropped sprite {cw}x{ch} at ({ox},{oy}), hollow orange triangle at 0 deg"))
    print(f"    heading_bug: cropped to {cw}x{ch} at offset ({ox},{oy})")

    # Lubber/aircraft - ARGB1555 cropped sprite
    sz, ox, oy, cw, ch = export_argb1555_cropped(lubber, os.path.join(output_dir, "lubber_aircraft.bin"))
    bin_exports.append(("lubber_aircraft.bin", "ARGB1555", 16, sz, "Lubber Line + Aircraft", "Static (fixed overlay)", True,
                        f"Cropped sprite {cw}x{ch} at ({ox},{oy}), red lubber+orange aircraft cross"))
    print(f"    lubber_aircraft: cropped to {cw}x{ch} at offset ({ox},{oy})")

    total_bin = sum(e[3] for e in bin_exports)
    print(f"  Total binary size: {total_bin:,} bytes ({total_bin/1024:.1f} KB)")

    # Add scratch buffer to memory tally
    scratch_size = SIZE * SIZE * 2  # RGB565 framebuffer
    total_ram = total_bin + scratch_size
    sdram = 8 * 1024 * 1024 // 8
    sram = 512 * 1024
    avail = sdram + sram
    print(f"  + scratch buffer (RGB565): {scratch_size:,} bytes ({scratch_size/1024:.1f} KB)")
    print(f"  Total RAM needed: {total_ram:,} bytes ({total_ram/1024:.1f} KB)")
    print(f"  Available: {avail:,} bytes ({avail/1024:.1f} KB)")
    print(f"  Headroom: {avail - total_ram:,} bytes ({(avail-total_ram)/1024:.1f} KB)")

    for name, fmt, bpp, size, *_ in bin_exports:
        print(f"  {name:25s}  {fmt:10s}  {bpp:2d} bpp  {size:>10,} bytes")

    print(f"\nAll files saved to: {output_dir}")
    print(f"\nLayer dimensions:")
    print(f"  Canvas: {SIZE}x{SIZE}")
    print(f"  Compass card outer diameter: {OUTER_RADIUS * 2}")
    print(f"  Compass card inner diameter: {INNER_RADIUS * 2}")
    print(f"  Ring width: {OUTER_RADIUS - INNER_RADIUS}px")
    print("\nDone!")

    print("\nBinary export complete.")
