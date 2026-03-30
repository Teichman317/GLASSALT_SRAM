/*
 * HSI (Horizontal Situation Indicator) Simulator
 * Desktop SDL2 renderer for 480x480 KI-525A HSI face
 * with clickable GUI control panel
 *
 * ---------------------------------------------------------------------------
 * OVERVIEW
 * ---------------------------------------------------------------------------
 * This program simulates a Bendix/King KI-525A HSI (Horizontal Situation
 * Indicator) — the primary navigation instrument in many general-aviation
 * aircraft.  It composites seven graphical layers into a 480x480 pixel
 * instrument face, exactly mirroring the layer architecture used on the
 * real STM32F767 hardware (LTDC + DMA2D).  Below the instrument face
 * is a 110-pixel-tall GUI control panel with clickable buttons for
 * adjusting heading, course, CDI deviation, and heading bug.
 *
 * The simulation runs entirely on the desktop using SDL2 for windowing,
 * input, and display.  All instrument artwork is loaded from pre-rendered
 * binary asset files exported from the embedded project's asset pipeline.
 *
 * ---------------------------------------------------------------------------
 * KI-525A HSI PRIMER
 * ---------------------------------------------------------------------------
 * The KI-525A displays:
 *   - A rotating COMPASS CARD (the circular degree ring) that shows the
 *     aircraft's current magnetic heading.  The card rotates so that the
 *     current heading is always under the fixed lubber line at the top.
 *   - A COURSE POINTER (a long arrow/needle) that the pilot sets to the
 *     desired VOR radial or localizer course.  It rotates independently
 *     of the compass card, staying referenced to magnetic north.
 *   - A CDI BAR (Course Deviation Indicator) — a movable bar that slides
 *     left and right perpendicular to the course pointer, indicating how
 *     far off-course the aircraft is.  The CDI bar rotates WITH the course
 *     pointer but also translates laterally.  This combined rotation +
 *     translation is the key visual feature of the HSI.
 *   - CDI DOTS — fixed reference dots (typically 5 on each side) that
 *     rotate with the course pointer, providing a scale for the CDI bar.
 *   - A HEADING BUG — a small marker on the compass card rim that the
 *     pilot sets to the desired heading for the autopilot.
 *   - A LUBBER LINE / AIRCRAFT SYMBOL — a fixed overlay at the top of
 *     the display indicating the aircraft's nose direction.
 *
 * ---------------------------------------------------------------------------
 * LAYER COMPOSITING ORDER (bottom to top)
 * ---------------------------------------------------------------------------
 * The instrument is built by painting layers in this exact order.  Each
 * successive layer paints on top of the previous ones, using alpha
 * blending where the layer format supports transparency.
 *
 *   Layer 1: BACKGROUND (L8 grayscale, 480x480, static / no rotation)
 *       The static bezel, face plate, and any fixed markings.  L8 format
 *       means each pixel is a single 8-bit luminance value (256 grays).
 *       This is the fastest layer to draw because there is no rotation
 *       and no alpha blending — it is simply copied to the framebuffer.
 *
 *   Layer 2: CDI DOTS DISK (AL44, 480x480, rotates with course)
 *       The five-dot-per-side deviation scale that the CDI bar slides
 *       across.  This layer rotates with the COURSE setting (not heading)
 *       so the dots always stay aligned with the course pointer.
 *       It is drawn UNDER the compass card so the card's opaque center
 *       area and outer ring obscure the dots outside the course window.
 *       AL44 format = 4-bit alpha + 4-bit luminance per byte.
 *
 *   Layer 3: COMPASS CARD (AL44, 480x480, rotates with heading)
 *       The numbered degree ring (0-360) that rotates to show current
 *       heading under the lubber line.  This is drawn OVER the CDI dots
 *       so that the card's opaque outer ring and center hub mask the
 *       dots, leaving only the narrow course window visible.
 *
 *   Layer 4: HEADING BUG (ARGB1555 sprite, rotates with heading bug)
 *       A small triangular marker on the compass card rim.  It rotates
 *       by (hdg_bug_setting - heading) so it appears at the correct
 *       position on the already-rotated compass card.
 *
 *   Layer 5: COURSE POINTER (ARGB1555 sprite, rotates with course)
 *       The long arrow/needle that shows the selected course.  Rotates
 *       by (course - heading) relative to the display.
 *
 *   Layer 6: CDI BAR (ARGB1555 sprite, rotates with course + lateral offset)
 *       ***THIS IS THE KEY LAYER***  The CDI bar both rotates (with the
 *       course pointer) AND translates laterally (perpendicular to the
 *       pointer axis) to show course deviation.  See the detailed
 *       explanation at the rotate_blend_sprite() call below.
 *
 *   Layer 7: LUBBER / AIRCRAFT SYMBOL (ARGB1555 sprite, static)
 *       The fixed aircraft silhouette and lubber line at the top.
 *       Drawn last so it is always on top of all rotating elements.
 *
 * ---------------------------------------------------------------------------
 * PIXEL FORMATS
 * ---------------------------------------------------------------------------
 *   L8:        8-bit luminance.  Each byte is a grayscale intensity 0-255.
 *              No alpha channel — always fully opaque.  Used for the
 *              background because it is the cheapest format (1 byte/pixel).
 *
 *   AL44:      4-bit alpha, 4-bit luminance packed into one byte.
 *              High nibble = alpha (0=transparent, 15=opaque).
 *              Low nibble  = luminance (0=black, 15=white).
 *              Both are scaled to 8-bit by multiplying by 17 (0x11),
 *              which maps 0->0, 1->17, ..., 15->255 perfectly.
 *              Used for the compass card and CDI dots because they need
 *              transparency (for the see-through areas) but are grayscale.
 *
 *   ARGB1555:  1-bit alpha, 5-bit red, 5-bit green, 5-bit blue.
 *              Bit 15 (MSB) = alpha: 1=opaque, 0=fully transparent.
 *              Bits 14-10 = red (0-31), bits 9-5 = green, bits 4-0 = blue.
 *              Each 5-bit channel is scaled to 8-bit by shifting left 3
 *              (equivalent to multiplying by 8), giving values 0,8,16,...248.
 *              Used for colored sprites (course pointer, CDI bar, heading
 *              bug, lubber) because they have color and need transparency
 *              but only on/off alpha (no partial transparency).
 *
 * These formats match what the STM32 LTDC and DMA2D hardware natively
 * support, so the asset files are identical between desktop and embedded.
 *
 * ---------------------------------------------------------------------------
 * Controls (keyboard):
 *   Left/Right -- heading          Up/Down -- course (OBS)
 *   Q/W        -- heading bug      A/D     -- CDI deviation
 *   SPACE      -- toggle auto-hdg  TAB     -- toggle auto-cdi
 *   R          -- reset all        ESC     -- quit
 *
 * Controls (mouse):
 *   Click or hold the arrow buttons in the bottom panel.
 * ---------------------------------------------------------------------------
 */

/* --- Standard library and SDL2 includes --- */
#include <SDL2/SDL.h>   /* SDL2 windowing, rendering, input, timing        */
#include <stdio.h>      /* printf, fprintf, snprintf, FILE I/O             */
#include <stdlib.h>     /* malloc, calloc, free                            */
#include <stdint.h>     /* uint8_t, uint16_t, uint32_t fixed-width types   */
#include <string.h>     /* strlen for string measurement in font rendering */
#include <math.h>       /* sinf, cosf, fmodf, fabsf, M_PI                 */

/* ================================================================== */
/*  Display geometry constants                                         */
/* ================================================================== */
/*
 * The window is divided into two regions:
 *   Top:    480x480 pixel instrument face (the HSI itself)
 *   Bottom: 480x110 pixel control panel (buttons, labels, values)
 *
 * CX, CY define the center of rotation for all instrument layers.
 * Every rotating element (compass card, course pointer, CDI bar, etc.)
 * rotates around this center point, which is the physical center of
 * the round HSI instrument face.
 */
#define DISP_W    480       /* instrument face width/height (square) */
#define DISP_H    480
#define PANEL_H   110       /* control panel height below instrument */
#define WIN_W     DISP_W    /* total window width = instrument width */
#define WIN_H     (DISP_H + PANEL_H)   /* total window height = 590 px */
#define CX        (DISP_W / 2)  /* center of rotation X = 240 */
#define CY        (DISP_H / 2)  /* center of rotation Y = 240 */

/* ================================================================== */
/*  Panel layout constants                                             */
/* ================================================================== */
/*
 * The control panel is divided into N_CONTROLS groups (4), each
 * occupying GROUP_W pixels horizontally (120 px each for 480 px total).
 * Each group has:
 *   - A text label at the top (e.g., "HDG", "CRS", "DEV", "BUG")
 *   - A numeric value display in the middle
 *   - Left (<) and right (>) arrow buttons at the bottom
 *
 * Button positions are computed by macros BTN_L_X(g) and BTN_R_X(g)
 * which place the left button 6 pixels from the left edge of the group
 * and the right button 6 pixels from the right edge.
 */
#define N_CONTROLS  4                        /* HDG, CRS, DEV, BUG       */
#define GROUP_W     (WIN_W / N_CONTROLS)     /* 120 px per control group  */
#define BTN_W       32                       /* button width in pixels    */
#define BTN_H       28                       /* button height in pixels   */
#define BTN_Y       (DISP_H + 58)           /* Y coordinate of button row */
#define LABEL_Y     (DISP_H + 14)           /* Y coordinate of label text */
#define VALUE_Y     (DISP_H + 38)           /* Y coordinate of value text */
/* Left button X within group g: 6px inset from group left edge */
#define BTN_L_X(g)  ((g) * GROUP_W + 6)
/* Right button X within group g: 6px inset from group right edge */
#define BTN_R_X(g)  ((g) * GROUP_W + GROUP_W - 6 - BTN_W)

/* ================================================================== */
/*  Panel color constants (ARGB8888 format, 0xAARRGGBB)                */
/* ================================================================== */
/*
 * These colors give the control panel a dark avionics-style appearance.
 * COL_VALUE uses green (0x44DD44) to mimic the look of green CRT or
 * LED avionics displays commonly found in aircraft instrument panels.
 */
#define COL_PANEL_BG    0xFF1A1A1Au  /* very dark gray panel background */
#define COL_SEPARATOR   0xFF333333u  /* slightly lighter separator lines */
#define COL_BTN_NORMAL  0xFF3A3A3Au  /* button idle state               */
#define COL_BTN_HOVER   0xFF505050u  /* button when mouse hovers over   */
#define COL_BTN_PRESS   0xFF686868u  /* button when clicked/held         */
#define COL_ARROW       0xFFDDDDDDu  /* arrow glyph color (light gray)  */
#define COL_LABEL       0xFF888888u  /* label text color (medium gray)  */
#define COL_VALUE       0xFF44DD44u  /* value text color — green avionics look */

/* ================================================================== */
/*  5x7 bitmap font                                                    */
/* ================================================================== */
/*
 * A minimal hand-coded bitmap font for rendering text in the control
 * panel.  Each glyph is 5 pixels wide and 7 pixels tall.  The font is
 * rendered at FONT_SCALE (2x), so each character occupies 12x14 pixels
 * on screen (including a 1-pixel gap between characters, also scaled).
 *
 * Only the characters actually needed for the panel displays are
 * included: digits 0-9, a few letters (for "HDG", "CRS", "DEV", "BUG"),
 * plus '+', '-', '.', and space.
 *
 * Each glyph row is stored as a uint8_t bitmask where bits 4..0
 * represent pixels left-to-right.  For example, 0x0E = 01110 in binary,
 * which lights up the middle three pixels.
 */
#define FONT_W  5                            /* glyph width in pixels     */
#define FONT_H  7                            /* glyph height in pixels    */
#define FONT_SCALE  2                        /* 2x magnification          */
#define CHAR_W  ((FONT_W + 1) * FONT_SCALE)  /* 12 px total per char cell */
#define CHAR_H  (FONT_H * FONT_SCALE)        /* 14 px total char height   */

/* Each glyph: 7 rows, bits 4..0 = pixels left to right */
static const struct { char ch; uint8_t r[7]; } glyphs[] = {
    /* ---- Digits ---- */
    {'0', {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}},  /* classic 0 with diagonal stroke */
    {'1', {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}},
    {'2', {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F}},
    {'3', {0x1F,0x02,0x04,0x02,0x01,0x11,0x0E}},
    {'4', {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}},
    {'5', {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E}},
    {'6', {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}},
    {'7', {0x1F,0x01,0x02,0x04,0x08,0x08,0x08}},
    {'8', {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}},
    {'9', {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C}},
    /* ---- Letters (only those needed for labels) ---- */
    {'A', {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}},
    {'B', {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}},
    {'C', {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}},
    {'D', {0x1C,0x12,0x11,0x11,0x11,0x12,0x1C}},
    {'E', {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}},
    {'G', {0x0E,0x11,0x10,0x17,0x11,0x11,0x0E}},
    {'H', {0x11,0x11,0x11,0x1F,0x11,0x11,0x11}},
    {'R', {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}},
    {'S', {0x0E,0x11,0x10,0x0E,0x01,0x11,0x0E}},
    {'U', {0x11,0x11,0x11,0x11,0x11,0x11,0x0E}},
    {'V', {0x11,0x11,0x11,0x11,0x0A,0x0A,0x04}},
    /* ---- Punctuation and whitespace ---- */
    {'+', {0x00,0x04,0x04,0x1F,0x04,0x04,0x00}},
    {'-', {0x00,0x00,0x00,0x1F,0x00,0x00,0x00}},
    {'.', {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C}},
    {' ', {0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
};
#define N_GLYPHS ((int)(sizeof(glyphs)/sizeof(glyphs[0])))

/* ------------------------------------------------------------------ */
/*  draw_char — render a single character to the framebuffer           */
/* ------------------------------------------------------------------ */
/*
 * Looks up the character 'c' in the glyph table and draws it at pixel
 * position (x, y) in the given color.  Each "on" bit in the glyph
 * bitmap is expanded to a FONT_SCALE x FONT_SCALE block of pixels.
 *
 * The bit test (0x10 >> col) walks the 5-bit row from MSB to LSB,
 * mapping bit 4 to the leftmost column and bit 0 to the rightmost.
 *
 * Bounds checking via (unsigned) cast: if px or py is negative, the
 * unsigned cast makes it a huge positive number that fails the < test.
 * This is a common embedded trick to avoid two comparisons per pixel.
 */
static void draw_char(uint32_t *fb, int x, int y, char c, uint32_t color)
{
    const uint8_t *bits = NULL;
    /* Linear search through glyph table — fast enough for ~25 entries */
    for (int i = 0; i < N_GLYPHS; i++)
        if (glyphs[i].ch == c) { bits = glyphs[i].r; break; }
    if (!bits) return;  /* character not in font — skip silently */

    /* Walk each pixel of the 5x7 glyph */
    for (int row = 0; row < FONT_H; row++)
        for (int col = 0; col < FONT_W; col++)
            if (bits[row] & (0x10 >> col))  /* is this pixel "on"? */
                /* Expand each glyph pixel to a FONT_SCALE x FONT_SCALE block */
                for (int sy = 0; sy < FONT_SCALE; sy++)
                    for (int sx = 0; sx < FONT_SCALE; sx++) {
                        int px = x + col * FONT_SCALE + sx;
                        int py = y + row * FONT_SCALE + sy;
                        /* Bounds check using unsigned cast trick */
                        if ((unsigned)px < WIN_W && (unsigned)py < WIN_H)
                            fb[py * WIN_W + px] = color;
                    }
}

/* ------------------------------------------------------------------ */
/*  draw_str — render a null-terminated string, left-aligned           */
/* ------------------------------------------------------------------ */
static void draw_str(uint32_t *fb, int x, int y, const char *s, uint32_t color)
{
    while (*s) { draw_char(fb, x, y, *s++, color); x += CHAR_W; }
}

/* ------------------------------------------------------------------ */
/*  draw_str_cx — render a string centered horizontally at center_x    */
/* ------------------------------------------------------------------ */
/* Draw string centered horizontally in a range */
static void draw_str_cx(uint32_t *fb, int center_x, int y,
                         const char *s, uint32_t color)
{
    int len = (int)strlen(s);
    /* Total pixel width: each char is CHAR_W, but the last char doesn't
     * need the trailing gap, so subtract FONT_SCALE (the scaled gap). */
    int total_w = len * CHAR_W - (FONT_SCALE); /* subtract trailing gap */
    draw_str(fb, center_x - total_w / 2, y, s, color);
}

/* ================================================================== */
/*  GUI button drawing helpers                                         */
/* ================================================================== */

/* ------------------------------------------------------------------ */
/*  fill_rect — fill a solid rectangle into the framebuffer            */
/* ------------------------------------------------------------------ */
/* Fill a rect */
static void fill_rect(uint32_t *fb, int rx, int ry, int rw, int rh, uint32_t c)
{
    for (int y = ry; y < ry + rh; y++) {
        if ((unsigned)y >= WIN_H) continue;     /* clip top/bottom */
        for (int x = rx; x < rx + rw; x++)
            if ((unsigned)x < WIN_W)             /* clip left/right */
                fb[y * WIN_W + x] = c;
    }
}

/* ------------------------------------------------------------------ */
/*  draw_arrow — draw a filled triangular arrow inside a button rect   */
/* ------------------------------------------------------------------ */
/*
 * Draws a left-pointing (dir=-1) or right-pointing (dir=+1) triangle
 * inside the button rectangle defined by (bx, by, bw, bh).
 *
 * The triangle is inset by 'pad' pixels from each edge.  It is drawn
 * scan-line by scan-line: for each row, the width of the triangle at
 * that row is computed proportionally to the distance from the center
 * row (the tip), creating a filled isoceles triangle.
 */
/* Draw a filled triangle arrow inside a button rect */
static void draw_arrow(uint32_t *fb, int bx, int by, int bw, int bh,
                        int dir, uint32_t color)
{
    /* dir: -1 = left-pointing (<), +1 = right-pointing (>) */
    int pad = 9;               /* padding from button edges           */
    int th = bh - 2 * pad;     /* triangle height (vertical extent)  */
    int tw = bw - 2 * pad;     /* triangle base width (horizontal)   */
    if (th < 2 || tw < 2) return;  /* too small to draw */
    int half = th / 2;         /* half-height = distance from center to tip */

    /* Draw the triangle row by row */
    for (int row = 0; row < th; row++) {
        int dist = abs(row - half);  /* distance from center row */
        /* Width of triangle at this row: widest at center, 1px at tips */
        int w = tw * (half - dist) / half;
        if (w < 1) w = 1;
        int y = by + pad + row;
        int x0;
        if (dir < 0)        /* <  base on right, tip on left */
            x0 = bx + bw - pad - w;
        else                /* >  base on left, tip on right */
            x0 = bx + pad;
        for (int c = 0; c < w; c++)
            if ((unsigned)(x0 + c) < WIN_W && (unsigned)y < WIN_H)
                fb[y * WIN_W + x0 + c] = color;
    }
}

/* ------------------------------------------------------------------ */
/*  draw_button — draw a complete button (background + arrow)          */
/* ------------------------------------------------------------------ */
/*
 * Renders a button with visual feedback:
 *   - Normal state:  dark gray background
 *   - Hovered:       slightly lighter (mouse is over the button)
 *   - Pressed:       even lighter (mouse button is held down)
 * The arrow direction (left or right) indicates whether the button
 * decreases or increases the associated value.
 */
/* Draw a single button (rect + arrow) and return its hit state */
static void draw_button(uint32_t *fb, int bx, int by, int dir,
                         int hovered, int pressed)
{
    uint32_t bg = pressed ? COL_BTN_PRESS : hovered ? COL_BTN_HOVER : COL_BTN_NORMAL;
    fill_rect(fb, bx, by, BTN_W, BTN_H, bg);
    draw_arrow(fb, bx, by, BTN_W, BTN_H, dir, COL_ARROW);
}

/* ================================================================== */
/*  ARGB1555 sprite structure and file loading                         */
/* ================================================================== */
/*
 * Sprites are used for instrument elements that need color and have
 * irregular shapes (course pointer, CDI bar, heading bug, lubber line).
 *
 * Each sprite file has an 8-byte header:
 *   - uint16_t ox: X origin offset (top-left corner X in display coords)
 *   - uint16_t oy: Y origin offset (top-left corner Y in display coords)
 *   - uint16_t w:  sprite width in pixels
 *   - uint16_t h:  sprite height in pixels
 *
 * Followed by w*h pixels in ARGB1555 format (2 bytes per pixel).
 *
 * The origin offset (ox, oy) positions the sprite relative to the
 * display origin (0,0 = top-left of instrument face).  This means
 * the sprite's artwork is pre-positioned in its asset file — e.g.,
 * the course pointer is vertically centered and its ox,oy place it
 * correctly when drawn at 0 degrees rotation.
 */
typedef struct {
    int ox, oy, w, h;  /* origin offset and dimensions */
    uint16_t *px;       /* pixel data in ARGB1555 format */
} Sprite;

/* ------------------------------------------------------------------ */
/*  load_bin — load a raw binary file of known size into memory        */
/* ------------------------------------------------------------------ */
/*
 * Used for loading the full-frame AL44 and L8 layers, where the file
 * is exactly DISP_W * DISP_H bytes (480*480 = 230,400 bytes).
 * Returns NULL on any error (file not found, wrong size, alloc fail).
 */
static void *load_bin(const char *path, size_t expected)
{
    FILE *f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "Cannot open %s\n", path); return NULL; }
    void *buf = malloc(expected);
    if (!buf) { fclose(f); return NULL; }
    size_t got = fread(buf, 1, expected, f);
    fclose(f);
    if (got != expected) {
        fprintf(stderr, "%s: expected %zu, got %zu\n", path, expected, got);
        free(buf); return NULL;
    }
    return buf;
}

/* ------------------------------------------------------------------ */
/*  load_sprite — load an ARGB1555 sprite file (header + pixel data)   */
/* ------------------------------------------------------------------ */
/*
 * Reads the 8-byte header (ox, oy, w, h as uint16_t), then allocates
 * and reads the pixel array.  The sprite is ready for blitting or
 * rotation immediately after loading.
 */
static int load_sprite(const char *path, Sprite *s)
{
    FILE *f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "Cannot open %s\n", path); return -1; }
    /* Read 4 uint16_t values: ox, oy, w, h */
    uint16_t hdr[4];
    if (fread(hdr, 2, 4, f) != 4) { fclose(f); return -1; }
    s->ox = hdr[0]; s->oy = hdr[1]; s->w = hdr[2]; s->h = hdr[3];
    /* Allocate pixel buffer and read ARGB1555 pixel data */
    size_t npx = (size_t)s->w * s->h;
    s->px = malloc(npx * 2);   /* 2 bytes per ARGB1555 pixel */
    if (!s->px) { fclose(f); return -1; }
    if (fread(s->px, 2, npx, f) != npx) {
        fprintf(stderr, "%s: short pixel read\n", path);
        free(s->px); fclose(f); return -1;
    }
    fclose(f);
    printf("  %s: offset(%d,%d) size(%dx%d)\n", path, s->ox, s->oy, s->w, s->h);
    return 0;
}

/* ================================================================== */
/*  Instrument compositing functions                                   */
/* ================================================================== */
/*
 * These functions composite the instrument layers onto a 32-bit ARGB8888
 * framebuffer (the SDL texture backing store).  The framebuffer is
 * WIN_W * WIN_H pixels, but instrument layers only write to the top
 * DISP_W * DISP_H region.
 *
 * ALPHA BLENDING:
 * blend_pixel() implements standard "over" compositing:
 *   result = src * alpha + dst * (1 - alpha)
 * This is applied per-channel (R, G, B) with alpha in the range 0-255.
 * Fast paths exist for alpha=0 (skip entirely) and alpha=255 (overwrite).
 *
 * INVERSE ROTATION (used by rotate_blend_al44 and rotate_blend_sprite):
 * Instead of rotating each source pixel forward to find where it lands
 * in the destination (which leaves gaps due to rounding), we iterate
 * over DESTINATION pixels and rotate BACKWARDS to find the corresponding
 * source pixel.  This guarantees every destination pixel gets a value
 * with no gaps.
 *
 * The math for inverse rotation by angle theta around center (CX, CY):
 *   Given destination pixel (dx, dy), compute offset from center:
 *     fx = dx - CX,   fy = dy - CY
 *   Rotate backwards (negate the angle, or equivalently transpose the
 *   rotation matrix):
 *     sx = CX + fx*cos(theta) + fy*sin(theta)
 *     sy = CY - fx*sin(theta) + fy*cos(theta)
 *   This gives the source pixel coordinates (sx, sy) to sample from.
 *
 *   Why this is the INVERSE rotation:
 *   The forward rotation matrix for angle theta is:
 *     [ cos(theta)  -sin(theta) ]
 *     [ sin(theta)   cos(theta) ]
 *   The inverse (backward) rotation is the transpose:
 *     [  cos(theta)  sin(theta) ]
 *     [ -sin(theta)  cos(theta) ]
 *   which is exactly what we apply to (fx, fy) to get (sx, sy).
 *
 * Nearest-neighbor sampling is used (no bilinear interpolation) to
 * match the pixel-exact look of the real STM32 LTDC hardware, which
 * also does not interpolate.  The +0.5f in the rounding ensures
 * correct rounding to the nearest source pixel.
 */

/* ------------------------------------------------------------------ */
/*  blend_pixel — alpha-blend a single RGBA pixel onto a destination   */
/* ------------------------------------------------------------------ */
/*
 * dst:  pointer to the destination pixel in ARGB8888 format
 * r,g,b: source color channels (0-255)
 * a:     source alpha (0=transparent, 255=opaque)
 *
 * Uses integer-only blending (no floating point) for speed.
 * The division by 255 is exact for the "over" operator.
 */
static inline void blend_pixel(uint32_t *dst,
                               uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    if (a == 0) return;     /* fully transparent — nothing to do */
    if (a == 255) {         /* fully opaque — just overwrite, skip blend math */
        *dst = 0xFF000000u | ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
        return;
    }
    /* Partial transparency: blend src over dst */
    uint32_t d = *dst;
    uint8_t dr = (d >> 16) & 0xFF;   /* extract destination red   */
    uint8_t dg = (d >>  8) & 0xFF;   /* extract destination green */
    uint8_t db =  d        & 0xFF;   /* extract destination blue  */
    /* Standard "over" blend: out = src*a + dst*(1-a), with a in [0,255] */
    dr = (uint8_t)((r * a + dr * (255 - a)) / 255);
    dg = (uint8_t)((g * a + dg * (255 - a)) / 255);
    db = (uint8_t)((b * a + db * (255 - a)) / 255);
    *dst = 0xFF000000u | ((uint32_t)dr << 16) | ((uint32_t)dg << 8) | db;
}

/* ------------------------------------------------------------------ */
/*  blit_background — Layer 1: static L8 grayscale background          */
/* ------------------------------------------------------------------ */
/*
 * Copies the L8 (8-bit luminance) background directly to the framebuffer
 * with no rotation and no alpha blending.  Each grayscale byte is
 * expanded to ARGB8888 by replicating the luminance into R, G, and B
 * channels (R=G=B=L), with alpha set to 0xFF (fully opaque).
 *
 * This is the first layer painted, so it completely fills the instrument
 * area and serves as the backdrop for all subsequent layers.
 */
/* Layer 1 -- Background (L8, 480x480, static) */
static void blit_background(uint32_t *fb, const uint8_t *bg)
{
    for (int i = 0; i < DISP_W * DISP_H; i++) {
        uint8_t l = bg[i];  /* 8-bit luminance value */
        /* Pack into ARGB8888: alpha=FF, R=G=B=luminance */
        fb[i] = 0xFF000000u | ((uint32_t)l << 16) | ((uint32_t)l << 8) | l;
    }
}

/* ------------------------------------------------------------------ */
/*  rotate_blend_al44 — rotate and blend a full-frame AL44 layer       */
/* ------------------------------------------------------------------ */
/*
 * Used for Layer 2 (CDI dots) and Layer 3 (compass card).
 *
 * Rotates the 480x480 AL44 source image by 'deg' degrees around the
 * display center (CX, CY) and alpha-blends it onto the framebuffer.
 *
 * AL44 pixel decoding:
 *   High nibble (bits 7-4) = 4-bit alpha (0-15)
 *   Low nibble  (bits 3-0) = 4-bit luminance (0-15)
 *   Both are scaled to 8-bit by multiplying by 17 (which maps 0->0,
 *   1->17, 2->34, ..., 15->255 — a perfect linear scale).
 *   The magic of 17 (0x11): 15 * 17 = 255 exactly.
 *
 * INVERSE ROTATION is used here: for every destination pixel (dx, dy),
 * we compute the source pixel (sx, sy) by rotating backwards.  This
 * avoids gaps that would appear if we rotated forward.  See the
 * detailed math explanation in the block comment above.
 *
 * This function iterates over ALL 480x480 destination pixels because
 * the AL44 layers are full-frame (they cover the entire instrument face).
 * Transparent pixels (alpha nibble = 0) are skipped early for speed.
 */
/* Rotate + blend a full 480x480 AL44 layer around display center */
static void rotate_blend_al44(uint32_t *fb, const uint8_t *src, float deg)
{
    /* Convert degrees to radians for trig functions */
    float rad = deg * (float)M_PI / 180.0f;
    float cs = cosf(rad), sn = sinf(rad);  /* precompute sin/cos once */

    /* Iterate over every destination pixel in the instrument face */
    for (int dy = 0; dy < DISP_H; dy++) {
        float fy = (float)(dy - CY);   /* Y offset from center of rotation */
        for (int dx = 0; dx < DISP_W; dx++) {
            float fx = (float)(dx - CX);  /* X offset from center of rotation */
            /*
             * INVERSE ROTATION: map destination pixel back to source coordinates.
             * This is the transpose of the forward rotation matrix:
             *   sx = CX + fx*cos + fy*sin
             *   sy = CY - fx*sin + fy*cos
             * The +0.5f provides rounding to nearest integer (nearest-neighbor).
             */
            int sx = (int)(CX + fx * cs + fy * sn + 0.5f);
            int sy = (int)(CY - fx * sn + fy * cs + 0.5f);
            /* If the source pixel falls outside the image, skip it */
            if ((unsigned)sx >= DISP_W || (unsigned)sy >= DISP_H) continue;

            /* Decode AL44 pixel */
            uint8_t p = src[sy * DISP_W + sx];
            uint8_t a4 = p >> 4;           /* 4-bit alpha (0-15) */
            if (a4 == 0) continue;         /* fully transparent — skip early */
            uint8_t l = (p & 0x0F) * 17;   /* 4-bit luminance -> 8-bit (0-255) */
            uint8_t a = a4 * 17;           /* 4-bit alpha -> 8-bit (0-255) */
            /* Blend this grayscale pixel (R=G=B=luminance) onto the framebuffer */
            blend_pixel(&fb[dy * WIN_W + dx], l, l, l, a);
        }
    }
}

/* ================================================================== */
/*  rotate_blend_sprite — the HEART of the HSI rendering               */
/* ================================================================== */
/*
 * Rotates and blends an ARGB1555 sprite around the display center,
 * with an optional pre-rotation translation (tx, ty).
 *
 * THIS IS THE CRITICAL FUNCTION FOR CDI BAR RENDERING.
 *
 * ---------------------------------------------------------------------------
 * CDI BAR MECHANICS — HOW ROTATION + TRANSLATION WORK TOGETHER
 * ---------------------------------------------------------------------------
 *
 * The CDI bar must simultaneously:
 *   1. ROTATE with the course pointer (so it stays aligned with the
 *      selected course direction), and
 *   2. TRANSLATE laterally (perpendicular to the course pointer axis)
 *      to show how far off-course the aircraft is.
 *
 * The key insight is the ORDER OF OPERATIONS:
 *   - The translation (tx, ty) is applied in the SPRITE'S LOCAL
 *     coordinate system BEFORE rotation.
 *   - Then the entire translated sprite is rotated around the display
 *     center.
 *
 * For the CDI bar, tx = cdi_dev (the lateral deviation in pixels) and
 * ty = 0.  Since the CDI bar sprite is authored as a VERTICAL bar
 * (pointing straight up at 0 degrees), a horizontal tx offset moves it
 * LEFT or RIGHT in the sprite's own frame.  When the whole assembly
 * is then rotated by the course angle, that left/right offset becomes
 * perpendicular-to-course in screen space.
 *
 * Example: if course = 90 degrees (east), the course pointer points
 * right on screen.  The CDI bar (originally vertical) rotates 90 degrees
 * to become horizontal.  The tx offset that was "left/right" in sprite
 * space becomes "up/down" in screen space — which is exactly perpendicular
 * to the east-pointing course pointer.  The CDI bar slides up and down
 * (perpendicular to the pointer) as deviation changes.
 *
 * ---------------------------------------------------------------------------
 * INVERSE ROTATION MATH WITH TRANSLATION
 * ---------------------------------------------------------------------------
 *
 * For each destination pixel (dx, dy), we need to find the corresponding
 * source pixel in the UN-rotated, UN-translated sprite.
 *
 * Step 1: Compute offset from display center:
 *     fx = dx - CX,    fy = dy - CY
 *
 * Step 2: Apply INVERSE rotation (rotate backwards by -deg):
 *     rx = CX + fx*cos(deg) + fy*sin(deg)
 *     ry = CY - fx*sin(deg) + fy*cos(deg)
 *   This gives us the point in the un-rotated coordinate system.
 *
 * Step 3: Undo the pre-rotation translation:
 *     sx_f = rx - tx
 *     sy_f = ry - ty
 *   This gives us the point in the original sprite coordinate system,
 *   before any translation was applied.
 *
 * Step 4: Convert from display coordinates to sprite-local coordinates:
 *     lx = sx_f - sprite.ox    (subtract sprite origin X)
 *     ly = sy_f - sprite.oy    (subtract sprite origin Y)
 *
 * Step 5: If (lx, ly) is within the sprite bounds, sample and blend
 *   that pixel.
 *
 * ---------------------------------------------------------------------------
 * BOUNDING BOX OPTIMIZATION
 * ---------------------------------------------------------------------------
 *
 * Instead of iterating over all 480x480 destination pixels (230,400 pixels),
 * we compute a tight axis-aligned bounding box (AABB) of the rotated sprite
 * in screen space.  This is done by:
 *   1. Taking the 4 corners of the sprite rectangle (in its local space,
 *      AFTER applying the translation offset).
 *   2. Rotating each corner into screen space using the FORWARD rotation.
 *   3. Finding the min/max X and Y of the rotated corners.
 *   4. Adding a 2-pixel margin for rounding safety.
 *   5. Clamping to the display bounds.
 *
 * For a small sprite like the heading bug (maybe 40x20 pixels), this
 * reduces the inner loop from 230,400 iterations to perhaps 3,000 —
 * a ~77x speedup.  Even for the full-length course pointer, the bounding
 * box is much smaller than the full frame.
 *
 * ---------------------------------------------------------------------------
 * ARGB1555 PIXEL DECODING
 * ---------------------------------------------------------------------------
 *
 * Each pixel is a uint16_t:
 *   Bit 15:    alpha flag (1 = opaque, 0 = fully transparent)
 *   Bits 14-10: red   (5 bits, 0-31)
 *   Bits 9-5:   green (5 bits, 0-31)
 *   Bits 4-0:   blue  (5 bits, 0-31)
 *
 * The 1-bit alpha means pixels are either fully visible or fully
 * invisible — there is no partial transparency.  This is fine for
 * hard-edged instrument graphics.  The check (p & 0x8000) tests the
 * alpha bit; if clear, the pixel is skipped entirely.
 *
 * The 5-bit color channels are expanded to 8-bit by shifting left 3,
 * which maps 0->0, 1->8, ..., 31->248.  (The bottom 3 bits are left
 * as zero; a more accurate expansion would be (val << 3) | (val >> 2)
 * to map 31->255, but the simpler shift is close enough and faster.)
 *
 * Parameters:
 *   fb   — destination framebuffer (ARGB8888, WIN_W * WIN_H)
 *   s    — pointer to the Sprite structure (has ox, oy, w, h, px)
 *   deg  — rotation angle in degrees (positive = clockwise on screen)
 *   tx   — pre-rotation X translation in pixels (CDI lateral offset)
 *   ty   — pre-rotation Y translation in pixels (unused for CDI, 0)
 */
/* Rotate + blend an ARGB1555 sprite around display center */
static void rotate_blend_sprite(uint32_t *fb, const Sprite *s,
                                float deg, float tx, float ty)
{
    /* Convert rotation angle to radians and precompute sin/cos */
    float rad = deg * (float)M_PI / 180.0f;
    float cs = cosf(rad), sn = sinf(rad);

    /*
     * BOUNDING BOX COMPUTATION:
     * Compute the 4 corners of the sprite's rectangle in display-space
     * coordinates, AFTER applying the pre-rotation translation (tx, ty).
     * These corners are in the "un-rotated display" coordinate system,
     * relative to (0, 0) at the display top-left.
     *
     * corner[i] = (sprite.ox + tx, sprite.oy + ty) for the top-left,
     * and similarly for the other three corners offset by (w, h).
     */
    float corners[4][2] = {
        { s->ox + tx,            s->oy + ty            },  /* top-left     */
        { s->ox + s->w + tx,     s->oy + ty            },  /* top-right    */
        { s->ox + tx,            s->oy + s->h + ty     },  /* bottom-left  */
        { s->ox + s->w + tx,     s->oy + s->h + ty     }   /* bottom-right */
    };

    /*
     * Rotate each corner into screen space using the FORWARD rotation
     * matrix to find where the sprite will appear after rotation.
     *
     * Forward rotation about (CX, CY) by angle theta:
     *   screen_x = CX + (fx * cos - fy * sin)
     *   screen_y = CY + (fx * sin + fy * cos)
     * where fx = corner_x - CX, fy = corner_y - CY.
     *
     * We track the min/max of all four rotated corners to get the AABB.
     */
    int min_x = DISP_W, min_y = DISP_H, max_x = 0, max_y = 0;
    for (int i = 0; i < 4; i++) {
        float fx = corners[i][0] - CX;    /* offset from rotation center */
        float fy = corners[i][1] - CY;
        int rx = (int)(CX + fx * cs - fy * sn);  /* forward rotation X */
        int ry = (int)(CY + fx * sn + fy * cs);  /* forward rotation Y */
        if (rx < min_x) min_x = rx;
        if (rx > max_x) max_x = rx;
        if (ry < min_y) min_y = ry;
        if (ry > max_y) max_y = ry;
    }
    /* Add a 2-pixel safety margin to account for floating-point rounding
     * errors at the edges of the bounding box.  Without this margin,
     * edge pixels could occasionally be clipped. */
    if (min_x < 0) min_x = 0;        else if (min_x > 2) min_x -= 2;
    if (min_y < 0) min_y = 0;        else if (min_y > 2) min_y -= 2;
    if (max_x >= DISP_W) max_x = DISP_W - 1; else max_x += 2;
    if (max_y >= DISP_H) max_y = DISP_H - 1; else max_y += 2;
    /* Double-clamp in case the +2 pushed us out of bounds */
    if (max_x >= DISP_W) max_x = DISP_W - 1;
    if (max_y >= DISP_H) max_y = DISP_H - 1;

    /*
     * MAIN RENDERING LOOP:
     * Only iterate over the bounding box region, not the entire display.
     * For each destination pixel, apply inverse rotation + inverse
     * translation to find the source pixel in the sprite.
     */
    for (int dy = min_y; dy <= max_y; dy++) {
        float fy = (float)(dy - CY);  /* destination Y offset from center */
        for (int dx = min_x; dx <= max_x; dx++) {
            float fx = (float)(dx - CX);  /* destination X offset from center */

            /*
             * INVERSE ROTATION then INVERSE TRANSLATION:
             *
             * Step 1 — Inverse rotation (same as rotate_blend_al44):
             *   un_rotated_x = CX + fx*cos + fy*sin
             *   un_rotated_y = CY - fx*sin + fy*cos
             *
             * Step 2 — Subtract the pre-rotation translation:
             *   sx_f = un_rotated_x - tx
             *   sy_f = un_rotated_y - ty
             *
             * This maps the screen pixel (dx, dy) back to the sprite's
             * original coordinate system (before translation and rotation).
             */
            float sx_f = CX + fx * cs + fy * sn - tx;
            float sy_f = CY - fx * sn + fy * cs - ty;

            /*
             * Convert from display coordinates to sprite-local coordinates:
             * The sprite's top-left corner is at (ox, oy) in display space,
             * so local_x = display_x - ox, local_y = display_y - oy.
             * +0.5f for nearest-neighbor rounding.
             */
            int lx = (int)(sx_f - s->ox + 0.5f);
            int ly = (int)(sy_f - s->oy + 0.5f);

            /* Bounds check: is this point inside the sprite rectangle?
             * Using unsigned cast trick: if lx < 0, (unsigned)lx wraps
             * to a huge value that fails the >= test. */
            if ((unsigned)lx >= (unsigned)s->w ||
                (unsigned)ly >= (unsigned)s->h) continue;

            /* Sample the ARGB1555 pixel from the sprite */
            uint16_t p = s->px[ly * s->w + lx];

            /* Check the 1-bit alpha (bit 15): 0 = transparent, skip */
            if (!(p & 0x8000)) continue;

            /* Decode 5-bit color channels and expand to 8-bit */
            uint8_t r = ((p >> 10) & 0x1F) << 3;  /* red:   bits 14-10, scaled */
            uint8_t g = ((p >>  5) & 0x1F) << 3;  /* green: bits 9-5, scaled   */
            uint8_t b = ( p        & 0x1F) << 3;  /* blue:  bits 4-0, scaled   */

            /* Write the pixel (no blending needed — ARGB1555 alpha is binary,
             * so visible pixels are always fully opaque). */
            fb[dy * WIN_W + dx] = 0xFF000000u |
                ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  blit_sprite — draw an ARGB1555 sprite with NO rotation (static)    */
/* ------------------------------------------------------------------ */
/*
 * Used for Layer 7 (lubber / aircraft symbol), which is always fixed
 * at the top of the display (it represents the aircraft's nose and
 * does not rotate).
 *
 * This is much simpler than rotate_blend_sprite — no trig, no inverse
 * rotation, just a direct copy from sprite-local to display coordinates
 * using the sprite's (ox, oy) offset.
 */
/* Blit ARGB1555 sprite with no rotation (static overlay) */
static void blit_sprite(uint32_t *fb, const Sprite *s)
{
    for (int y = 0; y < s->h; y++) {
        int dy = s->oy + y;  /* display Y = sprite origin Y + local Y */
        if ((unsigned)dy >= DISP_H) continue;  /* clip vertically */
        for (int x = 0; x < s->w; x++) {
            int dx = s->ox + x;  /* display X = sprite origin X + local X */
            if ((unsigned)dx >= DISP_W) continue;  /* clip horizontally */
            uint16_t p = s->px[y * s->w + x];
            if (!(p & 0x8000)) continue;  /* transparent pixel, skip */
            /* Decode ARGB1555 to ARGB8888 (same as in rotate_blend_sprite) */
            uint8_t r = ((p >> 10) & 0x1F) << 3;
            uint8_t g = ((p >>  5) & 0x1F) << 3;
            uint8_t b = ( p        & 0x1F) << 3;
            fb[dy * WIN_W + dx] = 0xFF000000u |
                ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
        }
    }
}

/* ================================================================== */
/*  Button hit testing                                                 */
/* ================================================================== */
/*
 * The GUI has 8 buttons total (2 per control group, 4 groups):
 *   ID 0 = HDG left (<)     ID 1 = HDG right (>)
 *   ID 2 = CRS left (<)     ID 3 = CRS right (>)
 *   ID 4 = DEV left (<)     ID 5 = DEV right (>)
 *   ID 6 = BUG left (<)     ID 7 = BUG right (>)
 *
 * get_btn_rect() computes the screen rectangle for a given button ID.
 * hit_test() checks all buttons against a mouse coordinate and returns
 * the ID of the button under the cursor, or -1 if none.
 */

/* Button IDs: 0=HDG_L 1=HDG_R 2=CRS_L 3=CRS_R
 *             4=DEV_L 5=DEV_R 6=BUG_L 7=BUG_R */
#define N_BUTTONS (N_CONTROLS * 2)

/* Compute the pixel rectangle of button 'id' */
static void get_btn_rect(int id, int *bx, int *by, int *bw, int *bh)
{
    int group = id / 2;       /* which control group (0-3) */
    int is_right = id & 1;    /* 0 = left button, 1 = right button */
    *bx = is_right ? BTN_R_X(group) : BTN_L_X(group);
    *by = BTN_Y;
    *bw = BTN_W;
    *bh = BTN_H;
}

/* Test if pixel (mx, my) is inside any button; return button ID or -1 */
static int hit_test(int mx, int my)
{
    for (int i = 0; i < N_BUTTONS; i++) {
        int bx, by, bw, bh;
        get_btn_rect(i, &bx, &by, &bw, &bh);
        if (mx >= bx && mx < bx + bw && my >= by && my < by + bh)
            return i;
    }
    return -1;  /* no button hit */
}

/* ================================================================== */
/*  Main — application entry point                                     */
/* ================================================================== */
/*
 * The main function:
 *   1. Loads all binary asset files (background, compass card, CDI dots,
 *      course pointer, CDI bar, heading bug, lubber/aircraft).
 *   2. Creates the SDL window, renderer, and streaming texture.
 *   3. Enters the main loop, which on every frame:
 *      a. Processes SDL events (keyboard, mouse)
 *      b. Updates instrument state (heading, course, deviation, etc.)
 *      c. Composites all 7 layers into the framebuffer
 *      d. Renders the GUI control panel
 *      e. Uploads the framebuffer to SDL and presents it
 *   4. On exit, frees all resources.
 */
int main(int argc, char *argv[])
{
    (void)argc; (void)argv;  /* suppress unused parameter warnings */

    /* ================================================================ */
    /*  ASSET LOADING                                                    */
    /* ================================================================ */
    /*
     * All assets are loaded from the ../HSI/ directory, which contains
     * the pre-rendered binary files exported from the embedded project's
     * asset pipeline.  The file formats match what the STM32 LTDC and
     * DMA2D hardware consume directly.
     */
    printf("Loading HSI layers from ../HSI/\n");

    /* Layer 1: Background — L8 format, 480*480 = 230,400 bytes
     * This is the static instrument face (bezel, markings, etc.) */
    uint8_t *bg_l8 = load_bin("../HSI/background.bin", DISP_W * DISP_H);
    if (!bg_l8) return 1;

    /* Layer 3: Compass Card — AL44 format, 480*480 = 230,400 bytes
     * The numbered degree ring that rotates with aircraft heading */
    uint8_t *compass_al44 = load_bin("../HSI/compass_card.bin", DISP_W * DISP_H);
    if (!compass_al44) return 1;

    /* Layer 2: CDI Dots — AL44 format, 480*480 = 230,400 bytes
     * The deviation reference dots that rotate with selected course */
    uint8_t *cdi_dots_al44 = load_bin("../HSI/cdi_dots.bin", DISP_W * DISP_H);
    if (!cdi_dots_al44) return 1;

    /* Sprite layers: each file has an 8-byte header + ARGB1555 pixel data */
    Sprite course_ptr, cdi_bar, hdg_bug, lubber;
    if (load_sprite("../HSI/course_pointer.bin", &course_ptr)) return 1;  /* Layer 5 */
    if (load_sprite("../HSI/cdi_bar.bin",        &cdi_bar))    return 1;  /* Layer 6 */
    if (load_sprite("../HSI/heading_bug.bin",    &hdg_bug))    return 1;  /* Layer 4 */
    if (load_sprite("../HSI/lubber_aircraft.bin", &lubber))     return 1;  /* Layer 7 */

    printf("All layers loaded OK\n");

    /* ================================================================ */
    /*  FRAMEBUFFER ALLOCATION                                           */
    /* ================================================================ */
    /*
     * The software framebuffer covers the entire window (instrument face
     * + control panel).  It is ARGB8888 format (4 bytes per pixel).
     * Total size: 480 * 590 * 4 = 1,132,800 bytes (~1.1 MB).
     *
     * calloc zero-initializes the buffer, so the panel area starts black.
     */
    /* ---- Framebuffer (full window including panel) ---- */
    uint32_t *fb = calloc(WIN_W * WIN_H, sizeof(uint32_t));
    if (!fb) { fprintf(stderr, "framebuffer alloc failed\n"); return 1; }

    /* ================================================================ */
    /*  SDL INITIALIZATION                                               */
    /* ================================================================ */
    /*
     * We use SDL2 for:
     *   - Window creation and management
     *   - Keyboard and mouse input
     *   - Texture upload and display (via SDL_Renderer)
     *   - VSync timing (SDL_RENDERER_PRESENTVSYNC)
     *
     * The streaming texture (SDL_TEXTUREACCESS_STREAMING) is updated
     * every frame by uploading our software-rendered framebuffer.
     * SDL_RenderSetLogicalSize ensures the aspect ratio is preserved
     * if the window is resized.
     */
    /* ---- SDL init ---- */
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 1;
    }
    SDL_Window *win = SDL_CreateWindow("HSI Simulator",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIN_W, WIN_H, 0);
    SDL_Renderer *ren = SDL_CreateRenderer(win, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    SDL_RenderSetLogicalSize(ren, WIN_W, WIN_H);  /* maintain aspect ratio */
    /* ARGB8888 streaming texture — matches our framebuffer format exactly */
    SDL_Texture *tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING, WIN_W, WIN_H);

    /* ================================================================ */
    /*  INSTRUMENT STATE VARIABLES                                       */
    /* ================================================================ */
    /*
     * These variables represent the current state of the HSI instrument:
     *
     *   heading:     The aircraft's current magnetic heading (0-359.9 deg).
     *                On a real HSI, this comes from a gyro/magnetometer.
     *                The compass card rotates so this value is at the lubber line.
     *
     *   course:      The selected course / OBS setting (0-359.9 deg).
     *                On a real HSI, the pilot sets this with the course knob.
     *                The course pointer and CDI dots rotate to show this course.
     *
     *   hdg_bug_deg: The heading bug setting (0-359.9 deg).
     *                A marker on the compass card rim that the pilot sets to
     *                the desired heading for autopilot tracking.
     *
     *   cdi_dev:     CDI deviation in PIXELS, range +/-75.
     *                Positive = bar deflects to the right of course.
     *                Negative = bar deflects to the left of course.
     *                On a real VOR, 1 dot = 2 degrees; on a localizer,
     *                1 dot = 0.5 degrees.  The dot spacing is 25 pixels
     *                here, so +/-75 px = +/-3 dots full-scale deflection.
     *
     *   auto_hdg:    Demo flag — when set, heading auto-rotates at 6 deg/s.
     *   auto_cdi:    Demo flag — when set, CDI oscillates automatically.
     */
    /* ---- Instrument state ---- */
    float heading     = 0.0f;       /* aircraft magnetic heading */
    float course      = 45.0f;      /* selected course / OBS */
    float hdg_bug_deg = 90.0f;      /* heading bug setting */
    float cdi_dev     = 0.0f;       /* CDI deviation in pixels, +/-75 */
    int   auto_hdg    = 1;          /* demo: auto-rotate heading */
    int   auto_cdi    = 1;          /* demo: auto-oscillate CDI */

    /* ================================================================ */
    /*  MOUSE INTERACTION STATE                                          */
    /* ================================================================ */
    /*
     * Button interaction uses a "click then hold" paradigm:
     *   - First click: immediate single step (e.g., +1 degree)
     *   - Hold for 400ms: continuous adjustment begins (e.g., 45 deg/s)
     *
     * pressed_btn tracks which button is currently held (-1 = none).
     * press_start records the SDL tick when the button was first pressed.
     * initial_step_done prevents the single-step from repeating.
     */
    /* ---- Mouse state ---- */
    int pressed_btn = -1;           /* which button is held, -1 = none */
    Uint32 press_start = 0;         /* tick when button was first pressed */
    int initial_step_done = 0;      /* did we apply the first click step? */

    int running = 1;
    Uint32 last_tick = SDL_GetTicks();

    /* Control labels displayed above each button group */
    static const char *labels[N_CONTROLS] = { "HDG", "CRS", "DEV", "BUG" };

    /* ================================================================ */
    /*  MAIN LOOP                                                        */
    /* ================================================================ */
    /*
     * The main loop runs once per frame (vsync-limited, typically 60 Hz).
     * Each iteration:
     *   1. Computes delta-time (dt) for smooth animation regardless of
     *      frame rate.  dt is clamped to 0.1s to prevent huge jumps if
     *      the application is paused/backgrounded.
     *   2. Processes input events (keyboard and mouse).
     *   3. Applies button actions (single-step or continuous hold).
     *   4. Applies keyboard-held key adjustments.
     *   5. Runs demo animations (auto heading rotation, CDI oscillation).
     *   6. Wraps angles to 0-360 range and clamps CDI deviation.
     *   7. Computes rotation angles for each layer.
     *   8. Renders all 7 instrument layers in compositing order.
     *   9. Renders the GUI control panel.
     *  10. Uploads framebuffer to SDL texture and presents.
     */
    while (running) {
        Uint32 now = SDL_GetTicks();
        float dt = (now - last_tick) / 1000.0f;  /* delta time in seconds */
        if (dt > 0.1f) dt = 0.1f;  /* clamp to prevent huge jumps */
        last_tick = now;

        /* ---- Mouse position for hover detection ---- */
        int mx, my;
        SDL_GetMouseState(&mx, &my);
        int hovered_btn = hit_test(mx, my);  /* which button is the mouse over? */

        /* ---- Process SDL events ---- */
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            switch (ev.type) {
            case SDL_QUIT:
                running = 0;  /* window close button */
                break;

            case SDL_KEYDOWN:
                switch (ev.key.keysym.sym) {
                case SDLK_ESCAPE: running = 0; break;     /* quit */
                case SDLK_SPACE:  auto_hdg = !auto_hdg; break;  /* toggle auto heading */
                case SDLK_TAB:    auto_cdi = !auto_cdi; break;  /* toggle auto CDI */
                case SDLK_r:
                    /* Reset all state to defaults */
                    heading = 0; course = 45; hdg_bug_deg = 90;
                    cdi_dev = 0; auto_hdg = 1; auto_cdi = 1;
                    break;
                default: break;
                }
                break;

            case SDL_MOUSEBUTTONDOWN:
                if (ev.button.button == SDL_BUTTON_LEFT) {
                    /* Record which button was clicked and when */
                    pressed_btn = hit_test(ev.button.x, ev.button.y);
                    press_start = now;
                    initial_step_done = 0;  /* allow the first-click step */
                }
                break;

            case SDL_MOUSEBUTTONUP:
                if (ev.button.button == SDL_BUTTON_LEFT)
                    pressed_btn = -1;  /* release the button */
                break;
            }
        }

        /* ============================================================ */
        /*  APPLY BUTTON ACTIONS                                         */
        /* ============================================================ */
        /*
         * Button interaction model:
         *   - On the first frame after click: apply a discrete step
         *     (1 degree for angles, 5 pixels for CDI deviation).
         *   - After 400ms of holding: switch to continuous mode at a
         *     rate proportional to dt (45 deg/s for angles, 75 px/s
         *     for CDI deviation).
         *
         * The button ID encodes both the group (HDG/CRS/DEV/BUG) and
         * the direction (left=decrease, right=increase):
         *   group = id / 2    (0=HDG, 1=CRS, 2=DEV, 3=BUG)
         *   dir   = id & 1    (0=left/minus, 1=right/plus)
         */
        /* Single step on first click, then continuous after 400ms hold */
        if (pressed_btn >= 0) {
            float step_deg = 1.0f;          /* 1 degree per click */
            float step_dev = 5.0f;          /* 5px per click (0.2 dots, since 1 dot = 25 px) */
            float cont_deg = 45.0f * dt;    /* 45 degrees/second continuous rate */
            float cont_dev = 75.0f * dt;    /* 75 pixels/second continuous rate */

            int dir = (pressed_btn & 1) ? 1 : -1;  /* even=left/minus, odd=right/plus */
            int group = pressed_btn / 2;
            float *target = NULL;   /* pointer to the variable being adjusted */
            float step = 0, cont = 0;

            /* Select which variable to adjust based on group */
            switch (group) {
            case 0: target = &heading;     step = step_deg; cont = cont_deg; auto_hdg = 0; break;
            case 1: target = &course;      step = step_deg; cont = cont_deg; break;
            case 2: target = &cdi_dev;     step = step_dev; cont = cont_dev; auto_cdi = 0; break;
            case 3: target = &hdg_bug_deg; step = step_deg; cont = cont_deg; break;
            }

            if (target) {
                if (!initial_step_done) {
                    /* First click: apply one discrete step */
                    *target += dir * step;
                    initial_step_done = 1;
                } else if (now - press_start > 400) {
                    /* After 400ms hold: continuous adjustment */
                    *target += dir * cont;
                }
            }
        }

        /* ============================================================ */
        /*  CONTINUOUS KEYBOARD INPUT                                    */
        /* ============================================================ */
        /*
         * SDL_GetKeyboardState returns a snapshot of ALL keys currently
         * held down.  This provides smooth continuous adjustment when
         * keys are held, unlike SDL_KEYDOWN events which only fire once
         * (or with key repeat delays).
         *
         * Rates: angles change at 30 deg/s, CDI at 50 px/s.
         * Adjusting heading or CDI manually disables the corresponding
         * auto-demo mode.
         */
        const Uint8 *keys = SDL_GetKeyboardState(NULL);
        float rate = 30.0f;       /* angular rate for keyboard: 30 deg/s */
        float dev_rate = 50.0f;   /* CDI rate for keyboard: 50 px/s */

        if (keys[SDL_SCANCODE_LEFT])  { heading -= rate * dt; auto_hdg = 0; }
        if (keys[SDL_SCANCODE_RIGHT]) { heading += rate * dt; auto_hdg = 0; }
        if (keys[SDL_SCANCODE_UP])    course  -= rate * dt;
        if (keys[SDL_SCANCODE_DOWN])  course  += rate * dt;
        if (keys[SDL_SCANCODE_Q])     hdg_bug_deg -= rate * dt;
        if (keys[SDL_SCANCODE_W])     hdg_bug_deg += rate * dt;
        if (keys[SDL_SCANCODE_A])     { cdi_dev -= dev_rate * dt; auto_cdi = 0; }
        if (keys[SDL_SCANCODE_D])     { cdi_dev += dev_rate * dt; auto_cdi = 0; }

        /* ============================================================ */
        /*  DEMO ANIMATIONS                                              */
        /* ============================================================ */
        /*
         * When auto_hdg is enabled, the heading slowly rotates at 6 deg/s,
         * causing the compass card to spin — useful for demonstrating
         * that the instrument looks correct in motion.
         *
         * When auto_cdi is enabled, the CDI deviation oscillates in a
         * triangle wave pattern between -50 and +50 pixels over an
         * 8-second period.  The math:
         *   t = time mod 8 seconds
         *   First half (t < 4s): dev goes from -50 to +50 (slope = 25 px/s)
         *   Second half (t >= 4s): dev goes from +50 to -50 (slope = -25 px/s)
         * This creates a smooth back-and-forth CDI bar sweep.
         */
        if (auto_hdg) heading += 6.0f * dt;
        if (auto_cdi) {
            float t = fmodf(now / 1000.0f, 8.0f);  /* 0-8 second cycle */
            cdi_dev = (t < 4.0f) ? -50.0f + 25.0f * t    /* rising ramp  */
                                 : 150.0f - 25.0f * t;   /* falling ramp */
        }

        /* ============================================================ */
        /*  NORMALIZE ANGLES AND CLAMP CDI                               */
        /* ============================================================ */
        /*
         * Wrap all angles into the 0-360 degree range using fmodf.
         * fmodf can return negative values for negative inputs, so we
         * check and add 360 if needed.
         *
         * CDI deviation is clamped to +/-75 pixels (3 dots full-scale),
         * matching the real KI-525A's physical CDI bar travel limits.
         */
        /* ---- Wrap angles 0-360, clamp CDI ---- */
        heading     = fmodf(heading, 360.0f);
        course      = fmodf(course, 360.0f);
        hdg_bug_deg = fmodf(hdg_bug_deg, 360.0f);
        if (heading < 0)     heading     += 360.0f;
        if (course < 0)      course      += 360.0f;
        if (hdg_bug_deg < 0) hdg_bug_deg += 360.0f;
        if (cdi_dev >  75.0f) cdi_dev =  75.0f;
        if (cdi_dev < -75.0f) cdi_dev = -75.0f;

        /* ============================================================ */
        /*  COMPUTE ROTATION ANGLES                                      */
        /* ============================================================ */
        /*
         * All rotations are expressed as "how many degrees to rotate
         * the layer on screen."  Positive = clockwise when viewed.
         *
         * card_rot = -heading:
         *   The compass card must rotate OPPOSITE to the heading.  When
         *   the aircraft turns right (heading increases), the card must
         *   rotate left (counter-clockwise) so the increasing heading
         *   value appears under the fixed lubber line at the top.
         *
         * crs_rot = course - heading:
         *   The course pointer must show the selected course relative
         *   to the current heading.  If course = 90 and heading = 0,
         *   the pointer points 90 degrees CW (east).  If heading then
         *   changes to 90, crs_rot becomes 0 and the pointer points
         *   straight up (the aircraft is now flying the course).
         *
         * bug_rot = hdg_bug_deg - heading:
         *   Same logic as course — the heading bug is positioned
         *   relative to the compass card, which already shows heading.
         */
        /* ---- Rotation angles (positive = CW on screen) ---- */
        float card_rot = -heading;              /* compass card rotation */
        float crs_rot  = course - heading;      /* course pointer / CDI rotation */
        float bug_rot  = hdg_bug_deg - heading; /* heading bug rotation */

        /* ============================================================ */
        /*  RENDER INSTRUMENT (top 480x480)                              */
        /* ============================================================ */
        /*
         * Layers are painted in strict bottom-to-top order.  Each layer
         * overwrites or alpha-blends onto whatever was painted before it.
         * The order matters because:
         *
         *   1. Background (static) — fills the entire instrument face.
         *
         *   2. CDI Dots (rotates with course) — drawn BEFORE the compass
         *      card so the card's opaque areas mask the dots.  Only the
         *      dots visible through the course window (the transparent
         *      center region of the compass card) are seen.
         *
         *   3. Compass Card (rotates with heading) — the numbered ring.
         *      Its opaque outer rim and center hub cover the dots outside
         *      the course deviation window.
         *
         *   4. Heading Bug (rotates with heading bug) — on top of the
         *      compass card so it is always visible on the card rim.
         *
         *   5. Course Pointer (rotates with course) — the long needle
         *      must be visible over the compass card markings.
         *
         *   6. CDI Bar (rotates with course + lateral translation) —
         *      drawn over the course pointer so the bar is clearly
         *      visible sliding across the dots.
         *
         *   7. Lubber / Aircraft Symbol (static) — drawn LAST so it
         *      is ALWAYS on top of everything.  This fixed reference
         *      mark must never be obscured by rotating elements.
         */

        /* Layer 1: Background — static L8 grayscale, no rotation */
        blit_background(fb, bg_l8);

        /* Layer 2: CDI Dots — rotates with course (same angle as pointer) */
        rotate_blend_al44(fb, cdi_dots_al44, crs_rot);

        /* Layer 3: Compass Card — rotates opposite to heading */
        rotate_blend_al44(fb, compass_al44, card_rot);

        /* Layer 4: Heading Bug — sprite rotates by bug angle relative to heading */
        rotate_blend_sprite(fb, &hdg_bug, bug_rot, 0, 0);

        /* Layer 5: Course Pointer — sprite rotates by course relative to heading */
        rotate_blend_sprite(fb, &course_ptr, crs_rot, 0, 0);

        /*
         * *** Layer 6: CDI BAR — THE KEY COMPOSITE OPERATION ***
         *
         * rotate_blend_sprite(fb, &cdi_bar, crs_rot, cdi_dev, 0);
         *
         * This is the call that makes the HSI come alive.  It renders
         * the CDI deviation bar with SIMULTANEOUS rotation and translation:
         *
         *   crs_rot:  rotates the bar to match the course pointer angle.
         *   cdi_dev:  lateral pixel offset (the tx parameter) that shifts
         *             the bar LEFT or RIGHT in the sprite's local coordinate
         *             system BEFORE the rotation is applied.
         *
         * HOW IT WORKS — step by step:
         *
         *   1. The CDI bar sprite is authored as a VERTICAL bar centered
         *      at the display center.  At 0 degrees with 0 deviation,
         *      it sits directly on top of the course pointer.
         *
         *   2. The tx parameter (cdi_dev) shifts the bar horizontally
         *      in the sprite's own coordinate frame.  If cdi_dev = +50,
         *      the bar moves 50 pixels to the right IN SPRITE SPACE.
         *
         *   3. The entire shifted bar is then rotated by crs_rot degrees
         *      around the display center.
         *
         *   4. Because the translation happens BEFORE rotation, the
         *      horizontal offset in sprite space becomes a PERPENDICULAR
         *      offset relative to the rotated course line:
         *
         *      - Course = 0 (north):  bar is vertical, offset is horizontal
         *      - Course = 90 (east):  bar is horizontal, offset is vertical
         *      - Course = 45 (NE):    bar is diagonal, offset is perpendicular
         *                             to the 45-degree line
         *
         *   5. This perfectly mimics the real KI-525A mechanism, where
         *      the CDI bar is mounted on a slider that moves perpendicular
         *      to the course pointer shaft, and the entire assembly
         *      (pointer + slider + bar) rotates as one unit.
         *
         *   Inside rotate_blend_sprite, the inverse mapping for each
         *   destination pixel (dx, dy) is:
         *     sx = CX + (dx-CX)*cos + (dy-CY)*sin - cdi_dev
         *     sy = CY - (dx-CX)*sin + (dy-CY)*cos - 0
         *   The "- cdi_dev" on the X axis is what creates the lateral
         *   offset, and it happens in the un-rotated coordinate frame
         *   because we apply the inverse rotation FIRST and THEN subtract
         *   the translation.  This is mathematically equivalent to
         *   translating the sprite first and then rotating the result.
         */
        rotate_blend_sprite(fb, &cdi_bar, crs_rot, cdi_dev, 0);

        /* Layer 7: Lubber / Aircraft symbol — static, always on top */
        blit_sprite(fb, &lubber);

        /* ============================================================ */
        /*  RENDER CONTROL PANEL (bottom 110px)                          */
        /* ============================================================ */
        /*
         * The control panel provides a visual interface for adjusting
         * the instrument's four parameters.  It is rendered entirely
         * in software using the same framebuffer — no SDL widgets.
         *
         * Layout (4 groups, each 120px wide):
         *   [  HDG  ] [  CRS  ] [  DEV  ] [  BUG  ]
         *   [  000  ] [  045  ] [ +0.0  ] [  090  ]
         *   [< ►  ►>] [< ►  ►>] [< ►  ►>] [< ►  ►>]
         *
         * HDG, CRS, BUG values are displayed as 3-digit degrees (000-359).
         * DEV is displayed in dots with sign (e.g., "+2.0" = 2 dots right),
         * computed by dividing the pixel offset by 25 (the dot spacing).
         */

        /* Panel background — dark gray fills the bottom panel area */
        fill_rect(fb, 0, DISP_H, WIN_W, PANEL_H, COL_PANEL_BG);

        /* Horizontal separator line at the top of the panel */
        fill_rect(fb, 0, DISP_H, WIN_W, 1, COL_SEPARATOR);

        /* Vertical separator lines between control groups */
        for (int g = 1; g < N_CONTROLS; g++)
            fill_rect(fb, g * GROUP_W, DISP_H + 4, 1, PANEL_H - 8, COL_SEPARATOR);

        /* Draw each control group: label, value, and two buttons */
        for (int g = 0; g < N_CONTROLS; g++) {
            int gcx = g * GROUP_W + GROUP_W / 2;   /* group horizontal center x */

            /* Label text (e.g., "HDG", "CRS", "DEV", "BUG") */
            draw_str_cx(fb, gcx, LABEL_Y, labels[g], COL_LABEL);

            /* Format and display the current value */
            char val[16];
            switch (g) {
            case 0: snprintf(val, sizeof(val), "%03.0f", heading);            break;  /* degrees */
            case 1: snprintf(val, sizeof(val), "%03.0f", course);             break;  /* degrees */
            case 2: snprintf(val, sizeof(val), "%+.1f", cdi_dev / 25.0f);    break;  /* dots (25 px/dot) */
            case 3: snprintf(val, sizeof(val), "%03.0f", hdg_bug_deg);       break;  /* degrees */
            }
            draw_str_cx(fb, gcx, VALUE_Y, val, COL_VALUE);

            /* Left (decrease) and right (increase) buttons with hover/press state */
            int btn_l = g * 2;      /* left button ID */
            int btn_r = g * 2 + 1;  /* right button ID */
            draw_button(fb, BTN_L_X(g), BTN_Y, -1,
                        hovered_btn == btn_l, pressed_btn == btn_l);
            draw_button(fb, BTN_R_X(g), BTN_Y,  1,
                        hovered_btn == btn_r, pressed_btn == btn_r);
        }

        /* ============================================================ */
        /*  PRESENT FRAME                                                */
        /* ============================================================ */
        /*
         * Upload the software-rendered framebuffer to the SDL texture,
         * then copy the texture to the renderer and present.
         *
         * SDL_UpdateTexture copies our ARGB8888 pixel data to the GPU.
         * The pitch (WIN_W * 4) tells SDL how many bytes per row.
         * SDL_RenderCopy blits the texture to the window.
         * SDL_RenderPresent swaps the backbuffer (and waits for vsync).
         */
        /* ---- Present ---- */
        SDL_UpdateTexture(tex, NULL, fb, WIN_W * 4);
        SDL_RenderCopy(ren, tex, NULL, NULL);
        SDL_RenderPresent(ren);
    }

    /* ================================================================ */
    /*  CLEANUP                                                          */
    /* ================================================================ */
    /*
     * Destroy SDL resources in reverse order of creation, then free
     * all heap-allocated memory (framebuffer, asset buffers, sprite
     * pixel arrays).
     */
    /* ---- Cleanup ---- */
    SDL_DestroyTexture(tex);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();

    free(fb);
    free(bg_l8);
    free(compass_al44);
    free(cdi_dots_al44);
    free(course_ptr.px);
    free(cdi_bar.px);
    free(hdg_bug.px);
    free(lubber.px);

    return 0;
}
