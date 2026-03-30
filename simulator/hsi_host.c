/*
 * HSI Host — runs on PC, sends UDP to Raspberry Pi receiver
 * Desktop SDL2 renderer for 480x480 KI-525A HSI face
 * with clickable GUI control panel
 *
 * ======================================================================
 * OVERVIEW
 * ======================================================================
 * This program simulates a KI-525A Horizontal Situation Indicator (HSI),
 * a navigation instrument found in general-aviation and military aircraft.
 * An HSI combines a heading indicator (directional gyro) with a VOR/ILS
 * course deviation indicator into a single round instrument face.
 *
 * The program has two roles:
 *   1. LOCAL RENDERER — draws the HSI on screen using SDL2, compositing
 *      multiple pre-rendered sprite layers with real-time rotation.
 *   2. UDP TRANSMITTER — sends the current instrument state (heading,
 *      course, heading bug, CDI deviation) to a Raspberry Pi at ~30 Hz
 *      so the Pi can render the same instrument on its own HDMI display.
 *
 * The rendering pipeline mimics what the STM32F767's LTDC + DMA2D
 * hardware does on the real instrument: each graphical layer is stored
 * as a pre-rendered bitmap and composited in painter's-order with
 * per-pixel alpha blending and rotation.
 *
 * Usage: hsi_host.exe [pi_ip_address]
 *        Default IP: 192.168.1.129, port 5555
 *
 * Controls (keyboard):
 *   Left/Right — heading          Up/Down — course (OBS)
 *   Q/W        — heading bug      A/D     — CDI deviation
 *   SPACE      — toggle auto-hdg  TAB     — toggle auto-cdi
 *   R          — reset all        ESC     — quit
 *
 * Controls (mouse):
 *   Click or hold the arrow buttons in the bottom panel.
 */

/* ======================================================================
 * INCLUDES
 * ======================================================================
 * SDL2       — cross-platform window, rendering, and input
 * stdio      — printf / fprintf / snprintf / fopen / fread / fclose
 * stdlib     — malloc / calloc / free / atoi
 * stdint     — fixed-width types (uint8_t, uint16_t, uint32_t)
 * string     — memset / strlen
 * math       — sinf / cosf / fmodf / fabsf / M_PI
 * winsock2   — Windows BSD-socket API for UDP networking
 */
#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <winsock2.h>

/*
 * Tell the MSVC linker to pull in the Winsock library automatically.
 * This avoids needing to add ws2_32.lib to the linker command line manually.
 * (MinGW/GCC ignores this pragma — you must pass -lws2_32 instead.)
 */
#pragma comment(lib, "ws2_32.lib")

/* ======================================================================
 * UDP PACKET STRUCTURE
 * ======================================================================
 * This is the binary packet sent to the Raspberry Pi at ~30 Hz.
 * The Pi receiver uses instrument_id to demux when multiple instruments
 * share the same UDP port. The struct is sent as raw bytes (no
 * serialization), so both sides must agree on packing and endianness
 * (both x86 PC and ARM Pi are little-endian, so this works).
 *
 * Fields:
 *   instrument_id — discriminator: 3 = HSI (other IDs exist for
 *                   altimeter, VSI, etc.)
 *   heading       — magnetic heading in degrees [0, 360). Controls
 *                   compass card rotation.
 *   course        — selected VOR/ILS course (OBS setting) in degrees
 *                   [0, 360). Controls course pointer + CDI dots rotation.
 *   heading_bug   — heading bug position in degrees [0, 360). The small
 *                   triangular marker the pilot sets for desired heading.
 *   cdi_dev       — course deviation in "dots" [-3, +3]. One dot equals
 *                   one CDI dot spacing. Negative = fly left, positive =
 *                   fly right to return to course.
 *
 * Total packet size: 4 + 4 + 4 + 4 + 4 = 20 bytes.
 */
typedef struct {
    uint32_t instrument_id;     /* 3 = HSI */
    float    heading;           /* degrees, 0-360 */
    float    course;            /* degrees, 0-360 */
    float    heading_bug;       /* degrees, 0-360 */
    float    cdi_dev;           /* dots, -3 to +3 */
} HsiPacket;

#define INSTRUMENT_HSI  3       /* Packet discriminator for the HSI */
#define PI_PORT         5555    /* UDP destination port on the Raspberry Pi */
#define SEND_INTERVAL   33      /* milliseconds between UDP sends (~30 Hz) */

/* ======================================================================
 * DISPLAY GEOMETRY
 * ======================================================================
 * The HSI instrument face is 480x480 pixels — a square that fits a round
 * instrument bezel. Below the instrument face is a 110-pixel tall
 * control panel strip for the GUI buttons and readouts.
 *
 * The window is therefore 480 wide x 590 tall (480 + 110).
 *
 * CX, CY are the center of the instrument face — the rotation pivot for
 * the compass card, course pointer, CDI bar, and heading bug. All
 * rotations happen around this point.
 */
#define DISP_W    480           /* Instrument face width in pixels */
#define DISP_H    480           /* Instrument face height in pixels */
#define PANEL_H   110           /* Height of the GUI control panel below */
#define WIN_W     DISP_W        /* Total window width */
#define WIN_H     (DISP_H + PANEL_H)   /* Total window height: 590 px */
#define CX        (DISP_W / 2)  /* X center of instrument: 240 */
#define CY        (DISP_H / 2)  /* Y center of instrument: 240 */

/* ======================================================================
 * PANEL LAYOUT
 * ======================================================================
 * The control panel is divided into 4 equal groups (one per parameter):
 *   Group 0: HDG (heading)
 *   Group 1: CRS (course / OBS)
 *   Group 2: DEV (CDI deviation in dots)
 *   Group 3: BUG (heading bug)
 *
 * Each group has a label, a numeric readout, and left/right arrow
 * buttons. The macros below position everything within each group.
 *
 * GROUP_W = 480 / 4 = 120 pixels per group.
 * BTN_L_X(g) gives the X position of the left (decrement) button.
 * BTN_R_X(g) gives the X position of the right (increment) button.
 */
#define N_CONTROLS  4                       /* Number of parameter groups */
#define GROUP_W     (WIN_W / N_CONTROLS)    /* Width of each group: 120 px */
#define BTN_W       32                      /* Button width in pixels */
#define BTN_H       28                      /* Button height in pixels */
#define BTN_Y       (DISP_H + 58)          /* Y position of buttons (absolute) */
#define LABEL_Y     (DISP_H + 14)          /* Y position of the parameter label */
#define VALUE_Y     (DISP_H + 38)          /* Y position of the numeric readout */
#define BTN_L_X(g)  ((g) * GROUP_W + 6)                    /* Left button X */
#define BTN_R_X(g)  ((g) * GROUP_W + GROUP_W - 6 - BTN_W)  /* Right button X */

/* ======================================================================
 * PANEL COLORS
 * ======================================================================
 * All colors are 32-bit ARGB (0xAARRGGBB). The alpha byte is 0xFF
 * (fully opaque) because these go straight into the framebuffer.
 * The panel uses a dark theme to look like real avionics.
 */
#define COL_PANEL_BG    0xFF1A1A1Au  /* Very dark gray panel background */
#define COL_SEPARATOR   0xFF333333u  /* Slightly lighter gray dividers */
#define COL_BTN_NORMAL  0xFF3A3A3Au  /* Button face in idle state */
#define COL_BTN_HOVER   0xFF505050u  /* Button face when mouse hovers */
#define COL_BTN_PRESS   0xFF686868u  /* Button face when pressed/held */
#define COL_ARROW       0xFFDDDDDDu  /* Arrow glyph color (near-white) */
#define COL_LABEL       0xFF888888u  /* Dim gray for "HDG", "CRS", etc. */
#define COL_VALUE       0xFF44DD44u  /* Bright green for numeric readouts */

/* ======================================================================
 * 5x7 BITMAP FONT
 * ======================================================================
 * A minimal bitmap font for rendering text in the control panel.
 * Each glyph is 5 pixels wide by 7 pixels tall. Each row of a glyph
 * is stored as a single byte; bits 4..0 represent columns left to right.
 * For example, '0' row 0 = 0x0E = 01110 binary, meaning columns 1,2,3
 * are lit and columns 0,4 are dark.
 *
 * FONT_SCALE = 2 means each source pixel becomes a 2x2 block on screen,
 * yielding 10x14 pixel characters. CHAR_W includes a 1-column gap
 * between characters (also scaled).
 *
 * Only the characters actually needed are included: digits 0-9, a few
 * letters (A,B,C,D,E,G,H,R,S,U,V for labels like "HDG", "CRS", "DEV",
 * "BUG", "HEADING", etc.), plus '+', '-', '.', and space.
 */
#define FONT_W  5       /* Glyph width in source pixels */
#define FONT_H  7       /* Glyph height in source pixels */
#define FONT_SCALE  2   /* Each source pixel becomes 2x2 on screen */
#define CHAR_W  ((FONT_W + 1) * FONT_SCALE)  /* Char cell width: 12 px */
#define CHAR_H  (FONT_H * FONT_SCALE)        /* Char cell height: 14 px */

/*
 * Glyph table. Each entry is a character and its 7-row bitmap.
 * The table is searched linearly by draw_char() — at 25 entries this
 * is negligible cost.
 */
static const struct { char ch; uint8_t r[7]; } glyphs[] = {
    {'0', {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}},
    {'1', {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}},
    {'2', {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F}},
    {'3', {0x1F,0x02,0x04,0x02,0x01,0x11,0x0E}},
    {'4', {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}},
    {'5', {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E}},
    {'6', {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}},
    {'7', {0x1F,0x01,0x02,0x04,0x08,0x08,0x08}},
    {'8', {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}},
    {'9', {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C}},
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
    {'+', {0x00,0x04,0x04,0x1F,0x04,0x04,0x00}},
    {'-', {0x00,0x00,0x00,0x1F,0x00,0x00,0x00}},
    {'.', {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C}},
    {' ', {0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
};
#define N_GLYPHS ((int)(sizeof(glyphs)/sizeof(glyphs[0])))

/* ----------------------------------------------------------------------
 * draw_char — render a single character into the framebuffer
 * ----------------------------------------------------------------------
 * Parameters:
 *   fb    — pointer to the ARGB8888 framebuffer (WIN_W * WIN_H pixels)
 *   x, y  — top-left screen position to place the character
 *   c     — the ASCII character to render
 *   color — 0xAARRGGBB color for the lit pixels; background is not drawn
 *           (transparent), so text overlays whatever is already in fb.
 *
 * Algorithm:
 *   1. Linear-search the glyph table for the matching character.
 *   2. For each of the 7 rows, test bits 4..0 (MSB = leftmost column).
 *      The expression (0x10 >> col) generates a bitmask for each column.
 *   3. Each lit source pixel is expanded to a FONT_SCALE x FONT_SCALE
 *      block (2x2) and written directly into the framebuffer.
 *   4. Bounds checking prevents writes outside the window.
 */
static void draw_char(uint32_t *fb, int x, int y, char c, uint32_t color)
{
    /* Find the glyph bitmap for this character */
    const uint8_t *bits = NULL;
    for (int i = 0; i < N_GLYPHS; i++)
        if (glyphs[i].ch == c) { bits = glyphs[i].r; break; }
    if (!bits) return;  /* Character not in our font — skip silently */

    /* Walk each row and column of the 5x7 glyph bitmap */
    for (int row = 0; row < FONT_H; row++)
        for (int col = 0; col < FONT_W; col++)
            if (bits[row] & (0x10 >> col))  /* Is this pixel lit? */
                /* Expand the single source pixel to a FONT_SCALE x FONT_SCALE block */
                for (int sy = 0; sy < FONT_SCALE; sy++)
                    for (int sx = 0; sx < FONT_SCALE; sx++) {
                        int px = x + col * FONT_SCALE + sx;
                        int py = y + row * FONT_SCALE + sy;
                        /* Bounds check: cast to unsigned so negative values
                         * wrap to large positives and fail the < test. */
                        if ((unsigned)px < WIN_W && (unsigned)py < WIN_H)
                            fb[py * WIN_W + px] = color;
                    }
}

/* ----------------------------------------------------------------------
 * draw_str — render a null-terminated string left-aligned at (x, y)
 * ---------------------------------------------------------------------- */
static void draw_str(uint32_t *fb, int x, int y, const char *s, uint32_t color)
{ while (*s) { draw_char(fb, x, y, *s++, color); x += CHAR_W; } }

/* ----------------------------------------------------------------------
 * draw_str_cx — render a string horizontally centered on column 'cx'
 * ----------------------------------------------------------------------
 * Computes the total pixel width of the string (accounting for the
 * inter-character gap being absent after the last character), then
 * offsets by half that width to center.
 */
static void draw_str_cx(uint32_t *fb, int cx, int y, const char *s, uint32_t color)
{
    int len = (int)strlen(s);
    int total_w = len * CHAR_W - FONT_SCALE;  /* Subtract trailing gap */
    draw_str(fb, cx - total_w / 2, y, s, color);
}

/* ======================================================================
 * GUI HELPERS
 * ======================================================================
 * Utility functions for drawing the control panel's buttons and arrows.
 */

/* ----------------------------------------------------------------------
 * fill_rect — draw a solid filled rectangle into the framebuffer
 * ----------------------------------------------------------------------
 * Simple scanline fill with per-pixel bounds checking. Used for panel
 * background, button faces, and separator lines.
 */
static void fill_rect(uint32_t *fb, int rx, int ry, int rw, int rh, uint32_t c)
{
    for (int y = ry; y < ry + rh; y++) {
        if ((unsigned)y >= WIN_H) continue;
        for (int x = rx; x < rx + rw; x++)
            if ((unsigned)x < WIN_W) fb[y * WIN_W + x] = c;
    }
}

/* ----------------------------------------------------------------------
 * draw_arrow — render a triangular arrow inside a button rectangle
 * ----------------------------------------------------------------------
 * Parameters:
 *   bx, by, bw, bh — bounding box of the button
 *   dir             — direction: -1 = left-pointing, +1 = right-pointing
 *   color           — arrow fill color
 *
 * The arrow is drawn as a diamond/triangle shape: each row's width is
 * proportional to (half - |row - half|), producing a pointed shape.
 * For dir < 0 (left arrow) the widest part is on the right; for dir > 0
 * (right arrow) the widest part is on the left.
 *
 * 'pad' insets the arrow from the button edges so it doesn't touch
 * the button border.
 */
static void draw_arrow(uint32_t *fb, int bx, int by, int bw, int bh,
                        int dir, uint32_t color)
{
    int pad = 9, th = bh - 2*pad, tw = bw - 2*pad;
    if (th < 2 || tw < 2) return;  /* Button too small for an arrow */
    int half = th / 2;
    for (int row = 0; row < th; row++) {
        /* Width tapers linearly: max at the middle row, 1 at the tips */
        int w = tw * (half - abs(row - half)) / half;
        if (w < 1) w = 1;
        int y = by + pad + row;
        /* For left arrow, the wide edge is right-aligned; for right, left-aligned */
        int x0 = (dir < 0) ? bx + bw - pad - w : bx + pad;
        for (int c2 = 0; c2 < w; c2++)
            if ((unsigned)(x0+c2) < WIN_W && (unsigned)y < WIN_H)
                fb[y * WIN_W + x0 + c2] = color;
    }
}

/* ----------------------------------------------------------------------
 * draw_button — render a complete button (background + arrow glyph)
 * ----------------------------------------------------------------------
 * Picks the appropriate background color based on hover/press state,
 * then draws the arrow glyph on top.
 */
static void draw_button(uint32_t *fb, int bx, int by, int dir, int hover, int press)
{
    uint32_t bg = press ? COL_BTN_PRESS : hover ? COL_BTN_HOVER : COL_BTN_NORMAL;
    fill_rect(fb, bx, by, BTN_W, BTN_H, bg);
    draw_arrow(fb, bx, by, BTN_W, BTN_H, dir, COL_ARROW);
}

/* ======================================================================
 * ARGB1555 SPRITE TYPE
 * ======================================================================
 * Sprites are used for instrument elements that need rotation and/or
 * alpha keying: the course pointer, CDI bar, heading bug, and lubber
 * (aircraft symbol).
 *
 * Each sprite is stored as a binary file with an 8-byte header followed
 * by raw ARGB1555 pixel data:
 *
 *   Offset  Size  Field
 *   0       2     ox    — X origin (where the sprite's left edge sits
 *                         in the 480x480 instrument coordinate system)
 *   2       2     oy    — Y origin (top edge)
 *   4       2     w     — sprite width in pixels
 *   6       2     h     — sprite height in pixels
 *   8       w*h*2 px    — pixel data in ARGB1555 format
 *
 * ARGB1555 FORMAT:
 *   Bit 15      = alpha (1 = opaque, 0 = fully transparent / key color)
 *   Bits 14..10 = red   (5 bits, 0-31)
 *   Bits 9..5   = green (5 bits, 0-31)
 *   Bits 4..0   = blue  (5 bits, 0-31)
 *
 * This is a 1-bit alpha format — each pixel is either fully opaque or
 * fully transparent. This matches the STM32F767's DMA2D ARGB1555 mode
 * and is ideal for hard-edged instrument graphics like pointers and bugs.
 * To convert to 8-bit-per-channel color, each 5-bit channel is shifted
 * left by 3 (e.g., 0x1F -> 0xF8), giving a [0, 248] range.
 */
typedef struct { int ox, oy, w, h; uint16_t *px; } Sprite;

/* ----------------------------------------------------------------------
 * load_bin — load a raw binary file of known size
 * ----------------------------------------------------------------------
 * Used for full-frame layers (background, compass card, CDI dots) that
 * are exactly DISP_W * DISP_H bytes. Returns a malloc'd buffer or NULL
 * on failure.
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

/* ----------------------------------------------------------------------
 * load_sprite — load an ARGB1555 sprite from a binary file
 * ----------------------------------------------------------------------
 * Reads the 8-byte header (ox, oy, w, h as four uint16_t values),
 * then allocates and reads w * h pixels of ARGB1555 data.
 */
static int load_sprite(const char *path, Sprite *s)
{
    FILE *f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "Cannot open %s\n", path); return -1; }
    uint16_t hdr[4];  /* ox, oy, w, h — each a 16-bit unsigned int */
    if (fread(hdr, 2, 4, f) != 4) { fclose(f); return -1; }
    s->ox = hdr[0]; s->oy = hdr[1]; s->w = hdr[2]; s->h = hdr[3];
    size_t npx = (size_t)s->w * s->h;
    s->px = malloc(npx * 2);  /* 2 bytes per ARGB1555 pixel */
    if (!s->px) { fclose(f); return -1; }
    if (fread(s->px, 2, npx, f) != npx) { free(s->px); fclose(f); return -1; }
    fclose(f);
    return 0;
}

/* ======================================================================
 * INSTRUMENT LAYER COMPOSITING
 * ======================================================================
 * The HSI is rendered by compositing layers in painter's order (back to
 * front), exactly as the STM32's LTDC + DMA2D would do it in hardware.
 *
 * The compositing order (bottom to top) is:
 *   1. Background     — static bezel/face artwork (L8 grayscale)
 *   2. CDI dots       — five course-deviation dots, rotated with course
 *   3. Compass card   — 360-degree heading ring, rotated by -heading
 *   4. Heading bug    — small triangle on compass rim, rotated by bug-heading
 *   5. Course pointer — long arrow through center, rotated by course-heading
 *   6. CDI bar        — deviation bar, rotated AND translated (see below)
 *   7. Lubber line    — fixed aircraft symbol at top center (no rotation)
 *
 * Two pixel formats are used for the source layers:
 *   - AL44: 8-bit pixels where high nibble = 4-bit alpha, low nibble =
 *     4-bit luminance. Used for full-frame layers (compass card, CDI
 *     dots) that have smooth anti-aliased edges. Each 4-bit value is
 *     expanded to 8 bits by multiplying by 17 (0xF * 17 = 255).
 *   - ARGB1555: 16-bit pixels with 1-bit alpha and 5-5-5 color. Used
 *     for smaller sprite overlays (course pointer, CDI bar, heading bug,
 *     lubber) that have hard edges and need color.
 *
 * Both formats match modes supported by the STM32F767's DMA2D blitter,
 * so the same asset files work on both the PC simulator and the real
 * embedded target.
 */

/* ----------------------------------------------------------------------
 * blend_pixel — alpha-blend a single pixel into the ARGB8888 framebuffer
 * ----------------------------------------------------------------------
 * Implements standard "source over" alpha compositing:
 *   result = src * alpha + dst * (1 - alpha)
 *
 * Fast paths for a == 0 (fully transparent, skip entirely) and
 * a == 255 (fully opaque, overwrite without blending math).
 *
 * The division by 255 is exact integer division (no floating point).
 */
static inline void blend_pixel(uint32_t *dst, uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    if (a == 0) return;     /* Fully transparent — nothing to do */
    if (a == 255) {         /* Fully opaque — no blending needed, just overwrite */
        *dst = 0xFF000000u|((uint32_t)r<<16)|((uint32_t)g<<8)|b;
        return;
    }
    /* Read the existing destination pixel and extract its R, G, B channels */
    uint32_t d = *dst;
    uint8_t dr = (d>>16)&0xFF, dg = (d>>8)&0xFF, db = d&0xFF;
    /* Standard alpha-blend formula: out = src*a/255 + dst*(255-a)/255 */
    *dst = 0xFF000000u |
        ((uint32_t)((r*a + dr*(255-a))/255) << 16) |
        ((uint32_t)((g*a + dg*(255-a))/255) <<  8) |
        (uint32_t)((b*a + db*(255-a))/255);
}

/* ----------------------------------------------------------------------
 * blit_background — copy the L8 (grayscale) background to the framebuffer
 * ----------------------------------------------------------------------
 * The background is a single-channel 8-bit luminance image (480x480).
 * Each byte becomes an ARGB8888 pixel with R=G=B=luminance, A=0xFF.
 * This is the very first layer drawn, so it fills the entire instrument
 * area — no blending is needed.
 */
static void blit_background(uint32_t *fb, const uint8_t *bg)
{
    for (int i = 0; i < DISP_W * DISP_H; i++) {
        uint8_t l = bg[i];  /* Luminance value 0-255 */
        fb[i] = 0xFF000000u | ((uint32_t)l<<16) | ((uint32_t)l<<8) | l;
    }
}

/* ----------------------------------------------------------------------
 * rotate_blend_al44 — rotate an AL44 full-frame layer and blend it
 * ----------------------------------------------------------------------
 * Used for the compass card and CDI dots layers.
 *
 * AL44 PIXEL FORMAT:
 *   High nibble (bits 7..4) = 4-bit alpha  (0 = transparent, 15 = opaque)
 *   Low nibble  (bits 3..0) = 4-bit luminance (0 = black, 15 = white)
 *   Each nibble is expanded to 8 bits by multiplying by 17:
 *     0x0 -> 0,  0x8 -> 136,  0xF -> 255
 *
 * INVERSE ROTATION MATH:
 *   We iterate over every DESTINATION pixel (dx, dy) and ask: "which
 *   SOURCE pixel should I sample?" This is called "inverse mapping" and
 *   avoids holes that would appear with forward mapping.
 *
 *   The destination pixel is offset from center: (fx, fy) = (dx-CX, dy-CY).
 *
 *   To find the corresponding source pixel, we apply the INVERSE rotation
 *   (rotate by -deg). If the forward rotation matrix is:
 *       | cos(t)  -sin(t) |
 *       | sin(t)   cos(t) |
 *   then the inverse is:
 *       | cos(t)   sin(t) |
 *       |-sin(t)   cos(t) |
 *
 *   So the source coordinates are:
 *       sx = CX + fx*cos(t) + fy*sin(t)
 *       sy = CY - fx*sin(t) + fy*cos(t)
 *
 *   If (sx, sy) falls outside the source image, we skip that destination
 *   pixel (it maps to "outside the compass card" and is transparent).
 *
 *   This runs over every pixel in the 480x480 frame (230,400 pixels).
 *   On modern CPUs this is fast enough at 60+ FPS.
 */
static void rotate_blend_al44(uint32_t *fb, const uint8_t *src, float deg)
{
    float rad = deg * (float)M_PI / 180.0f;  /* Convert degrees to radians */
    float cs = cosf(rad), sn = sinf(rad);    /* Pre-compute sin and cos */

    /* Iterate over every destination pixel */
    for (int dy = 0; dy < DISP_H; dy++) {
        float fy = (float)(dy - CY);  /* Y offset from instrument center */
        for (int dx = 0; dx < DISP_W; dx++) {
            float fx = (float)(dx - CX);  /* X offset from instrument center */

            /* Inverse rotation: find which source pixel maps to (dx, dy) */
            int sx = (int)(CX + fx*cs + fy*sn + 0.5f);  /* +0.5 for rounding */
            int sy = (int)(CY - fx*sn + fy*cs + 0.5f);

            /* Skip if the source coordinate is out of bounds */
            if ((unsigned)sx >= DISP_W || (unsigned)sy >= DISP_H) continue;

            /* Read the AL44 source pixel */
            uint8_t p = src[sy * DISP_W + sx];
            uint8_t a4 = p >> 4;       /* Extract 4-bit alpha */
            if (a4 == 0) continue;     /* Fully transparent — skip */

            /* Expand 4-bit alpha and luminance to 8 bits (* 17 maps 0xF -> 255) */
            /* Luminance is used for all three RGB channels (grayscale) */
            blend_pixel(&fb[dy*WIN_W+dx], (p&0xF)*17, (p&0xF)*17, (p&0xF)*17, a4*17);
        }
    }
}

/* ======================================================================
 * rotate_blend_sprite — THE CRITICAL CDI BAR RENDERING FUNCTION
 * ======================================================================
 * This function is the heart of the HSI's CDI (Course Deviation Indicator)
 * bar rendering. It performs a combined ROTATION + LATERAL TRANSLATION
 * of an ARGB1555 sprite, which is what makes the CDI bar slide side-to-side
 * perpendicular to the course pointer while the entire assembly rotates
 * with the selected course.
 *
 * PARAMETERS:
 *   fb   — destination ARGB8888 framebuffer
 *   s    — source ARGB1555 sprite (with origin ox, oy in instrument space)
 *   deg  — rotation angle in degrees (clockwise)
 *   tx   — lateral X translation in SOURCE (pre-rotation) coordinate space
 *   ty   — lateral Y translation in SOURCE (pre-rotation) coordinate space
 *
 * KEY INSIGHT — CDI BAR MECHANICS:
 * --------------------------------
 * On a real KI-525A HSI, the course pointer is a long arrow that passes
 * through the center of the instrument and rotates to indicate the
 * selected VOR/ILS course. The CDI bar is a narrow bar that sits
 * alongside the course pointer. The CDI bar:
 *
 *   (a) ROTATES with the course pointer — it always stays parallel to the
 *       course arrow.
 *   (b) TRANSLATES LATERALLY — it slides LEFT or RIGHT (perpendicular to
 *       the course pointer) to show how far off-course the aircraft is.
 *
 * The critical rendering call in the main loop is:
 *
 *   rotate_blend_sprite(fb, &cdi_bar, crs_rot, cdi_dev, 0);
 *
 * Here:
 *   crs_rot = course - heading  (rotation angle, same as course pointer)
 *   cdi_dev = lateral pixel offset (e.g., -75 to +75 pixels)
 *
 * The tx parameter (cdi_dev) is applied in SOURCE SPACE — that is, BEFORE
 * the rotation. This is the mathematical trick that makes everything work:
 *
 *   - In source space, the CDI bar is a vertical bar near the center.
 *   - tx shifts it LEFT or RIGHT in source space (perpendicular to the
 *     bar's long axis in source space).
 *   - Then the rotation transforms the entire shifted bar into screen
 *     space, automatically preserving the perpendicular relationship.
 *
 * So when the course pointer is pointing North (0 deg), tx shifts the
 * CDI bar East/West. When the pointer is pointing East (90 deg), the
 * same tx shift in source space becomes a North/South shift on screen.
 * The bar always moves perpendicular to the pointer, regardless of angle.
 *
 * COMBINED ROTATION + TRANSLATION MATH:
 * --------------------------------------
 * Like rotate_blend_al44, this uses INVERSE MAPPING. For each destination
 * pixel (dx, dy) we compute the corresponding source coordinate. The
 * difference here is that the translation is subtracted AFTER the inverse
 * rotation and BEFORE the sprite-local lookup:
 *
 *   Step 1: Convert destination pixel to center-relative coordinates:
 *           fx = dx - CX,  fy = dy - CY
 *
 *   Step 2: Apply inverse rotation (rotate by -deg) to get the un-rotated
 *           position in source space:
 *           sx_f = CX + fx*cos(t) + fy*sin(t)
 *           sy_f = CY - fx*sin(t) + fy*cos(t)
 *
 *   Step 3: Subtract the translation to account for the CDI bar's lateral
 *           shift. Because tx and ty are in source space, we simply
 *           subtract them from the un-rotated source coordinates:
 *           sx_f -= tx
 *           sy_f -= ty
 *
 *   Step 4: Convert from instrument coordinates to sprite-local coordinates:
 *           lx = sx_f - sprite.ox
 *           ly = sy_f - sprite.oy
 *
 *   Step 5: If (lx, ly) falls within the sprite, read and composite the pixel.
 *
 * Equivalently, this computes:
 *   source_point = R^(-1) * (dest_point - center) + center - translation - origin
 *
 * The result is that the sprite appears rotated by 'deg' degrees around
 * the instrument center, and shifted by (tx, ty) in the pre-rotation
 * (source) coordinate frame.
 *
 * BOUNDING BOX OPTIMIZATION:
 * --------------------------
 * Instead of iterating over all 480x480 = 230,400 destination pixels
 * (as rotate_blend_al44 does for full-frame layers), this function
 * computes a tight AXIS-ALIGNED BOUNDING BOX (AABB) of the rotated
 * sprite and only iterates over pixels within that box.
 *
 * The bounding box is found by:
 *   1. Taking the four corners of the sprite rectangle in source space
 *      (including the tx/ty translation).
 *   2. Forward-rotating each corner into screen space using the FORWARD
 *      rotation matrix:
 *        rx = CX + fx*cos(t) - fy*sin(t)
 *        ry = CY + fx*sin(t) + fy*cos(t)
 *   3. Taking the min/max of the rotated corners to get the AABB.
 *   4. Adding a 2-pixel margin to account for rounding errors.
 *   5. Clamping to the display bounds.
 *
 * For a narrow sprite like the CDI bar (maybe 10x400 pixels), the
 * bounding box might be roughly 400x400 when rotated 45 degrees — but
 * that is still better than scanning the entire frame every time, and
 * for near-cardinal angles the box is much tighter.
 *
 * ARGB1555 PIXEL DECODING:
 *   bit 15       = alpha (1=opaque, 0=transparent; checked as p & 0x8000)
 *   bits 14..10  = red   (5 bits; extracted, shifted left 3 to get 8-bit)
 *   bits 9..5    = green (5 bits; same treatment)
 *   bits 4..0    = blue  (5 bits; same treatment)
 *
 * Since alpha is 1-bit, no blending is needed — opaque pixels fully
 * overwrite the destination, and transparent pixels are skipped.
 */
static void rotate_blend_sprite(uint32_t *fb, const Sprite *s,
                                float deg, float tx, float ty)
{
    float rad = deg*(float)M_PI/180.0f;     /* Degrees to radians */
    float cs = cosf(rad), sn = sinf(rad);   /* Pre-compute trig */

    /*
     * Compute the four corners of the sprite in source (instrument) space,
     * INCLUDING the (tx, ty) translation. These are the corners of the
     * rectangle that the sprite occupies before rotation.
     *
     * corners[0] = top-left      corners[1] = top-right
     * corners[2] = bottom-left   corners[3] = bottom-right
     */
    float corners[4][2] = {
        {s->ox+tx, s->oy+ty}, {s->ox+s->w+tx, s->oy+ty},
        {s->ox+tx, s->oy+s->h+ty}, {s->ox+s->w+tx, s->oy+s->h+ty}
    };

    /*
     * Forward-rotate each corner from source space to screen space to
     * find the axis-aligned bounding box (AABB) in screen coordinates.
     * The forward rotation matrix is:
     *   screen_x = CX + (src_x - CX)*cos - (src_y - CY)*sin
     *   screen_y = CY + (src_x - CX)*sin + (src_y - CY)*cos
     */
    int mn_x=DISP_W, mn_y=DISP_H, mx_x=0, mx_y=0;
    for (int i=0;i<4;i++) {
        float fx=corners[i][0]-CX, fy=corners[i][1]-CY;
        int rx=(int)(CX+fx*cs-fy*sn), ry=(int)(CY+fx*sn+fy*cs);
        if(rx<mn_x)mn_x=rx; if(rx>mx_x)mx_x=rx;
        if(ry<mn_y)mn_y=ry; if(ry>mx_y)mx_y=ry;
    }

    /* Expand the bounding box by 2 pixels on each side to catch rounding */
    if(mn_x<0)mn_x=0; else if(mn_x>2)mn_x-=2;
    if(mn_y<0)mn_y=0; else if(mn_y>2)mn_y-=2;
    if(mx_x>=DISP_W)mx_x=DISP_W-1; else mx_x+=2;
    if(mx_y>=DISP_H)mx_y=DISP_H-1; else mx_y+=2;
    /* Clamp again after expansion in case +2 pushed us past the edge */
    if(mx_x>=DISP_W)mx_x=DISP_W-1;
    if(mx_y>=DISP_H)mx_y=DISP_H-1;

    /*
     * Iterate over only the destination pixels within the bounding box.
     * For each one, inverse-map back to source space, subtract the
     * translation, and look up the sprite pixel.
     */
    for (int dy=mn_y; dy<=mx_y; dy++) {
        float fy=(float)(dy-CY);
        for (int dx=mn_x; dx<=mx_x; dx++) {
            float fx=(float)(dx-CX);

            /*
             * INVERSE ROTATION + TRANSLATION:
             * First, inverse-rotate the destination point back to source space:
             *   sx_f = CX + fx*cos(t) + fy*sin(t)
             *   sy_f = CY - fx*sin(t) + fy*cos(t)
             * Then subtract the translation (tx, ty) to account for the CDI
             * bar's lateral offset in source space:
             *   sx_f -= tx
             *   sy_f -= ty
             *
             * This single line does both operations:
             */
            float sx_f=CX+fx*cs+fy*sn-tx, sy_f=CY-fx*sn+fy*cs-ty;

            /*
             * Convert from instrument coordinates to sprite-local coordinates
             * by subtracting the sprite's origin (ox, oy). The +0.5 is for
             * nearest-neighbor rounding.
             */
            int lx=(int)(sx_f-s->ox+0.5f), ly=(int)(sy_f-s->oy+0.5f);

            /* Bounds check: is this point within the sprite rectangle? */
            if((unsigned)lx>=(unsigned)s->w||(unsigned)ly>=(unsigned)s->h) continue;

            /* Read the ARGB1555 pixel from the sprite */
            uint16_t p=s->px[ly*s->w+lx];

            /* Check 1-bit alpha: bit 15. If 0, pixel is transparent — skip. */
            if(!(p&0x8000)) continue;

            /*
             * Decode ARGB1555 to ARGB8888 and write directly to framebuffer:
             *   Red:   bits 14..10, shifted left 3 to expand 5-bit to 8-bit
             *   Green: bits 9..5,   shifted left 3
             *   Blue:  bits 4..0,   shifted left 3
             *   Alpha: always 0xFF (fully opaque, since we passed the alpha test)
             */
            fb[dy*WIN_W+dx] = 0xFF000000u |
                ((uint32_t)(((p>>10)&0x1F)<<3)<<16) |
                ((uint32_t)(((p>>5)&0x1F)<<3)<<8) |
                (uint32_t)((p&0x1F)<<3);
        }
    }
}

/* ----------------------------------------------------------------------
 * blit_sprite — draw a non-rotated ARGB1555 sprite at its native position
 * ----------------------------------------------------------------------
 * Used for the lubber line (aircraft symbol) which is always fixed at the
 * top of the instrument and does not rotate. This is a simple scanline
 * blit with 1-bit alpha keying — much faster than rotate_blend_sprite
 * since no trigonometry is involved.
 *
 * The sprite's (ox, oy) gives its screen position. Each pixel is decoded
 * from ARGB1555 to ARGB8888 the same way as in rotate_blend_sprite.
 */
static void blit_sprite(uint32_t *fb, const Sprite *s)
{
    for (int y=0; y<s->h; y++) {
        int dy=s->oy+y; if((unsigned)dy>=DISP_H) continue;
        for (int x=0; x<s->w; x++) {
            int dx=s->ox+x; if((unsigned)dx>=DISP_W) continue;
            uint16_t p=s->px[y*s->w+x]; if(!(p&0x8000)) continue;  /* Alpha test */
            fb[dy*WIN_W+dx] = 0xFF000000u |
                ((uint32_t)(((p>>10)&0x1F)<<3)<<16) |
                ((uint32_t)(((p>>5)&0x1F)<<3)<<8) |
                (uint32_t)((p&0x1F)<<3);
        }
    }
}

/* ======================================================================
 * BUTTON HIT TESTING
 * ======================================================================
 * The control panel has 8 buttons total: 2 per parameter group (left
 * and right arrows). Buttons are indexed 0..7:
 *   Button 0 = HDG left   Button 1 = HDG right
 *   Button 2 = CRS left   Button 3 = CRS right
 *   Button 4 = DEV left   Button 5 = DEV right
 *   Button 6 = BUG left   Button 7 = BUG right
 *
 * Group index = button_index / 2
 * Is-right   = button_index & 1
 */
#define N_BUTTONS (N_CONTROLS * 2)  /* 8 buttons total */

/* ----------------------------------------------------------------------
 * hit_test — determine which button (if any) contains the point (mx, my)
 * ----------------------------------------------------------------------
 * Returns the button index 0..7, or -1 if no button was hit.
 * Tests each button's bounding rectangle in sequence.
 */
static int hit_test(int mx, int my)
{
    for (int i = 0; i < N_BUTTONS; i++) {
        int g = i/2, is_r = i&1;   /* Group index and left/right flag */
        int bx = is_r ? BTN_R_X(g) : BTN_L_X(g);
        if (mx >= bx && mx < bx+BTN_W && my >= BTN_Y && my < BTN_Y+BTN_H)
            return i;
    }
    return -1;  /* No button hit */
}

/* ======================================================================
 * MAIN — program entry point
 * ======================================================================
 * Initializes Winsock for UDP networking, loads all sprite/layer assets,
 * creates the SDL2 window and renderer, then enters the main loop.
 *
 * The main loop:
 *   1. Processes SDL events (keyboard, mouse)
 *   2. Updates instrument state (heading, course, CDI, heading bug)
 *   3. Sends a UDP packet to the Pi at ~30 Hz
 *   4. Renders the instrument face by compositing layers
 *   5. Renders the control panel
 *   6. Presents the framebuffer to the screen via SDL
 */
int main(int argc, char *argv[])
{
    /* Default Raspberry Pi IP address; can be overridden by command-line argument */
    const char *pi_ip = "192.168.1.129";
    if (argc > 1) pi_ip = argv[1];

    /* ==================================================================
     * WINSOCK INITIALIZATION + UDP SOCKET SETUP
     * ==================================================================
     * Windows requires WSAStartup() before any socket calls. We request
     * Winsock version 2.2 (the standard modern version).
     *
     * We create a SOCK_DGRAM (UDP) socket. UDP is connectionless and
     * fire-and-forget — we just sendto() each packet without establishing
     * a connection. This is ideal for real-time instrument data where
     * occasional packet loss is acceptable (the next packet will arrive
     * 33ms later with fresh data anyway).
     *
     * The sockaddr_in structure is filled with the Pi's IP address and
     * port. inet_addr() converts the dotted-decimal string "192.168.1.129"
     * to a 32-bit network-byte-order address. htons() converts the port
     * number to network byte order (big-endian).
     */
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2,2), &wsa) != 0) {
        fprintf(stderr, "WSAStartup failed\n"); return 1;
    }
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
        fprintf(stderr, "socket() failed\n"); return 1;
    }
    struct sockaddr_in pi_addr;
    memset(&pi_addr, 0, sizeof(pi_addr));
    pi_addr.sin_family = AF_INET;              /* IPv4 */
    pi_addr.sin_port = htons(PI_PORT);         /* Port 5555, network byte order */
    pi_addr.sin_addr.s_addr = inet_addr(pi_ip);/* Pi's IP address */

    printf("=== HSI Host ===\n");
    printf("Sending UDP to %s:%d\n\n", pi_ip, PI_PORT);

    /* ==================================================================
     * LOAD INSTRUMENT ASSETS
     * ==================================================================
     * All assets live in ../HSI/ relative to the simulator executable.
     * They were pre-rendered from vector artwork and exported in the
     * pixel formats used by the STM32's DMA2D hardware.
     *
     * Full-frame layers (480x480 = 230,400 bytes each):
     *   background.bin    — L8 (8-bit grayscale) bezel and face markings
     *   compass_card.bin  — AL44 (4-bit alpha + 4-bit luminance) heading ring
     *                       with degree markings and cardinal letters
     *   cdi_dots.bin      — AL44 CDI deviation scale dots
     *
     * Sprite overlays (ARGB1555 with 8-byte header):
     *   course_pointer.bin — the long course arrow through the center
     *   cdi_bar.bin        — the narrow CDI deviation bar
     *   heading_bug.bin    — the small heading bug triangle on the compass rim
     *   lubber_aircraft.bin — the fixed aircraft/lubber symbol at the top
     */
    uint8_t *bg_l8 = load_bin("../HSI/background.bin", DISP_W * DISP_H);
    if (!bg_l8) return 1;
    uint8_t *compass_al44 = load_bin("../HSI/compass_card.bin", DISP_W * DISP_H);
    if (!compass_al44) return 1;
    uint8_t *cdi_dots_al44 = load_bin("../HSI/cdi_dots.bin", DISP_W * DISP_H);
    if (!cdi_dots_al44) return 1;

    Sprite course_ptr, cdi_bar, hdg_bug, lubber;
    if (load_sprite("../HSI/course_pointer.bin", &course_ptr)) return 1;
    if (load_sprite("../HSI/cdi_bar.bin",        &cdi_bar))    return 1;
    if (load_sprite("../HSI/heading_bug.bin",    &hdg_bug))    return 1;
    if (load_sprite("../HSI/lubber_aircraft.bin", &lubber))     return 1;

    /* ==================================================================
     * FRAMEBUFFER ALLOCATION
     * ==================================================================
     * Allocate a single ARGB8888 framebuffer covering the full window
     * (480 x 590 = 283,200 pixels x 4 bytes = ~1.1 MB). This is where
     * all rendering happens — both the instrument face and the control
     * panel are drawn here, then the whole buffer is uploaded to an
     * SDL texture for display.
     */
    uint32_t *fb = calloc(WIN_W * WIN_H, sizeof(uint32_t));
    if (!fb) return 1;

    /* ==================================================================
     * SDL2 INITIALIZATION
     * ==================================================================
     * Create a window and hardware-accelerated renderer with VSync.
     * SDL_RenderSetLogicalSize ensures the content scales correctly
     * if the window is resized. The streaming texture is updated every
     * frame from our software framebuffer — this is a "software rendering
     * to hardware texture" approach, same paradigm as the STM32's LTDC
     * writing to display RAM.
     */
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window *win = SDL_CreateWindow("HSI Host",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIN_W, WIN_H, 0);
    SDL_Renderer *ren = SDL_CreateRenderer(win, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    SDL_RenderSetLogicalSize(ren, WIN_W, WIN_H);
    SDL_Texture *tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING, WIN_W, WIN_H);

    /* ==================================================================
     * INSTRUMENT STATE VARIABLES
     * ==================================================================
     * These four floats fully describe the HSI's visual state:
     *
     *   heading     — aircraft magnetic heading in degrees [0, 360).
     *                 Determines compass card rotation.
     *   course      — selected VOR/ILS course (OBS knob) in degrees [0, 360).
     *                 Determines course pointer and CDI dots rotation.
     *   hdg_bug_deg — heading bug position in degrees [0, 360).
     *                 The small triangle on the compass ring.
     *   cdi_dev     — CDI bar lateral offset in PIXELS [-75, +75].
     *                 Internally this is pixels; it is converted to "dots"
     *                 (dividing by 25) when sent to the Pi and when
     *                 displayed in the GUI panel. One dot = 25 pixels.
     *
     * auto_hdg / auto_cdi control demo animations:
     *   auto_hdg — when true, heading slowly sweeps (6 deg/sec)
     *   auto_cdi — when true, CDI oscillates sinusoidally between -50 and +50 px
     */
    float heading = 0, course = 45, hdg_bug_deg = 90, cdi_dev = 0;
    int auto_hdg = 1, auto_cdi = 1;

    /*
     * GUI state for mouse button interaction.
     * pressed_btn tracks which panel button is currently held down (-1 = none).
     * press_start records when the button was first pressed (for repeat delay).
     * initial_step_done prevents the first click from also triggering the
     * held-down continuous adjustment.
     */
    int pressed_btn = -1;
    Uint32 press_start = 0;
    int initial_step_done = 0;

    /* UDP send timing — we throttle to ~30 Hz regardless of frame rate */
    Uint32 last_send = 0;
    int packets_sent = 0;

    /* Labels for the four parameter groups in the control panel */
    static const char *labels[N_CONTROLS] = { "HDG", "CRS", "DEV", "BUG" };

    int running = 1;
    Uint32 last_tick = SDL_GetTicks();

    /* ==================================================================
     * MAIN LOOP
     * ==================================================================
     * Runs at the display's refresh rate (typically 60 Hz due to VSync).
     * Each iteration:
     *   1. Compute delta time for smooth animation
     *   2. Process input events (keyboard + mouse)
     *   3. Apply button actions (click and hold-to-repeat)
     *   4. Apply keyboard continuous input
     *   5. Run auto-animations if enabled
     *   6. Wrap/clamp all values to valid ranges
     *   7. Send UDP packet to Pi (throttled to ~30 Hz)
     *   8. Compute rotation angles
     *   9. Render instrument layers in compositing order
     *  10. Render GUI control panel
     *  11. Upload framebuffer to SDL texture and present
     */
    while (running) {
        /* ---- Delta time calculation ---- */
        Uint32 now = SDL_GetTicks();
        float dt = (now - last_tick) / 1000.0f;  /* Seconds since last frame */
        if (dt > 0.1f) dt = 0.1f;  /* Clamp to prevent huge jumps after stalls */
        last_tick = now;

        /* ---- Mouse hover state ---- */
        int mx, my;
        SDL_GetMouseState(&mx, &my);
        int hovered_btn = hit_test(mx, my);  /* Which button is the mouse over? */

        /* ---- Event processing ---- */
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            switch (ev.type) {
            case SDL_QUIT: running = 0; break;  /* Window close button */
            case SDL_KEYDOWN:
                switch (ev.key.keysym.sym) {
                case SDLK_ESCAPE: running = 0; break;           /* Quit */
                case SDLK_SPACE:  auto_hdg = !auto_hdg; break;  /* Toggle auto heading sweep */
                case SDLK_TAB:    auto_cdi = !auto_cdi; break;  /* Toggle auto CDI oscillation */
                case SDLK_r:      /* Reset all values to defaults */
                    heading=0; course=45; hdg_bug_deg=90;
                    cdi_dev=0; auto_hdg=1; auto_cdi=1; break;
                default: break;
                } break;
            case SDL_MOUSEBUTTONDOWN:
                /* Record which panel button was clicked and when */
                if (ev.button.button == SDL_BUTTON_LEFT) {
                    pressed_btn = hit_test(ev.button.x, ev.button.y);
                    press_start = now; initial_step_done = 0;
                } break;
            case SDL_MOUSEBUTTONUP:
                if (ev.button.button == SDL_BUTTON_LEFT) pressed_btn = -1;
                break;
            }
        }

        /* ==============================================================
         * BUTTON ACTIONS (click + hold-to-repeat)
         * ==============================================================
         * When a panel button is held down:
         *   - On first press: apply a single discrete step (e.g., +1 degree)
         *   - After 400ms hold: apply continuous adjustment scaled by dt
         *
         * The direction is determined by the button index:
         *   Even index = left button = decrement (dir = -1)
         *   Odd index  = right button = increment (dir = +1)
         *
         * The group determines which parameter to adjust:
         *   0 = heading (also disables auto-heading)
         *   1 = course
         *   2 = CDI deviation (larger step: 5 px per click, 75 px/sec held)
         *   3 = heading bug
         */
        if (pressed_btn >= 0) {
            int dir = (pressed_btn&1) ? 1 : -1;  /* Left = -1, Right = +1 */
            int group = pressed_btn/2;
            float *target = NULL;
            float step=1, cont=45*dt;  /* Default: 1 deg/click, 45 deg/sec held */
            switch (group) {
            case 0: target=&heading;     auto_hdg=0; break;
            case 1: target=&course;      break;
            case 2: target=&cdi_dev;     step=5; cont=75*dt; auto_cdi=0; break;
            case 3: target=&hdg_bug_deg; break;
            }
            if (target) {
                /* First click: apply one discrete step */
                if (!initial_step_done) { *target+=dir*step; initial_step_done=1; }
                /* After 400ms hold delay: continuous adjustment */
                else if (now-press_start>400) *target+=dir*cont;
            }
        }

        /* ==============================================================
         * KEYBOARD CONTINUOUS INPUT
         * ==============================================================
         * SDL_GetKeyboardState returns a snapshot of all keys. Held keys
         * produce smooth continuous changes scaled by dt (frame time).
         * Rates: 30 deg/sec for heading/course/bug, 50 px/sec for CDI.
         */
        const Uint8 *keys = SDL_GetKeyboardState(NULL);
        if (keys[SDL_SCANCODE_LEFT])  { heading-=30*dt; auto_hdg=0; }
        if (keys[SDL_SCANCODE_RIGHT]) { heading+=30*dt; auto_hdg=0; }
        if (keys[SDL_SCANCODE_UP])    course-=30*dt;
        if (keys[SDL_SCANCODE_DOWN])  course+=30*dt;
        if (keys[SDL_SCANCODE_Q])     hdg_bug_deg-=30*dt;
        if (keys[SDL_SCANCODE_W])     hdg_bug_deg+=30*dt;
        if (keys[SDL_SCANCODE_A])     { cdi_dev-=50*dt; auto_cdi=0; }
        if (keys[SDL_SCANCODE_D])     { cdi_dev+=50*dt; auto_cdi=0; }

        /* ==============================================================
         * AUTO-ANIMATIONS
         * ==============================================================
         * When enabled, these provide a hands-off demo mode:
         *   auto_hdg: heading rotates at 6 deg/sec (one full revolution
         *             every 60 seconds).
         *   auto_cdi: CDI deviation follows a triangle wave with an
         *             8-second period, sweeping from -50 to +50 pixels
         *             (i.e., -2 to +2 dots). The triangle wave is
         *             computed with a piecewise linear function:
         *               t in [0,4): cdi = -50 + 25*t  (ramps up)
         *               t in [4,8): cdi = 150 - 25*t  (ramps down)
         */
        if (auto_hdg) heading += 6.0f * dt;
        if (auto_cdi) {
            float t = fmodf(now/1000.0f, 8.0f);  /* Sawtooth 0..8 seconds */
            cdi_dev = (t<4) ? -50+25*t : 150-25*t;  /* Triangle wave */
        }

        /* ==============================================================
         * WRAP AND CLAMP
         * ==============================================================
         * Angular values wrap at 360 degrees (they are circular).
         * CDI deviation is clamped to +/-75 pixels (+/-3 dots), which is
         * the physical limit of the CDI bar travel on a real KI-525A.
         */
        heading=fmodf(heading,360); if(heading<0) heading+=360;
        course=fmodf(course,360);   if(course<0)  course+=360;
        hdg_bug_deg=fmodf(hdg_bug_deg,360); if(hdg_bug_deg<0) hdg_bug_deg+=360;
        if(cdi_dev>75) cdi_dev=75; if(cdi_dev<-75) cdi_dev=-75;

        /* ==============================================================
         * SEND UDP PACKET TO RASPBERRY PI (~30 Hz)
         * ==============================================================
         * Throttled by SEND_INTERVAL (33ms). The packet contains the raw
         * instrument state. Note that cdi_dev is converted from pixels to
         * dots (divide by 25.0) for the network protocol — the Pi
         * receiver converts back to pixels internally.
         *
         * sendto() fires the packet off to the Pi's IP:port. Since UDP is
         * connectionless, there is no error if the Pi is unreachable —
         * the packet just disappears. This is fine for real-time data.
         */
        if (now - last_send >= SEND_INTERVAL) {
            HsiPacket pkt;
            pkt.instrument_id = INSTRUMENT_HSI;
            pkt.heading = heading;
            pkt.course = course;
            pkt.heading_bug = hdg_bug_deg;
            pkt.cdi_dev = cdi_dev / 25.0f;   /* convert pixels to dots */
            sendto(sock, (const char *)&pkt, sizeof(pkt), 0,
                   (struct sockaddr *)&pi_addr, sizeof(pi_addr));
            last_send = now;
            packets_sent++;
        }

        /* ==============================================================
         * COMPUTE ROTATION ANGLES
         * ==============================================================
         * All rotations are relative to the aircraft's heading:
         *
         *   card_rot — compass card rotates OPPOSITE to heading. If the
         *     aircraft heading is 090, the compass card must rotate -90
         *     degrees so that "090" appears under the lubber line at top.
         *
         *   crs_rot — the course pointer and CDI assembly rotate by
         *     (course - heading). If the pilot selects course 180 and the
         *     aircraft heading is 090, the pointer appears at 180-090 = 90
         *     degrees clockwise from the top (pointing right on screen).
         *
         *   bug_rot — the heading bug rotates by (bug - heading), just
         *     like the course pointer but tracking the heading bug setting.
         */
        float card_rot = -heading;
        float crs_rot  = course - heading;
        float bug_rot  = hdg_bug_deg - heading;

        /* ==============================================================
         * RENDER INSTRUMENT FACE — LAYER COMPOSITING ORDER
         * ==============================================================
         * Each layer is drawn on top of the previous one, exactly like
         * the STM32's DMA2D would composite them. The order matters:
         * later layers obscure earlier ones.
         *
         * Layer 1: Background (L8 grayscale)
         *   The static bezel artwork — always drawn first, fills the
         *   entire 480x480 area. No rotation.
         *
         * Layer 2: CDI dots (AL44, rotated by crs_rot)
         *   The five deviation-scale dots that rotate with the course
         *   pointer. They show the pilot the CDI scale orientation.
         *
         * Layer 3: Compass card (AL44, rotated by card_rot = -heading)
         *   The 360-degree heading ring with tick marks and cardinal
         *   letters (N, E, S, W). Rotates opposite to aircraft heading
         *   so the current heading appears under the lubber line.
         *
         * Layer 4: Heading bug (ARGB1555 sprite, rotated by bug_rot)
         *   A small triangle on the compass card rim marking the pilot's
         *   desired heading. tx=0, ty=0 (no lateral translation).
         *
         * Layer 5: Course pointer (ARGB1555 sprite, rotated by crs_rot)
         *   The long arrow through the center showing the selected VOR/ILS
         *   course. tx=0, ty=0 (centered, no translation).
         *
         * Layer 6: CDI bar (ARGB1555 sprite, rotated by crs_rot, tx=cdi_dev)
         *   *** THIS IS THE KEY LAYER ***
         *   The narrow deviation bar that slides laterally. It uses the
         *   same rotation as the course pointer (crs_rot) but with a
         *   non-zero tx = cdi_dev. This tx is applied in source space
         *   BEFORE rotation, causing the bar to move perpendicular to
         *   the course pointer regardless of the pointer's angle.
         *   See the detailed explanation in rotate_blend_sprite above.
         *
         * Layer 7: Lubber line / aircraft symbol (ARGB1555 sprite, no rotation)
         *   The fixed reference mark at the top of the instrument that
         *   represents the aircraft's nose. Always points straight up.
         *   Drawn last so it appears on top of everything.
         */
        blit_background(fb, bg_l8);                                  /* Layer 1 */
        rotate_blend_al44(fb, cdi_dots_al44, crs_rot);              /* Layer 2 */
        rotate_blend_al44(fb, compass_al44, card_rot);               /* Layer 3 */
        rotate_blend_sprite(fb, &hdg_bug, bug_rot, 0, 0);           /* Layer 4 */
        rotate_blend_sprite(fb, &course_ptr, crs_rot, 0, 0);        /* Layer 5 */
        rotate_blend_sprite(fb, &cdi_bar, crs_rot, cdi_dev, 0);     /* Layer 6 */
        blit_sprite(fb, &lubber);                                    /* Layer 7 */

        /* ==============================================================
         * RENDER CONTROL PANEL
         * ==============================================================
         * The panel sits below the instrument face (y >= DISP_H) and
         * shows four parameter groups: HDG, CRS, DEV, BUG.
         *
         * Each group has:
         *   - A gray label (e.g., "HDG")
         *   - A green numeric readout (e.g., "045")
         *   - Left and right arrow buttons for adjustment
         *
         * Vertical separator lines divide the groups. A horizontal
         * separator line sits at the top edge of the panel.
         */
        fill_rect(fb, 0, DISP_H, WIN_W, PANEL_H, COL_PANEL_BG);  /* Panel background */
        fill_rect(fb, 0, DISP_H, WIN_W, 1, COL_SEPARATOR);        /* Top border line */
        /* Vertical dividers between groups */
        for (int g=1; g<N_CONTROLS; g++)
            fill_rect(fb, g*GROUP_W, DISP_H+4, 1, PANEL_H-8, COL_SEPARATOR);

        /* Draw each parameter group */
        for (int g=0; g<N_CONTROLS; g++) {
            int gcx = g*GROUP_W + GROUP_W/2;  /* Horizontal center of this group */

            /* Parameter label (e.g., "HDG") */
            draw_str_cx(fb, gcx, LABEL_Y, labels[g], COL_LABEL);

            /* Format and draw the numeric value */
            char val[16];
            switch(g) {
            case 0: snprintf(val,sizeof(val),"%03.0f",heading); break;     /* "045" */
            case 1: snprintf(val,sizeof(val),"%03.0f",course); break;      /* "180" */
            case 2: snprintf(val,sizeof(val),"%+.1f",cdi_dev/25.0f); break;/* "+1.2" dots */
            case 3: snprintf(val,sizeof(val),"%03.0f",hdg_bug_deg); break; /* "090" */
            }
            draw_str_cx(fb, gcx, VALUE_Y, val, COL_VALUE);

            /* Left arrow button (decrement) */
            draw_button(fb, BTN_L_X(g), BTN_Y, -1,
                        hovered_btn==g*2, pressed_btn==g*2);
            /* Right arrow button (increment) */
            draw_button(fb, BTN_R_X(g), BTN_Y,  1,
                        hovered_btn==g*2+1, pressed_btn==g*2+1);
        }

        /* ==============================================================
         * PRESENT FRAME
         * ==============================================================
         * Upload the software framebuffer to the SDL texture, then copy
         * the texture to the screen. SDL_RenderPresent flips the display
         * and (because of PRESENTVSYNC) blocks until the next VSync,
         * giving us a steady ~60 FPS frame rate.
         *
         * The pitch parameter (WIN_W * 4) tells SDL the byte stride
         * between rows in our framebuffer (480 pixels * 4 bytes/pixel).
         */
        SDL_UpdateTexture(tex, NULL, fb, WIN_W*4);
        SDL_RenderCopy(ren, tex, NULL, NULL);
        SDL_RenderPresent(ren);
    }

    /* ==================================================================
     * CLEANUP
     * ==================================================================
     * Close the UDP socket and shut down Winsock. Destroy SDL resources
     * in reverse order of creation. Free all allocated memory.
     */
    closesocket(sock);
    WSACleanup();
    SDL_DestroyTexture(tex); SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win); SDL_Quit();
    free(fb); free(bg_l8); free(compass_al44); free(cdi_dots_al44);
    free(course_ptr.px); free(cdi_bar.px); free(hdg_bug.px); free(lubber.px);
    return 0;
}
