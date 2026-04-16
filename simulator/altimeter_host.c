/*
 * Altimeter Host — runs on PC, sends UDP to Raspberry Pi receiver
 * 480x480 AAU-19/A altimeter with GUI control panel
 *
 * Usage: altimeter_host.exe [pi_ip]
 *        Default IP: 192.168.1.129, port 5555
 *
 * Controls:
 *   Up/Down arrows — baro pressure
 *   Mouse buttons  — ALT and BARO in panel
 *   SPACE          — toggle auto altitude animation
 *   R              — reset
 *   ESC            — quit
 *
 * =====================================================================
 *  HOW THIS SIMULATOR WORKS — OVERVIEW
 * =====================================================================
 *
 *  The AAU-19/A altimeter displays altitude using two independent
 *  visual systems that work together:
 *
 *  1. A ROTATING POINTER that sweeps one full revolution per 1000 ft.
 *     This gives fine-grained altitude readout within a 1000-ft band.
 *
 *  2. A set of DRUM COUNTER WHEELS (like an odometer) that show the
 *     100s, 1000s, and 10000s digits as vertically-scrolling number
 *     strips.  These drums use a "Geneva drive" style carry mechanism:
 *     when a lower-order digit rolls from 9 through 0, the next higher
 *     digit doesn't snap — it smoothly transitions over that last 10%
 *     of travel, giving a realistic mechanical feel.
 *
 *  Additionally, there is a BAROMETRIC PRESSURE DRUM (Kollsman window)
 *  that displays the altimeter setting in inches of mercury (inHg),
 *  using the same Geneva-drive carry logic.  The baro drum glyphs are
 *  dynamically extracted from the 100s digit strip artwork at startup.
 *
 *  The simulator renders everything into a 480x480 ARGB8888 framebuffer,
 *  composites the pointer with per-pixel alpha blending, and streams
 *  the altitude + baro values to a Raspberry Pi over UDP at ~30 Hz
 *  so the Pi can drive a physical HDMI display in real time.
 *
 * =====================================================================
 *  PIXEL FORMAT PIPELINE
 * =====================================================================
 *
 *  The raw artwork assets are stored as compact 16-bit formats to match
 *  the STM32 LTDC framebuffer formats used on the real hardware:
 *
 *    - Background dial face:  RGB565   (16-bit, no alpha, 5-6-5 bits)
 *    - Pointer sprite:        ARGB4444 (16-bit, 4-bit alpha channel)
 *    - Drum digit strips:     RGB565   (opaque digit artwork)
 *
 *  At load time, these are upconverted to ARGB8888 (32-bit) for the
 *  SDL display pipeline.  The upconversion replicates the high bits
 *  into the low bits (e.g., 5-bit value 0x1F becomes 0xFF) to get
 *  a full-range 8-bit result without dark banding artifacts.
 */
#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <winsock2.h>

#pragma comment(lib, "ws2_32.lib")

/* ---- UDP packet ---- */
/*
 * The AltPacket structure is the network protocol between this host
 * simulator and the Raspberry Pi HDMI receiver.  It is sent as a raw
 * binary UDP datagram — no marshalling or byte-order conversion,
 * since both the PC host and the Pi are little-endian ARM/x86.
 *
 * instrument_id:  Identifies which instrument this packet is for.
 *                 The Pi receiver can drive multiple instruments on
 *                 a single HDMI display (altimeter, VSI, etc.).
 * altitude:       Current indicated altitude in feet (0–99999).
 * baro_inhg:      Current barometric pressure setting in inches Hg
 *                 (typically 28.10–31.00).
 */
typedef struct {
    uint32_t instrument_id;
    float    altitude;
    float    baro_inhg;
} AltPacket;
#define INSTRUMENT_ALT  1       /* Instrument ID for the altimeter */
#define PI_PORT         5555    /* UDP destination port on the Pi */
#define SEND_INTERVAL   33      /* ~30 Hz send rate (33 ms between packets) */

/* ---- Display geometry ---- */
/*
 * The instrument face is 480x480 pixels — a square that matches the
 * round altimeter bezel.  Below it is a 110-pixel-tall control panel
 * strip with clickable buttons for adjusting altitude and baro.
 * The full SDL window is therefore 480 wide x 590 tall.
 *
 * CX, CY define the center of the dial face, which is the rotation
 * pivot point for the pointer needle.
 */
#define DISP_W    480
#define DISP_H    480
#define PANEL_H   110
#define WIN_W     DISP_W
#define WIN_H     (DISP_H + PANEL_H)
#define CX        (DISP_W / 2)         /* 240 — horizontal center */
#define CY        (DISP_H / 2)         /* 240 — vertical center   */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---- Panel layout ---- */
/*
 * The control panel is divided into N_CONTROLS groups (ALT and BARO),
 * each GROUP_W pixels wide.  Each group has a label, a numeric value
 * display, and left/right arrow buttons for decrement/increment.
 *
 * BTN_L_X(g) and BTN_R_X(g) compute the x-position of the left and
 * right buttons for group g, providing symmetric padding from the
 * group edges.
 */
#define N_CONTROLS  2
#define GROUP_W     (WIN_W / N_CONTROLS)    /* 240 px per group */
#define BTN_W       32
#define BTN_H       28
#define BTN_Y       (DISP_H + 58)
#define LABEL_Y     (DISP_H + 14)
#define VALUE_Y     (DISP_H + 38)
#define BTN_L_X(g)  ((g) * GROUP_W + 30)
#define BTN_R_X(g)  ((g) * GROUP_W + GROUP_W - 30 - BTN_W)

/* ---- Panel colors ---- */
/*
 * All panel colors are specified as 0xAARRGGBB (ARGB8888).
 * The panel uses a dark theme to keep visual focus on the instrument.
 */
#define COL_PANEL_BG    0xFF1A1A1Au     /* Near-black panel background */
#define COL_SEPARATOR   0xFF333333u     /* Subtle divider lines */
#define COL_BTN_NORMAL  0xFF3A3A3Au     /* Button default state */
#define COL_BTN_HOVER   0xFF505050u     /* Button hover (mouse over) */
#define COL_BTN_PRESS   0xFF686868u     /* Button pressed (mouse down) */
#define COL_ARROW       0xFFDDDDDDu     /* Arrow glyph inside buttons */
#define COL_LABEL       0xFF888888u     /* Group label text ("ALT", "BARO") */
#define COL_VALUE       0xFF44DD44u     /* Numeric readout — green for visibility */

/* ---- Drum constants ---- */
/*
 * DRUM COUNTER WHEEL SYSTEM
 * =========================
 *
 * The altimeter drum display works like a mechanical odometer:
 * each digit (0-9) is painted onto a vertical "strip" bitmap,
 * arranged sequentially from top to bottom.  To display a particular
 * value, we scroll the strip up or down so that the desired digit
 * is centered in the visible window.
 *
 * The 100s drum uses a strip that is 28 pixels wide and 567 pixels
 * tall, containing 10 digits (0-9).  Each digit cell is therefore
 * 567/10 = 56.7 pixels tall.  The visible portion is a 104-pixel
 * tall window in the instrument face, roughly centered on the
 * current digit.
 *
 * The 1000s and 10000s drums use a wider strip (45 px) with slightly
 * different geometry.  The 1000s strip has a 53-pixel hatch pattern
 * at the top that is skipped during rendering (it's a crosshatch
 * that appears below zero in the real instrument).
 *
 * DRUM_X, DRUM_Y define where the 100s drum window sits on the
 * instrument face.  DRUM1K_X and DRUM10K_X are computed relative
 * to DRUM_X, placing the higher-order drums to the left.
 */
#define STRIP_WIDTH   28            /* 100s digit strip: pixel width */
#define STRIP_HEIGHT  567           /* 100s digit strip: total pixel height */
#define STRIP_DIGITS  10            /* Number of digits on each strip (0-9) */
#define DIGIT_HEIGHT  ((float)STRIP_HEIGHT / STRIP_DIGITS)  /* ~56.7 px per digit cell */
#define DRUM_X        164           /* 100s drum window X position on face */
#define DRUM_Y        174           /* 100s drum window Y position on face */
#define DRUM_VIS_H    104           /* 100s drum visible height (pixels) */
#define STRIP1K_WIDTH   45          /* 1000s/10000s digit strip: pixel width */
#define STRIP1K_FULL_H  583         /* 1000s strip total height including hatch */
#define STRIP1K_HEIGHT  530         /* 1000s strip usable height (after hatch) */
#define STRIP1K_HATCH   53          /* Height of crosshatch region at top of 1000s strip */
#define DIGIT1K_HEIGHT  53.0f       /* Pixel height of each digit cell in 1000s strip */
#define DRUM1K_X        (DRUM_X - 10 - STRIP1K_WIDTH)  /* 1000s drum: left of 100s drum */
#define DRUM10K_X       (DRUM1K_X - STRIP1K_WIDTH)     /* 10000s drum: left of 1000s drum */
#define DRUM1K_Y        (DRUM_Y + (DRUM_VIS_H - DRUM1K_VIS_H) / 2)  /* Vertically centered */
#define DRUM1K_VIS_H    51          /* 1000s/10000s drum visible height (smaller window) */
#define BARO_GAP 0
#define BARO_Y   280                /* Kollsman window Y position (baro drum) */

/* ---- Pointer ---- */
/*
 * The pointer needle sprite is 63 x 240 pixels, stored as ARGB4444
 * (16-bit with alpha).  The alpha channel allows smooth anti-aliased
 * edges when composited over the dial face.
 *
 * PIVOT_X, PIVOT_Y define the rotation pivot within the pointer
 * sprite — the point that aligns with the center of the dial (CX,CY).
 * The pivot is near the top of the sprite (Y=50), well above center,
 * because the pointer is a long needle that extends mostly downward
 * from the hub, with just a small counterweight above.
 */
#define PTR_WIDTH   63
#define PTR_HEIGHT  240
#define PIVOT_X     32
#define PIVOT_Y     36

/* Screen-space rotation center for the pointer.  This is where the pointer's
 * pivot (PIVOT_X, PIVOT_Y) gets placed on the dial.  The dial face is square
 * but the round bezel artwork places the visual hub a bit above geometric
 * center, so we offset Y upward. */
#define PIVOT_SCREEN_X  (CX - 5)
#define PIVOT_SCREEN_Y  (CY - 10)

/* ================================================================== */
/*  5x7 bitmap font                                                   */
/* ================================================================== */
/*
 * A minimal monospaced bitmap font for the control panel readouts.
 * Each glyph is 5 pixels wide by 7 pixels tall, stored as an array
 * of 7 bytes where each bit represents a pixel (MSB = leftmost).
 * FONT_SCALE of 2 doubles the rendering size to 10x14 for legibility.
 * CHAR_W includes a 1-pixel inter-character gap (scaled to 2 pixels).
 *
 * Only the characters needed for the panel are defined: digits 0-9,
 * letters A/B/L/O/R/T (for "ALT" and "BARO"), decimal point, and space.
 */
#define FONT_W  5
#define FONT_H  7
#define FONT_SCALE  2
#define CHAR_W  ((FONT_W + 1) * FONT_SCALE)    /* 12 px total width per character */
#define CHAR_H  (FONT_H * FONT_SCALE)          /* 14 px total height */

static const struct { char ch; uint8_t r[7]; } glyphs[] = {
    /*  Each row byte encodes 5 pixels: bit4=leftmost, bit0=rightmost.
     *  For example, '0' row 0 is 0x0E = 01110 binary, drawing the
     *  top arc of the zero with the center 3 pixels lit.              */
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
    {'L', {0x10,0x10,0x10,0x10,0x10,0x10,0x1F}},
    {'O', {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}},
    {'R', {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}},
    {'T', {0x1F,0x04,0x04,0x04,0x04,0x04,0x04}},
    {'.', {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C}},
    {' ', {0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
};
#define N_GLYPHS ((int)(sizeof(glyphs)/sizeof(glyphs[0])))

/*
 * draw_char — render a single character from the bitmap font
 *
 * Looks up the character 'c' in the glyph table, then paints it
 * pixel-by-pixel into the framebuffer at position (x, y).
 * Each font pixel is expanded to a FONT_SCALE x FONT_SCALE block
 * (2x2 by default) so the text is legible on screen.
 *
 * The bit test (0x10 >> col) walks from MSB to LSB across the
 * 5-bit row, treating bit 4 as the leftmost pixel column.
 */
static void draw_char(uint32_t *fb, int x, int y, char c, uint32_t color)
{
    const uint8_t *bits = NULL;
    for (int i = 0; i < N_GLYPHS; i++)
        if (glyphs[i].ch == c) { bits = glyphs[i].r; break; }
    if (!bits) return;
    for (int row = 0; row < FONT_H; row++)
        for (int col = 0; col < FONT_W; col++)
            if (bits[row] & (0x10 >> col))
                for (int sy = 0; sy < FONT_SCALE; sy++)
                    for (int sx = 0; sx < FONT_SCALE; sx++) {
                        int px = x + col*FONT_SCALE + sx;
                        int py = y + row*FONT_SCALE + sy;
                        if ((unsigned)px < WIN_W && (unsigned)py < WIN_H)
                            fb[py * WIN_W + px] = color;
                    }
}

/* draw_str — render a null-terminated string left-aligned at (x, y) */
static void draw_str(uint32_t *fb, int x, int y, const char *s, uint32_t color)
{ while (*s) { draw_char(fb, x, y, *s++, color); x += CHAR_W; } }

/* draw_str_cx — render a string horizontally centered about cx */
static void draw_str_cx(uint32_t *fb, int cx, int y, const char *s, uint32_t color)
{
    int len = (int)strlen(s);
    int total_w = len * CHAR_W - FONT_SCALE;   /* subtract trailing gap */
    draw_str(fb, cx - total_w/2, y, s, color);
}

/* ================================================================== */
/*  GUI helpers                                                       */
/* ================================================================== */

/* fill_rect — fill an axis-aligned rectangle with a solid color,
 * with bounds checking against the window dimensions.              */
static void fill_rect(uint32_t *fb, int rx, int ry, int rw, int rh, uint32_t c)
{
    for (int y = ry; y < ry + rh; y++) {
        if ((unsigned)y >= WIN_H) continue;
        for (int x = rx; x < rx + rw; x++)
            if ((unsigned)x < WIN_W) fb[y * WIN_W + x] = c;
    }
}

/*
 * draw_arrow — draw a triangular arrow glyph inside a button rectangle.
 *
 * The arrow is drawn row-by-row as a diamond/triangle shape.
 * 'dir' controls direction: negative = pointing left, positive = right.
 * 'pad' insets the arrow from the button edges.
 * Each row's width is proportional to its distance from the center,
 * creating a pointed triangle shape using the formula:
 *   width = total_width * (half - |row - half|) / half
 * which yields 0 at top and bottom, maximum at the vertical midpoint.
 */
static void draw_arrow(uint32_t *fb, int bx, int by, int bw, int bh,
                        int dir, uint32_t color)
{
    int pad = 9, th = bh-2*pad, tw = bw-2*pad;
    if (th < 2 || tw < 2) return;
    int half = th/2;
    for (int row = 0; row < th; row++) {
        int w = tw * (half - abs(row-half)) / half;
        if (w < 1) w = 1;
        int y = by + pad + row;
        /* For left arrow, right-align the triangle; for right arrow, left-align */
        int x0 = (dir < 0) ? bx+bw-pad-w : bx+pad;
        for (int c2 = 0; c2 < w; c2++)
            if ((unsigned)(x0+c2) < WIN_W && (unsigned)y < WIN_H)
                fb[y * WIN_W + x0 + c2] = color;
    }
}

/*
 * draw_button — render a panel button with hover/press visual states.
 * Combines a filled rectangle (background) with a directional arrow.
 */
static void draw_button(uint32_t *fb, int bx, int by, int dir, int hover, int press)
{
    uint32_t bg = press ? COL_BTN_PRESS : hover ? COL_BTN_HOVER : COL_BTN_NORMAL;
    fill_rect(fb, bx, by, BTN_W, BTN_H, bg);
    draw_arrow(fb, bx, by, BTN_W, BTN_H, dir, COL_ARROW);
}

/* ================================================================== */
/*  Button hit testing                                                */
/* ================================================================== */
/*
 * There are N_BUTTONS = N_CONTROLS * 2 = 4 clickable buttons total:
 *   Button 0: ALT decrement (left arrow)
 *   Button 1: ALT increment (right arrow)
 *   Button 2: BARO decrement (left arrow)
 *   Button 3: BARO increment (right arrow)
 *
 * hit_test() returns the button index under the mouse, or -1 if none.
 */
#define N_BUTTONS (N_CONTROLS * 2)

static int hit_test(int mx, int my)
{
    for (int i = 0; i < N_BUTTONS; i++) {
        int g = i/2, is_r = i&1;       /* g = group index, is_r = is right button */
        int bx = is_r ? BTN_R_X(g) : BTN_L_X(g);
        if (mx >= bx && mx < bx+BTN_W && my >= BTN_Y && my < BTN_Y+BTN_H)
            return i;
    }
    return -1;
}

/* ================================================================== */
/*  Altimeter rendering                                               */
/* ================================================================== */

/*
 * rgb565_to_argb8888 — convert RGB565 pixels to ARGB8888
 *
 * RGB565 packs red (5 bits), green (6 bits), blue (5 bits) into 16 bits:
 *   [RRRRR GGGGGG BBBBB]
 *
 * To expand to 8 bits per channel, we first shift the field into the
 * high bits of a byte, then OR in the top bits again into the low bits.
 * For example, a 5-bit red value 0x1F (11111) becomes:
 *   shifted:  11111000 (0xF8)
 *   OR'd:     11111111 (0xFF)  — full white, not dark 0xF8
 *
 * This "bit replication" trick maps the full 5-bit range [0..31] onto
 * the full 8-bit range [0..255] with no gaps or dark bias.
 *
 * Alpha is always set to 0xFF (fully opaque) since RGB565 has no
 * alpha channel.
 */
static void rgb565_to_argb8888(const uint16_t *src, uint32_t *dst, int count)
{
    for (int i = 0; i < count; i++) {
        uint16_t p = src[i];
        uint8_t r = ((p>>11)&0x1F)<<3; r|=r>>5;   /* 5-bit red   -> 8-bit */
        uint8_t g = ((p>>5)&0x3F)<<2;  g|=g>>6;    /* 6-bit green -> 8-bit */
        uint8_t b = (p&0x1F)<<3;       b|=b>>5;    /* 5-bit blue  -> 8-bit */
        dst[i] = 0xFF000000|(r<<16)|(g<<8)|b;
    }
}

/*
 * argb4444_to_argb8888 — convert ARGB4444 pixels to ARGB8888
 *
 * ARGB4444 stores 4 bits each for alpha, red, green, blue:
 *   [AAAA RRRR GGGG BBBB]
 *
 * Each 4-bit nibble is expanded to 8 bits by multiplying by 17 (0x11),
 * which maps 0x0->0x00, 0x8->0x88, 0xF->0xFF evenly.  This is
 * equivalent to duplicating the nibble: 0xA -> 0xAA.
 *
 * The alpha channel is critical for the pointer sprite — it provides
 * smooth anti-aliased edges around the needle shape, allowing clean
 * compositing over the dial face background.
 */
static void argb4444_to_argb8888(const uint16_t *src, uint32_t *dst, int count)
{
    for (int i = 0; i < count; i++) {
        uint16_t p = src[i];
        dst[i] = (((p>>12)&0xF)*17<<24)|(((p>>8)&0xF)*17<<16)|
                 (((p>>4)&0xF)*17<<8)|((p&0xF)*17);
    }
}

/*
 * blit_drum — render a vertically-scrolling drum counter digit
 *
 * This is the core of the mechanical drum counter simulation.
 * Each drum is a vertical strip of artwork containing digits 0-9
 * painted sequentially from top to bottom.  To show a particular
 * value, we compute a vertical scroll offset and blit the visible
 * portion of the strip into a rectangular window on the instrument face.
 *
 * HOW THE SCROLLING WORKS:
 *
 *   The 'value' parameter is a float (0.0 to 9.999...) representing
 *   which digit should be centered in the visible window.  For example:
 *     value=3.0  -> digit "3" is perfectly centered
 *     value=3.5  -> halfway between "3" and "4"
 *     value=9.8  -> near the bottom of "9", about to roll to "0"
 *
 *   y_offset = value * digit_h   — pixel offset into the strip
 *   strip_y_start = y_offset - vis_h/2 + digit_h/2
 *     This formula centers the current digit vertically in the window.
 *     We subtract half the visible height and add half a digit height
 *     so the digit's center aligns with the window's center.
 *
 * WRAPPING (MODULAR ARITHMETIC):
 *
 *   The while loops handle wrap-around:  when the scroll position
 *   goes past the bottom of the strip (past digit 9), it wraps back
 *   to the top (digit 0).  This creates the illusion of an endless
 *   rotating drum cylinder, just like the real mechanical counter.
 *   Without wrapping, digit 9 would scroll into empty space instead
 *   of smoothly transitioning to 0.
 *
 * The RGB565-to-ARGB8888 conversion is done inline per-pixel here
 * (rather than pre-converting the whole strip) because only a small
 * visible slice of the strip is rendered each frame, making inline
 * conversion more efficient than a full upfront conversion.
 *
 * Parameters:
 *   fb       — destination ARGB8888 framebuffer (full window)
 *   strip    — source RGB565 digit strip artwork
 *   strip_w  — width of the strip in pixels
 *   strip_h  — total height of the strip in pixels
 *   digit_h  — height of one digit cell in pixels (float for precision)
 *   x, y     — top-left corner of the drum window on the instrument face
 *   vis_h    — height of the visible drum window in pixels
 *   value    — which digit to center (0.0–9.999, wraps around)
 */
static void blit_drum(uint32_t *fb, const uint16_t *strip,
    int strip_w, int strip_h, float digit_h,
    int x, int y, int vis_h, float value)
{
    /* Convert the floating-point digit value to a pixel offset into the strip */
    int y_offset = (int)(value * digit_h);

    /* Calculate which strip row corresponds to the top of the visible window.
     * The digit_h/2 term ensures we center the digit cell in the window,
     * and vis_h/2 offsets from the window's center to its top edge. */
    int strip_y_start = y_offset - vis_h/2 + (int)(digit_h/2);

    /* Iterate over each visible row in the drum window */
    for (int row = 0; row < vis_h; row++) {
        int sy = strip_y_start + row;

        /* Wrap around: if we've scrolled past the top or bottom of the
         * strip, wrap to the other end.  This simulates the circular
         * nature of the drum — after 9 comes 0 again. */
        while (sy < 0) sy += strip_h;
        while (sy >= strip_h) sy -= strip_h;

        int bg_y = y + row;
        if (bg_y < 0 || bg_y >= DISP_H) continue;

        /* Blit each pixel in this row, converting RGB565 to ARGB8888 inline */
        for (int col = 0; col < strip_w; col++) {
            int bg_x = x + col;
            if (bg_x < 0 || bg_x >= DISP_W) continue;
            uint16_t p = strip[sy*strip_w+col];
            uint8_t r = ((p>>11)&0x1F)<<3; r|=r>>5;
            uint8_t g = ((p>>5)&0x3F)<<2;  g|=g>>6;
            uint8_t b = (p&0x1F)<<3;       b|=b>>5;
            fb[bg_y*WIN_W+bg_x] = 0xFF000000|(r<<16)|(g<<8)|b;
        }
    }
}

/*
 * update_baro_drums — render the barometric pressure Kollsman window
 *
 * The Kollsman window on the AAU-19/A displays the altimeter setting
 * in inches of mercury (e.g., 29.92) using four small counter drums.
 * This function decomposes the baro_inhg value into its four digits
 * and renders each one using the baro_strip (which was built at
 * startup by extracting and repacking glyphs from the 100s strip).
 *
 * GENEVA DRIVE CARRY MECHANISM:
 *
 *   In a real mechanical counter, digits don't snap — when the ones
 *   digit rolls from 9 to 0, it physically pushes the tens digit
 *   forward through a Geneva drive (or similar cam mechanism).
 *   The carry happens smoothly over the last ~10% of the lower
 *   digit's rotation.
 *
 *   We simulate this in software:
 *
 *   1. Decompose baro_inhg * 100 into four digit positions
 *      (d0 = thousands, d1 = hundreds, d2 = tens, d3 = ones).
 *
 *   2. For the ones digit (d3), use the raw fractional value.
 *
 *   3. For each higher digit, compute the "carry fraction":
 *        c3 = (d3 >= 9) ? d3 - 9 : 0
 *      This is zero when d3 is 0-8, and ramps from 0.0 to 1.0
 *      as d3 goes from 9.0 to 10.0 (which wraps to 0.0).
 *
 *   4. Add the carry to the next digit's floor value:
 *        d2 = floor(d2_raw) + c3
 *      So when d3 passes through 9, d2 smoothly advances.
 *
 *   5. Repeat the carry cascade: c2 feeds into d1, c1 into d0.
 *
 *   6. Finally, fmodf(…+10, 10) ensures all values wrap into 0-9.
 *
 *   The result is that all four drums rotate smoothly with realistic
 *   mechanical coupling — no jarring snaps between digits.
 *
 * The four drums are rendered side-by-side, centered horizontally
 * at x=321 (the position of the Kollsman window on the dial face).
 */
static void update_baro_drums(uint32_t *fb, const uint16_t *strip,
    int strip_w, int strip_h, float digit_h, int vis_h, float baro_inhg)
{
    /* Multiply by 100 to work in integer-like units:
     * e.g., 29.92 inHg -> bv = 2992.0, so d3 (ones) = 2, d2 (tens) = 9, etc. */
    float bv = baro_inhg*100;

    /* Extract raw digit positions (fractional) from right (ones) to left (thousands) */
    float d3=fmodf(bv,10), d2r=fmodf(bv/10,10), d1r=fmodf(bv/100,10), d0r=fmodf(bv/1000,10);

    /* Geneva drive carry cascade — each carry is zero until the lower
     * digit enters the 9-to-0 transition zone, then ramps 0.0 to 1.0 */
    float c3=d3>=9?d3-9:0; float d2=floorf(d2r)+c3;
    float c2=(d2>=9&&d2<10)?d2-9:0; float d1=floorf(d1r)+c2;
    float c1=(d1>=9&&d1<10)?d1-9:0; float d0=floorf(d0r)+c1;

    /* Wrap all digit values into the 0-9 range for the drum renderer */
    d0=fmodf(d0+10,10); d1=fmodf(d1+10,10); d2=fmodf(d2+10,10); d3=fmodf(d3+10,10);

    /* Render four drums side-by-side, centered at the Kollsman window position */
    int x0=321-4*strip_w/2;
    blit_drum(fb,strip,strip_w,strip_h,digit_h,x0,BARO_Y,vis_h,d0);           /* thousands */
    blit_drum(fb,strip,strip_w,strip_h,digit_h,x0+strip_w,BARO_Y,vis_h,d1);   /* hundreds  */
    blit_drum(fb,strip,strip_w,strip_h,digit_h,x0+2*strip_w,BARO_Y,vis_h,d2); /* tens      */
    blit_drum(fb,strip,strip_w,strip_h,digit_h,x0+3*strip_w,BARO_Y,vis_h,d3); /* ones      */
}

/*
 * rotate_pointer — rotate the pointer needle and alpha-blend onto the dial
 *
 * INVERSE MAPPING TECHNIQUE:
 *
 *   Rather than rotating each source (pointer) pixel forward to find
 *   where it lands on screen — which would leave gaps between pixels
 *   due to rounding — we use INVERSE MAPPING:
 *
 *   For every destination pixel in the bounding box, we apply the
 *   inverse rotation to find which source pixel it came from.
 *   This guarantees every destination pixel is filled with no gaps.
 *
 *   The math:
 *     Forward rotation (source -> screen):
 *       dx = CX + (sx-PIVOT_X)*cos(a) - (sy-PIVOT_Y)*sin(a)
 *       dy = CY + (sx-PIVOT_X)*sin(a) + (sy-PIVOT_Y)*cos(a)
 *
 *     Inverse rotation (screen -> source):
 *       sx = PIVOT_X + (dx-CX)*cos(a) + (dy-CY)*sin(a)
 *       sy = PIVOT_Y - (dx-CX)*sin(a) + (dy-CY)*cos(a)
 *
 *     (The inverse of a rotation by angle 'a' is rotation by '-a',
 *      and cos(-a)=cos(a), sin(-a)=-sin(a), giving the sign flip.)
 *
 * BOUNDING BOX OPTIMIZATION:
 *
 *   We first forward-transform all four corners of the pointer sprite
 *   to find the axis-aligned bounding box on screen.  Then we only
 *   iterate over pixels within that box, skipping the vast majority
 *   of the 480x480 framebuffer.  This is much faster than scanning
 *   every pixel.
 *
 * ALPHA BLENDING:
 *
 *   The pointer sprite has an alpha channel (from ARGB4444 source).
 *   For each destination pixel, we composite using the standard
 *   "source over" blend formula:
 *
 *     result = src * alpha + dst * (1 - alpha)
 *
 *   Applied per-channel (R, G, B) with integer math:
 *     out_r = (src_r * sa + dst_r * (255 - sa)) / 255
 *
 *   Fast paths:
 *     - sa == 0:   fully transparent, skip pixel entirely
 *     - sa == 255: fully opaque, overwrite without blending
 *
 *   This gives smooth anti-aliased edges on the pointer needle.
 *
 * Parameters:
 *   fb        — destination ARGB8888 framebuffer
 *   ptr       — source ARGB8888 pointer sprite (pre-converted from ARGB4444)
 *   angle_rad — rotation angle in radians (0 = pointing up / 12 o'clock)
 */
static void rotate_pointer(uint32_t *fb, const uint32_t *ptr, float angle_rad)
{
    float cosA=cosf(angle_rad), sinA=sinf(angle_rad);

    /* Forward-transform the four corners of the pointer sprite to find
     * the screen-space axis-aligned bounding box (AABB). */
    int csx[4]={0,PTR_WIDTH-1,0,PTR_WIDTH-1}, csy[4]={0,0,PTR_HEIGHT-1,PTR_HEIGHT-1};
    int nx0=DISP_W,ny0=DISP_H,nx1=0,ny1=0;
    for(int c=0;c<4;c++){
        float fx=(float)(csx[c]-PIVOT_X), fy=(float)(csy[c]-PIVOT_Y);
        int dx=PIVOT_SCREEN_X+(int)(fx*cosA-fy*sinA), dy=PIVOT_SCREEN_Y+(int)(fx*sinA+fy*cosA);
        if(dx<nx0)nx0=dx; if(dx>nx1)nx1=dx;
        if(dy<ny0)ny0=dy; if(dy>ny1)ny1=dy;
    }

    /* Clamp the bounding box to the visible instrument area */
    if(nx0<0)nx0=0; if(ny0<0)ny0=0;
    if(nx1>=DISP_W)nx1=DISP_W-1; if(ny1>=DISP_H)ny1=DISP_H-1;

    /* Iterate over every pixel in the bounding box and inverse-map
     * back to the pointer sprite coordinate space */
    for(int dy=ny0;dy<=ny1;dy++) for(int dx=nx0;dx<=nx1;dx++){
        /* Offset from screen center (rotation pivot) */
        float fx=(float)(dx-PIVOT_SCREEN_X), fy=(float)(dy-PIVOT_SCREEN_Y);

        /* Inverse rotation: screen coordinates -> pointer sprite coordinates.
         * This reverses the forward rotation so we can sample the source pixel. */
        int sx=PIVOT_X+(int)(fx*cosA+fy*sinA+0.5f);
        int sy=PIVOT_Y+(int)(-fx*sinA+fy*cosA+0.5f);

        /* If the inverse-mapped point falls outside the pointer sprite, skip it */
        if(sx<0||sx>=PTR_WIDTH||sy<0||sy>=PTR_HEIGHT) continue;

        /* Read the source pixel and its alpha channel */
        uint32_t sp=ptr[sy*PTR_WIDTH+sx]; uint8_t sa=(sp>>24)&0xFF;

        /* Fast path: fully transparent pixel — nothing to draw */
        if(sa==0) continue;

        /* Fast path: fully opaque pixel — direct overwrite, no blending needed */
        if(sa==255){fb[dy*WIN_W+dx]=sp; continue;}

        /* Partial transparency: perform source-over alpha blending.
         * ia (inverse alpha) = 255 - sa, representing the destination's
         * contribution weight.  Each color channel is blended independently:
         *   out = (src_channel * src_alpha + dst_channel * inv_alpha) / 255 */
        uint32_t dp=fb[dy*WIN_W+dx]; uint8_t ia=255-sa;
        fb[dy*WIN_W+dx]=0xFF000000|
            ((((sp>>16&0xFF)*sa+(dp>>16&0xFF)*ia)/255)<<16)|
            ((((sp>>8&0xFF)*sa+(dp>>8&0xFF)*ia)/255)<<8)|
            (((sp&0xFF)*sa+(dp&0xFF)*ia)/255);
    }
}

/*
 * load_bin — load a raw binary asset file into memory
 *
 * Opens the file at 'path', reads exactly 'expected' bytes, and
 * returns a malloc'd buffer.  Returns NULL on any failure (missing
 * file, wrong size, allocation failure).  The caller is responsible
 * for freeing the returned buffer.
 *
 * This is used to load the pre-rendered instrument artwork:
 * dial face backgrounds, digit strip bitmaps, and pointer sprites.
 * These are raw pixel arrays with no file header — the dimensions
 * are known at compile time from the #defines above.
 */
static void *load_bin(const char *path, size_t expected)
{
    FILE *f=fopen(path,"rb"); if(!f){fprintf(stderr,"Cannot open %s\n",path);return NULL;}
    void *buf=malloc(expected); if(!buf){fclose(f);return NULL;}
    size_t got=fread(buf,1,expected,f); fclose(f);
    if(got!=expected){fprintf(stderr,"%s: expected %zu got %zu\n",path,expected,got);free(buf);return NULL;}
    return buf;
}

/* ================================================================== */
/*  Main                                                              */
/* ================================================================== */
int main(int argc, char *argv[])
{
    /* Default Pi IP address; can be overridden on the command line */
    const char *pi_ip = "192.168.1.129";
    if (argc > 1) pi_ip = argv[1];

    /* Initialize Winsock for UDP communication with the Raspberry Pi.
     * We use a connectionless UDP socket because we're sending
     * periodic instrument state at ~30 Hz — occasional dropped packets
     * are acceptable (the next packet arrives 33 ms later). */
    WSADATA wsa; WSAStartup(MAKEWORD(2,2), &wsa);
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in pi_addr;
    memset(&pi_addr,0,sizeof(pi_addr));
    pi_addr.sin_family = AF_INET;
    pi_addr.sin_port = htons(PI_PORT);
    pi_addr.sin_addr.s_addr = inet_addr(pi_ip);
    printf("=== Altimeter Host → %s:%d ===\n", pi_ip, PI_PORT);

    /* ---- Load assets ---- */
    /*
     * All artwork assets are raw binary pixel dumps exported from the
     * STM32 resource pipeline.  The path "../" points up from the
     * simulator/ subdirectory to the project root where the .bin files live.
     *
     * image.bin:           480x480 RGB565 — the altimeter dial face background
     * 100sWheel.bin:       28x567  RGB565 — the 100s drum digit strip (0-9)
     * pointerargb4444.bin: 63x240  ARGB4444 — the pointer needle with alpha
     * 1000sWheel.bin:      45x583  RGB565 — the 1000s drum strip (includes hatch)
     * 10000sWheel.bin:     45x530  RGB565 — the 10000s drum strip
     */
    uint16_t *bg565=load_bin("../image.bin",DISP_W*DISP_H*2); if(!bg565)return 1;
    uint16_t *strip=load_bin("../100sWheel.bin",STRIP_WIDTH*STRIP_HEIGHT*2); if(!strip)return 1;
    uint16_t *ptr4444=load_bin("../pointerargb4444.bin",PTR_WIDTH*PTR_HEIGHT*2); if(!ptr4444)return 1;
    uint16_t *strip1k_full=load_bin("../1000sWheel.bin",STRIP1K_WIDTH*STRIP1K_FULL_H*2); if(!strip1k_full)return 1;

    /* Skip the crosshatch region at the top of the 1000s strip.
     * The hatch pattern shows through the drum window when the
     * altitude is below 0 on the real instrument — we skip past it
     * by advancing the pointer by STRIP1K_HATCH rows. */
    uint16_t *strip1k = strip1k_full + STRIP1K_WIDTH*STRIP1K_HATCH;
    uint16_t *strip10k=load_bin("../10000sWheel.bin",STRIP1K_WIDTH*STRIP1K_HEIGHT*2); if(!strip10k)return 1;

    /* ---- Build baro strip ---- */
    /*
     * BUILD THE BAROMETRIC PRESSURE DRUM STRIP
     *
     * The baro Kollsman window on the real AAU-19/A uses smaller digit
     * drums than the main altitude counter.  Rather than requiring a
     * separate artwork file, we dynamically construct the baro strip
     * by extracting digit glyphs from the 100s drum strip and repacking
     * them into a tighter layout.
     *
     * The process:
     *
     * 1. FIND GLYPH BOUNDS:  Scan every pixel in every digit cell of
     *    the 100s strip.  For each row, compute the luminance and check
     *    if any pixel is "dark" (lum < 128, meaning ink/digit pixels vs.
     *    white background).  Track the topmost and bottommost rows that
     *    contain dark pixels across all 10 digits.  This gives us the
     *    tight vertical bounding box of the actual digit glyphs.
     *
     * 2. CROP HORIZONTALLY: The digits only occupy columns 2-25 of the
     *    28-pixel-wide strip (the edges are padding/border), so we
     *    extract just that range (baro_strip_w = 24 pixels wide).
     *
     * 3. REPACK:  Create a new strip where each digit cell is exactly
     *    glyph_h + 10 pixels tall (glyph height plus padding), instead
     *    of the original ~56.7 pixels.  This makes the baro digits
     *    compact enough to fit in the small Kollsman window.
     *
     * 4. THRESHOLD TO B&W:  Each pixel is converted to either white
     *    (0xFFFF in RGB565) or black (0x0000) based on the luminance
     *    threshold.  This cleans up any anti-aliasing artifacts and
     *    gives crisp digits at the small baro drum size.
     *
     * The luminance formula used is the standard BT.601 approximation:
     *   Y = 0.299*R + 0.587*G + 0.114*B
     * Implemented as fixed-point:  (R*77 + G*150 + B*29) >> 8
     */
    int orig_cell=(int)(DIGIT_HEIGHT+0.5f), crop_left=2, crop_right=25;
    int baro_strip_w=crop_right-crop_left+1;    /* 24 pixels wide */

    /* Pass 1: scan all digits to find the tightest vertical glyph bounds */
    int glyph_top=orig_cell, glyph_bot=0;
    for(int d=0;d<STRIP_DIGITS;d++){
        int cell_y=(int)(d*DIGIT_HEIGHT);       /* top of this digit's cell */
        for(int row=0;row<orig_cell&&(cell_y+row)<STRIP_HEIGHT;row++){
            int has=0;                          /* flag: does this row have ink? */
            for(int col=0;col<STRIP_WIDTH;col++){
                uint16_t p=strip[(cell_y+row)*STRIP_WIDTH+col];
                /* BT.601 luminance from RGB565 */
                int lum=((((p>>11)&0x1F)<<3)*77+(((p>>5)&0x3F)<<2)*150+((p&0x1F)<<3)*29)>>8;
                if(lum<128){has=1;break;}       /* dark pixel found = glyph ink */
            }
            if(has){if(row<glyph_top)glyph_top=row;if(row>glyph_bot)glyph_bot=row;}
        }
    }

    /* Compute the new baro strip geometry based on discovered glyph bounds */
    int glyph_h=glyph_bot-glyph_top+1, baro_cell=glyph_h+10;  /* 10px padding per cell */
    int baro_strip_h=baro_cell*STRIP_DIGITS;    /* total baro strip height (10 digits) */
    float baro_digit_h=(float)baro_cell;        /* floating-point cell height for blit_drum */

    /* Pass 2: extract glyphs and repack into the compact baro strip */
    uint16_t *baro_strip=calloc(baro_strip_w*baro_strip_h,2);  /* calloc = white background */
    for(int d=0;d<STRIP_DIGITS;d++){
        int src_y=(int)(d*DIGIT_HEIGHT)+glyph_top, dst_y=d*baro_cell+5;  /* 5px top padding */
        for(int row=0;row<glyph_h;row++){
            int sy=src_y+row; if(sy>=STRIP_HEIGHT)break;
            for(int col=0;col<baro_strip_w;col++){
                uint16_t p=strip[sy*STRIP_WIDTH+crop_left+col];
                int lum=((((p>>11)&0x1F)<<3)*77+(((p>>5)&0x3F)<<2)*150+((p&0x1F)<<3)*29)>>8;
                /* Threshold: dark pixels become white (0xFFFF) for visibility on
                 * the dark dial face; light pixels become black (0x0000) background */
                baro_strip[(dst_y+row)*baro_strip_w+col]=(lum<128)?0xFFFF:0x0000;
            }
        }
    }

    /* ---- Convert artwork to ARGB8888 for SDL rendering ---- */
    /*
     * bg_clean: the pristine dial face background, copied fresh into
     *           the framebuffer each frame before drawing drums + pointer.
     * fb:       the working framebuffer (instrument + control panel).
     * ptr8888:  the pointer needle sprite in ARGB8888 with alpha.
     */
    uint32_t *bg_clean=malloc(DISP_W*DISP_H*4);
    uint32_t *fb=calloc(WIN_W*WIN_H,4);
    uint32_t *ptr8888=malloc(PTR_WIDTH*PTR_HEIGHT*4);
    rgb565_to_argb8888(bg565,bg_clean,DISP_W*DISP_H); free(bg565);
    argb4444_to_argb8888(ptr4444,ptr8888,PTR_WIDTH*PTR_HEIGHT); free(ptr4444);

    /* ---- SDL ---- */
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window *win=SDL_CreateWindow("Altimeter Host",
        SDL_WINDOWPOS_CENTERED,SDL_WINDOWPOS_CENTERED,WIN_W,WIN_H,0);
    SDL_Renderer *ren=SDL_CreateRenderer(win,-1,
        SDL_RENDERER_ACCELERATED|SDL_RENDERER_PRESENTVSYNC);
    SDL_RenderSetLogicalSize(ren,WIN_W,WIN_H);
    SDL_Texture *tex=SDL_CreateTexture(ren,SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING,WIN_W,WIN_H);

    /* ---- State ---- */
    /*
     * altitude:  current indicated altitude in feet (0 to 99999)
     * baro_inhg: barometric pressure setting in inches Hg (28.10 to 31.00)
     * alt_dir:   animation direction (+1 = climbing, -1 = descending)
     * auto_alt:  when true, altitude oscillates automatically for demo
     */
    float altitude=8000, baro_inhg=29.92f, alt_dir=1;
    int auto_alt = 1;
    int pressed_btn = -1;       /* currently held mouse button index (-1 = none) */
    Uint32 press_start = 0;     /* timestamp when mouse button was pressed */
    int initial_step_done = 0;  /* flag: has the first discrete step been applied? */
    Uint32 last_tick=SDL_GetTicks(), last_send=0;

    static const char *labels[N_CONTROLS] = { "ALT", "BARO" };

    /* ================================================================
     *  MAIN RENDERING LOOP
     *
     *  Each iteration:
     *    1. Process SDL events (keyboard, mouse)
     *    2. Apply button/key-driven value changes
     *    3. Run auto-altitude animation if enabled
     *    4. Clamp values to valid ranges
     *    5. Send UDP packet to Pi at ~30 Hz
     *    6. Compute drum positions + pointer angle
     *    7. Render: background -> drums -> pointer -> panel
     *    8. Present to screen via SDL
     * ================================================================ */
    int running=1;
    while(running){
        Uint32 now=SDL_GetTicks();
        float dt=(now-last_tick)/1000.0f;
        if(dt>0.1f)dt=0.1f;    /* Cap delta-time to prevent huge jumps after stalls */
        last_tick=now;

        int mx, my;
        SDL_GetMouseState(&mx, &my);
        int hovered_btn = hit_test(mx, my);

        SDL_Event ev;
        while(SDL_PollEvent(&ev)){
            switch (ev.type) {
            case SDL_QUIT: running=0; break;
            case SDL_KEYDOWN:
                switch(ev.key.keysym.sym){
                case SDLK_ESCAPE: running=0; break;
                case SDLK_SPACE: auto_alt=!auto_alt; break;
                case SDLK_r: altitude=8000; baro_inhg=29.92f; auto_alt=1; alt_dir=1; break;
                default: break;
                } break;
            case SDL_MOUSEBUTTONDOWN:
                if(ev.button.button==SDL_BUTTON_LEFT){
                    pressed_btn=hit_test(ev.button.x,ev.button.y);
                    press_start=now; initial_step_done=0;
                } break;
            case SDL_MOUSEBUTTONUP:
                if(ev.button.button==SDL_BUTTON_LEFT) pressed_btn=-1;
                break;
            }
        }

        /* ---- Button actions ---- */
        /*
         * Button behavior uses a two-phase model:
         *
         * Phase 1 (initial click):  Apply a single discrete step
         *   (100 ft for altitude, 0.01 inHg for baro).  This gives
         *   precise single-click control.
         *
         * Phase 2 (hold > 400ms):  Switch to continuous mode where
         *   the value changes proportional to dt (delta time),
         *   giving a smooth ramp.  The continuous rate is 10x the
         *   discrete step for altitude (1000 ft/sec) and 10x for
         *   baro (0.10 inHg/sec).
         *
         * Clicking the ALT buttons also disables auto_alt animation
         * to give the user manual control.
         */
        if (pressed_btn >= 0) {
            int dir = (pressed_btn & 1) ? 1 : -1;  /* even=left=decrement, odd=right=increment */
            int group = pressed_btn / 2;            /* 0=ALT, 1=BARO */
            float *target = NULL;
            float step = 0, cont = 0;

            switch (group) {
            case 0: target=&altitude;  step=100; cont=1000*dt; auto_alt=0; break;
            case 1: target=&baro_inhg; step=0.01f; cont=0.10f*dt; break;
            }

            if (target) {
                if (!initial_step_done) { *target+=dir*step; initial_step_done=1; }
                else if (now-press_start>400) *target+=dir*cont;
            }
        }

        /* ---- Arrow keys for baro (continuous) ---- */
        /* Keyboard arrow keys provide continuous baro adjustment at a
         * gentle rate (0.05 inHg/sec), suitable for fine-tuning. */
        const Uint8 *keys = SDL_GetKeyboardState(NULL);
        if (keys[SDL_SCANCODE_UP])   baro_inhg += 0.05f * dt;
        if (keys[SDL_SCANCODE_DOWN]) baro_inhg -= 0.05f * dt;

        /* ---- Auto altitude animation ---- */
        /* When auto_alt is enabled, altitude sweeps up and down between
         * 8000 and 12000 feet at 200 ft/sec, demonstrating the full
         * drum carry mechanism as digits roll over (e.g., 9999 -> 10000). */
        if (auto_alt) {
            altitude += dt*200*alt_dir;
            if(altitude>=12000){altitude=12000;alt_dir=-1;}
            if(altitude<=8000){altitude=8000;alt_dir=1;}
        }

        /* ---- Clamp ---- */
        /* Enforce valid ranges — the real AAU-19/A can display 0 to 99999 ft,
         * and the baro window supports approximately 28.10 to 31.00 inHg. */
        if(altitude>99999) altitude=99999;
        if(altitude<0) altitude=0;
        if(baro_inhg>31) baro_inhg=31;
        if(baro_inhg<28.1f) baro_inhg=28.1f;

        /* ---- Send UDP ---- */
        /*
         * Throttle UDP transmission to ~30 Hz (every 33 ms).
         * Each packet contains the current altitude and baro setting,
         * allowing the Pi receiver to render its own instrument face
         * independently.  The packet is small (12 bytes) and
         * connectionless, so network overhead is minimal.
         */
        if(now-last_send>=SEND_INTERVAL){
            AltPacket pkt={INSTRUMENT_ALT, altitude, baro_inhg};
            sendto(sock,(char*)&pkt,sizeof(pkt),0,
                   (struct sockaddr*)&pi_addr,sizeof(pi_addr));
            last_send=now;
        }

        /* ---- Compute drum positions ---- */
        /*
         * POINTER ANGLE:
         *   One full revolution (2*pi radians) = 1000 feet.
         *   So altitude / 1000 gives the number of revolutions,
         *   multiplied by 2*pi to get radians.
         *   At 8000 ft, the pointer has made 8 full revolutions.
         *   At 8500 ft, it's at the 6 o'clock position (half revolution).
         *
         *   The +M_PI offset compensates for the pointer sprite's
         *   orientation: its tip points DOWN in source coordinates
         *   (pivot is near the top of the sprite), so at angle 0
         *   the tip would point at "5" on the dial.  Adding pi rotates
         *   it half a turn so altitude 0 / 1000 / 2000 etc. correctly
         *   places the tip at "0" (12 o'clock).
         *
         * 100s DRUM VALUE (drum_val):
         *   We want the 100s digit (0-9) centered in the drum window.
         *   altitude/100 gives the raw hundreds value; fmodf(…,10)
         *   wraps it to 0-9, matching the digital altitude readout.
         *
         * 1000s DRUM VALUE (d1kv):
         *   The integer part of altitude/1000 gives which digit to show.
         *   The Geneva drive logic is applied here: when the lower three
         *   digits (altitude % 1000) are in the range 0-99, AND altitude
         *   is at least 100, the 1000s digit is in a carry transition.
         *   The fractional carry (w1k/100) smoothly advances the 1000s
         *   digit from its previous value to the next, creating the
         *   realistic rolling motion. The -1 + w1k/100 term means the
         *   carry animation starts when we're within 100 feet of the
         *   rollover point.
         *
         * 10000s DRUM VALUE (d10kv):
         *   Identical logic to 1000s, but operating on altitude % 10000.
         *   The carry animation happens when the lower four digits are
         *   in the 0-99 range, smoothly advancing the 10000s drum.
         */
        float ptr_angle=(altitude/1000)*2*(float)M_PI + (float)M_PI;
        float drum_val=fmodf(altitude/100,10);
        float d1kr=fmodf(altitude/1000,10), w1k=fmodf(altitude,1000);
        float d1kv=(w1k<100&&altitude>=100)?fmodf(floorf(d1kr)-1+w1k/100+10,10):floorf(d1kr);
        float d10kr=fmodf(altitude/10000,10), w10k=fmodf(altitude,10000);
        float d10kv=(w10k<100&&altitude>=100)?fmodf(floorf(d10kr)-1+w10k/100+10,10):floorf(d10kr);

        /* ---- Render instrument ---- */
        /* Copy clean background into instrument area only.
         * The control panel below is rendered separately.
         * We copy row-by-row because the framebuffer is WIN_W wide
         * while the background is only DISP_W wide (they are the same
         * in this case, but the row stride uses WIN_W). */
        for (int y=0; y<DISP_H; y++)
            memcpy(&fb[y*WIN_W], &bg_clean[y*DISP_W], DISP_W*4);

        /* Render all three altitude counter drums.
         * They are drawn from right (100s) to left (10000s), matching
         * the physical layout on the real instrument. */
        blit_drum(fb,strip,STRIP_WIDTH,STRIP_HEIGHT,DIGIT_HEIGHT,DRUM_X,DRUM_Y,DRUM_VIS_H,drum_val);
        blit_drum(fb,strip1k,STRIP1K_WIDTH,STRIP1K_HEIGHT,DIGIT1K_HEIGHT,DRUM1K_X,DRUM1K_Y,DRUM1K_VIS_H,d1kv);
        blit_drum(fb,strip10k,STRIP1K_WIDTH,STRIP1K_HEIGHT,DIGIT1K_HEIGHT,DRUM10K_X,DRUM1K_Y,DRUM1K_VIS_H,d10kv);

        /* Render the barometric pressure Kollsman window drums.
         * baro_vis is slightly taller than one digit cell to allow
         * a peek at adjacent digits during transitions. */
        int baro_vis=(int)(baro_digit_h*1.1f);
        update_baro_drums(fb,baro_strip,baro_strip_w,baro_strip_h,baro_digit_h,baro_vis,baro_inhg);

        /* Rotate and alpha-blend the pointer needle on top of everything */
        rotate_pointer(fb,ptr8888,ptr_angle);

        /* ---- Render control panel ---- */
        /* Draw the dark panel background and separator lines */
        fill_rect(fb, 0, DISP_H, WIN_W, PANEL_H, COL_PANEL_BG);
        fill_rect(fb, 0, DISP_H, WIN_W, 1, COL_SEPARATOR);            /* top border */
        fill_rect(fb, GROUP_W, DISP_H+4, 1, PANEL_H-8, COL_SEPARATOR);/* vertical divider */

        /* Draw each control group: label, numeric value, and +/- buttons */
        for (int g=0; g<N_CONTROLS; g++) {
            int gcx = g*GROUP_W + GROUP_W/2;    /* horizontal center of this group */
            draw_str_cx(fb, gcx, LABEL_Y, labels[g], COL_LABEL);

            /* Format the numeric value for display */
            char val[16];
            switch(g) {
            case 0: snprintf(val,sizeof(val),"%05.0f",altitude); break;   /* "08000" format */
            case 1: snprintf(val,sizeof(val),"%.2f",baro_inhg); break;    /* "29.92" format */
            }
            draw_str_cx(fb, gcx, VALUE_Y, val, COL_VALUE);

            /* Left button (decrement) and right button (increment) with
             * hover/press visual feedback based on mouse state */
            draw_button(fb, BTN_L_X(g), BTN_Y, -1,
                        hovered_btn==g*2, pressed_btn==g*2);
            draw_button(fb, BTN_R_X(g), BTN_Y,  1,
                        hovered_btn==g*2+1, pressed_btn==g*2+1);
        }

        /* ---- Present ---- */
        /* Upload the framebuffer to the GPU texture and present to screen.
         * SDL_RENDERER_PRESENTVSYNC ensures we don't render faster than
         * the display refresh rate. */
        SDL_UpdateTexture(tex,NULL,fb,WIN_W*4);
        SDL_RenderCopy(ren,tex,NULL,NULL);
        SDL_RenderPresent(ren);
    }

    /* ---- Cleanup ---- */
    closesocket(sock); WSACleanup();
    free(bg_clean);free(fb);free(ptr8888);free(strip);free(strip1k_full);free(strip10k);free(baro_strip);
    SDL_DestroyTexture(tex);SDL_DestroyRenderer(ren);SDL_DestroyWindow(win);SDL_Quit();
    return 0;
}
