/*
 * ADI (Attitude Director Indicator) Host
 * Procedural rendering — no bitmap assets needed
 * Sends pitch/roll to Raspberry Pi via UDP
 *
 * Usage: adi_host.exe [pi_ip]
 *        Default IP: 192.168.1.129, port 5555
 *
 * Controls:
 *   Up/Down      — pitch
 *   Left/Right   — roll
 *   Mouse buttons — PITCH and ROLL in panel
 *   SPACE        — toggle auto animation
 *   R            — reset (wings level)
 *   ESC          — quit
 *
 * -----------------------------------------------------------------------
 * How the 3D artificial horizon works (render_adi):
 *
 * The core idea is to render an artificial horizon as a true 3D sphere,
 * not a flat "sliding texture" like most software attitude indicators.
 * Every pixel inside the circular instrument face is ray-traced onto the
 * surface of a unit sphere, and that sphere point is then un-rotated by
 * the current roll and pitch to find which part of the "world" the pilot
 * is looking at.  The result is a convincing round ball that darkens
 * near its edges, with pitch lines that properly curve and foreshorten
 * as they approach the limb — exactly like a physical gyro horizon.
 *
 * The rendering is done in a single per-pixel loop (Pass 1) followed by
 * overlay passes for the roll scale, roll pointer, aircraft symbol, and
 * fixed index triangle.
 * -----------------------------------------------------------------------
 */
#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <winsock2.h>

#pragma comment(lib, "ws2_32.lib")

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ====================================================================== */
/*  UDP packet definition                                                  */
/* ====================================================================== */
/*
 * AdiPacket is the binary datagram sent from this host to the Raspberry Pi
 * receiver at ~30 Hz.  The Pi uses instrument_id to demux when multiple
 * instruments (altimeter, VSI, ADI, etc.) share one UDP port.
 *
 * Layout (12 bytes, packed):
 *   [0..3]  uint32 instrument_id   — always 4 for the ADI
 *   [4..7]  float  pitch           — nose-up positive, degrees
 *   [8..11] float  roll            — right-wing-down positive, degrees
 */
typedef struct {
    uint32_t instrument_id;     /* 4 = ADI */
    float    pitch;             /* degrees, positive = nose up */
    float    roll;              /* degrees, positive = right wing down */
} AdiPacket;
#define INSTRUMENT_ADI  4       /* unique ID so the Pi receiver knows this is ADI data */
#define PI_PORT         5555    /* UDP destination port on the Raspberry Pi             */
#define SEND_INTERVAL   33      /* milliseconds between UDP sends (~30 Hz)             */

/* ====================================================================== */
/*  Display geometry                                                       */
/* ====================================================================== */
/*
 * The window is divided into two regions stacked vertically:
 *
 *  +--------------------+
 *  |                    |  DISP_H (480px) — the instrument face
 *  |   ADI sphere       |
 *  |                    |
 *  +--------------------+
 *  |  PITCH  |   ROLL   |  PANEL_H (110px) — interactive controls
 *  +--------------------+
 *
 * Total window: 480 x 590 pixels.
 */
#define DISP_W    480           /* instrument face width  (square) */
#define DISP_H    480           /* instrument face height (square) */
#define PANEL_H   110           /* control panel height beneath the ADI */
#define WIN_W     DISP_W        /* overall window width */
#define WIN_H     (DISP_H + PANEL_H)  /* overall window height */

/* ====================================================================== */
/*  Panel layout (2 control groups: PITCH and ROLL)                        */
/* ====================================================================== */
/*
 * The panel is split into N_CONTROLS equal-width columns.  Each column
 * contains a label, a numeric readout, and left/right arrow buttons
 * for decrementing/incrementing the value.
 *
 *  +-------+-------+
 *  | PITCH |  ROLL |     <- LABEL_Y
 *  |  +5   |  -12  |     <- VALUE_Y
 *  | [<][>]| [<][>]|     <- BTN_Y
 *  +-------+-------+
 */
#define N_CONTROLS  2                       /* number of parameter groups        */
#define GROUP_W     (WIN_W / N_CONTROLS)    /* pixel width of each group column  */
#define BTN_W       32                      /* arrow button width in pixels      */
#define BTN_H       28                      /* arrow button height in pixels     */
#define BTN_Y       (DISP_H + 58)          /* top-of-button Y coordinate        */
#define LABEL_Y     (DISP_H + 14)          /* Y of the group label text         */
#define VALUE_Y     (DISP_H + 38)          /* Y of the numeric value text       */
#define BTN_L_X(g)  ((g) * GROUP_W + 30)              /* left button X for group g  */
#define BTN_R_X(g)  ((g) * GROUP_W + GROUP_W - 30 - BTN_W) /* right button X        */

/* ====================================================================== */
/*  Panel colors (ARGB8888 format — 0xAARRGGBB)                            */
/* ====================================================================== */
#define COL_PANEL_BG    0xFF1A1A1Au   /* dark grey panel background          */
#define COL_SEPARATOR   0xFF333333u   /* thin separator lines between groups */
#define COL_BTN_NORMAL  0xFF3A3A3Au   /* button at rest                      */
#define COL_BTN_HOVER   0xFF505050u   /* button when mouse is hovering       */
#define COL_BTN_PRESS   0xFF686868u   /* button while held down              */
#define COL_ARROW_C     0xFFDDDDDDu   /* arrow glyph inside the button       */
#define COL_LABEL       0xFF888888u   /* muted grey for "PITCH" / "ROLL"     */
#define COL_VALUE       0xFF44DD44u   /* bright green for numeric readouts   */

/* ====================================================================== */
/*  ADI rendering constants                                                */
/* ====================================================================== */
/*
 * ADI_R  — radius of the visible sphere in pixels.  190 px in a 480 px
 *          frame leaves room for the bezel ring and roll scale ticks.
 *
 * ADI_PPD — pixels per degree of pitch.  Currently unused directly in
 *           the sphere render (the sphere maps pitch via trig), but kept
 *           for reference and potential use in overlay code.
 *
 * ADI_CX, ADI_CY — the center of the instrument face in pixel coords.
 */
#define ADI_R       190         /* main circle radius (pixels) */
#define ADI_PPD     7.0f        /* pixels per degree of pitch  */
#define ADI_CX      240         /* instrument center X         */
#define ADI_CY      240         /* instrument center Y         */

/* ADI colors (ARGB8888) */
#define COL_SKY     0xFF1874CDu   /* blue sky on the upper hemisphere  */
#define COL_GND     0xFF8B5A2Bu   /* brown ground on the lower half    */
#define COL_WHITE   0xFFFFFFFFu   /* horizon line, ticks, pointer      */
#define COL_BLACK   0xFF000000u   /* area outside the instrument       */
#define COL_GOLD    0xFFFFD700u   /* aircraft symbol (wings + dot)     */
#define COL_BEZEL   0xFF222222u   /* dark ring between sphere and ticks */

/* ====================================================================== */
/*  5x7 bitmap font                                                        */
/* ====================================================================== */
/*
 * A tiny monospaced bitmap font used to draw labels ("PITCH", "ROLL")
 * and numeric values ("+12", "-5") directly into the framebuffer without
 * needing SDL_ttf or any font file.
 *
 * Each glyph is 5 pixels wide by 7 pixels tall.  The bits in each row
 * byte are packed MSB-left: bit 4 = leftmost column, bit 0 = rightmost.
 * For example, '0' row 0 = 0x0E = 0b01110, drawing columns 1-3 lit.
 *
 * FONT_SCALE=2 doubles each pixel to 10x14 on screen for readability.
 * CHAR_W = (5+1)*2 = 12 pixels per character cell including 1-column gap.
 */
#define FONT_W  5
#define FONT_H  7
#define FONT_SCALE  2
#define CHAR_W  ((FONT_W + 1) * FONT_SCALE)

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
    {'C', {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}},
    {'H', {0x11,0x11,0x11,0x1F,0x11,0x11,0x11}},
    {'I', {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}},
    {'L', {0x10,0x10,0x10,0x10,0x10,0x10,0x1F}},
    {'O', {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}},
    {'P', {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}},
    {'R', {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}},
    {'T', {0x1F,0x04,0x04,0x04,0x04,0x04,0x04}},
    {'+', {0x00,0x04,0x04,0x1F,0x04,0x04,0x00}},
    {'-', {0x00,0x00,0x00,0x1F,0x00,0x00,0x00}},
    {'.', {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C}},
    {' ', {0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
};
#define N_GLYPHS ((int)(sizeof(glyphs)/sizeof(glyphs[0])))

/*
 * draw_char -- render a single character into the framebuffer.
 *
 * Looks up the character in the glyphs[] table, then iterates over each
 * row and column of the 5x7 bitmap.  For each "on" pixel, it writes a
 * FONT_SCALE x FONT_SCALE block of pixels into the framebuffer.
 *
 * The bit test `(0x10 >> col)` checks columns left-to-right: bit 4 is
 * the leftmost column (col=0), bit 0 is the rightmost (col=4).
 */
static void draw_char(uint32_t *fb, int x, int y, char c, uint32_t color)
{
    const uint8_t *bits = NULL;
    /* Linear search for the glyph — only 22 entries, so this is fine */
    for (int i = 0; i < N_GLYPHS; i++)
        if (glyphs[i].ch == c) { bits = glyphs[i].r; break; }
    if (!bits) return;  /* character not in our font — skip it silently */
    for (int row = 0; row < FONT_H; row++)
        for (int col = 0; col < FONT_W; col++)
            if (bits[row] & (0x10 >> col))    /* is this pixel lit? */
                for (int sy = 0; sy < FONT_SCALE; sy++)
                    for (int sx = 0; sx < FONT_SCALE; sx++) {
                        int px = x+col*FONT_SCALE+sx, py = y+row*FONT_SCALE+sy;
                        if ((unsigned)px<WIN_W && (unsigned)py<WIN_H)
                            fb[py*WIN_W+px] = color;
                    }
}

/*
 * draw_str -- render a null-terminated string, advancing CHAR_W pixels
 * per character (left-aligned at position x,y).
 */
static void draw_str(uint32_t *fb, int x, int y, const char *s, uint32_t color)
{ while (*s) { draw_char(fb,x,y,*s++,color); x+=CHAR_W; } }

/*
 * draw_str_cx -- render a string centered horizontally about pixel cx.
 * Computes the total pixel width of the string, then offsets the start
 * to the left by half that width.  The "- FONT_SCALE" accounts for the
 * trailing inter-character gap that shouldn't count toward centering.
 */
static void draw_str_cx(uint32_t *fb, int cx, int y, const char *s, uint32_t color)
{
    int total_w = (int)strlen(s)*CHAR_W - FONT_SCALE;
    draw_str(fb, cx-total_w/2, y, s, color);
}

/* ====================================================================== */
/*  GUI helpers                                                            */
/* ====================================================================== */

/*
 * fill_rect -- fill a solid axis-aligned rectangle into the framebuffer.
 * Clips to WIN_W x WIN_H.  Used for panel background, separator lines,
 * and button backgrounds.
 */
static void fill_rect(uint32_t *fb, int rx, int ry, int rw, int rh, uint32_t c)
{
    for (int y=ry; y<ry+rh; y++) { if((unsigned)y>=WIN_H) continue;
        for (int x=rx; x<rx+rw; x++) if((unsigned)x<WIN_W) fb[y*WIN_W+x]=c; }
}

/*
 * draw_arrow_btn -- draw a small triangular arrow glyph inside a button.
 *
 * 'dir' controls the direction: -1 = left-pointing, +1 = right-pointing.
 * The arrow is constructed row-by-row: for each row, the width is
 * proportional to the distance from the midpoint (widest at center,
 * zero at top and bottom), forming a triangle.
 *
 * If dir < 0 (left arrow), the triangle's base is on the right side.
 * If dir > 0 (right arrow), the triangle's base is on the left side.
 */
static void draw_arrow_btn(uint32_t *fb, int bx, int by, int bw, int bh,
                            int dir, uint32_t color)
{
    int pad=9, th=bh-2*pad, tw=bw-2*pad, half=th/2;
    if(th<2||tw<2) return;
    for (int row=0;row<th;row++) {
        int w=tw*(half-abs(row-half))/half; if(w<1)w=1;
        int y=by+pad+row, x0=(dir<0)?bx+bw-pad-w:bx+pad;
        for(int c2=0;c2<w;c2++) if((unsigned)(x0+c2)<WIN_W&&(unsigned)y<WIN_H)
            fb[y*WIN_W+x0+c2]=color;
    }
}

/*
 * draw_button -- render a complete button (background + arrow glyph).
 * Chooses the background color based on hover/press state, then draws
 * the directional arrow on top.
 */
static void draw_button(uint32_t *fb, int bx, int by, int dir, int hover, int press)
{
    uint32_t bg=press?COL_BTN_PRESS:hover?COL_BTN_HOVER:COL_BTN_NORMAL;
    fill_rect(fb,bx,by,BTN_W,BTN_H,bg);
    draw_arrow_btn(fb,bx,by,BTN_W,BTN_H,dir,COL_ARROW_C);
}

/*
 * hit_test -- determine which button (if any) the mouse cursor is over.
 *
 * Returns an index 0..N_BUTTONS-1 where:
 *   0 = PITCH left  (decrement)      1 = PITCH right (increment)
 *   2 = ROLL  left  (decrement)      3 = ROLL  right (increment)
 *
 * Returns -1 if the cursor is not over any button.
 */
#define N_BUTTONS (N_CONTROLS*2)
static int hit_test(int mx, int my)
{
    for(int i=0;i<N_BUTTONS;i++){
        int g=i/2, is_r=i&1;
        int bx=is_r?BTN_R_X(g):BTN_L_X(g);
        if(mx>=bx&&mx<bx+BTN_W&&my>=BTN_Y&&my<BTN_Y+BTN_H) return i;
    }
    return -1;
}

/* ====================================================================== */
/*  ADI procedural rendering                                               */
/* ====================================================================== */
/*
 * This is the heart of the program.  render_adi() draws a photorealistic
 * 3D spherical artificial horizon entirely with math — no textures, no
 * bitmaps.  The sphere appears to float behind a round window with a
 * bezel ring, and rotates convincingly in pitch and roll.
 *
 * The rendering proceeds in four passes:
 *
 *   Pass 1 — Per-pixel sphere raycasting (sky/ground/horizon/pitch lines)
 *   Pass 2 — Roll scale ticks on the outer bezel
 *   Pass 3 — Roll pointer triangle (indicates current bank angle)
 *   Pass 4 — Fixed aircraft symbol and top index triangle
 */

/*
 * px -- set a single pixel with bounds checking.
 *
 * The stride is WIN_W because this host renders into a window-sized
 * framebuffer (the ADI occupies the top DISP_W x DISP_H portion).
 * The unsigned cast trick: if x or y is negative, it wraps to a huge
 * positive number that fails the < comparison, so one test covers both
 * the lower and upper bound.
 */
static inline void px(uint32_t *fb, int x, int y, uint32_t c)
{
    if ((unsigned)x < DISP_W && (unsigned)y < DISP_H)
        fb[y * WIN_W + x] = c;
}

/*
 * thick_line -- Bresenham-style line rasterizer with configurable width.
 *
 * Walks from (x0,y0) to (x1,y1) in equal steps along the dominant axis.
 * At each step, draws 'thickness' pixels perpendicular to the dominant
 * axis direction.  This is used by potential overlay features; the main
 * sphere rendering does not need lines since everything is per-pixel.
 */
static void thick_line(uint32_t *fb, int x0, int y0, int x1, int y1,
                        int thickness, uint32_t color)
{
    int dx = abs(x1-x0), dy = abs(y1-y0);
    int steps = (dx > dy) ? dx : dy;
    if (steps == 0) { px(fb,x0,y0,color); return; }
    float xinc = (float)(x1-x0)/steps, yinc = (float)(y1-y0)/steps;
    float fx=x0, fy=y0;
    int half = thickness/2;
    for (int i=0; i<=steps; i++) {
        for (int t=-half; t<=half; t++) {
            if (dx > dy) px(fb,(int)(fx+0.5f),(int)(fy+0.5f)+t,color);
            else         px(fb,(int)(fx+0.5f)+t,(int)(fy+0.5f),color);
        }
        fx+=xinc; fy+=yinc;
    }
}

/* ====================================================================== */
/*  render_adi — the main 3D sphere rendering function                     */
/* ====================================================================== */
/*
 * MATHEMATICAL OVERVIEW
 * ---------------------
 * Imagine a physical gyroscopic artificial horizon: a painted ball sits
 * behind a round window.  The top hemisphere is blue (sky), the bottom
 * is brown (ground), and white lines mark the horizon and pitch angles.
 * When the aircraft pitches or rolls, the ball counter-rotates so the
 * painted horizon stays level with the real world.
 *
 * We simulate this digitally.  For each pixel inside the circular window:
 *
 *   1. MAP TO SPHERE — Convert the 2D pixel position to a 3D point on
 *      the surface of a unit sphere (radius = 1).
 *
 *   2. UN-ROTATE — Apply the inverse of the aircraft's roll and pitch
 *      to find what "world direction" this sphere point corresponds to.
 *
 *   3. SHADE & COLOR — Use the world-space Y component to decide sky
 *      vs. ground, draw the horizon line, draw pitch reference lines,
 *      and apply edge-darkening for the 3D curvature illusion.
 *
 * Parameters:
 *   fb     — pointer to the ARGB8888 framebuffer
 *   pitch  — aircraft pitch in degrees (positive = nose up)
 *   roll   — aircraft roll in degrees (positive = right wing down)
 *   stride — framebuffer row stride in pixels (= WIN_W for host window)
 */
static void render_adi(uint32_t *fb, float pitch, float roll, int stride)
{
    /* ------------------------------------------------------------------
     * PRE-COMPUTE ROTATION TRIG
     * ------------------------------------------------------------------
     * We need to "un-rotate" each sphere point by the aircraft's current
     * roll and pitch.  Since the aircraft has rolled and pitched, the
     * ball must appear to rotate the opposite way (inverse rotation).
     *
     * Roll is a rotation about the Z axis (the axis pointing out of the
     * screen toward the pilot).  Pitch is a rotation about the X axis
     * (the axis pointing to the pilot's right).
     *
     * cr, sr = cos/sin of roll angle
     * cp, sp = cos/sin of pitch angle
     */
    float roll_rad = roll * (float)M_PI / 180.0f;
    float pitch_rad = pitch * (float)M_PI / 180.0f;
    float cr = cosf(roll_rad), sr = sinf(roll_rad);
    float cp = cosf(pitch_rad), sp = sinf(pitch_rad);

    /* ------------------------------------------------------------------
     * SKY AND GROUND BASE COLORS (as normalized floats for shading math)
     * ------------------------------------------------------------------
     * These are the same values as COL_SKY and COL_GND but broken into
     * floating-point R,G,B components so we can multiply by the shade
     * factor without integer overflow concerns.
     *
     *   sky  = RGB(0.094, 0.455, 0.804) = #1874CD (steel blue)
     *   ground = RGB(0.545, 0.353, 0.169) = #8B5A2B (saddle brown)
     */
    const float sky_r=0.094f, sky_g=0.455f, sky_b=0.804f;
    const float gnd_r=0.545f, gnd_g=0.353f, gnd_b=0.169f;

    /* ------------------------------------------------------------------
     * PRE-COMPUTE PITCH LINE SIN TARGETS
     * ------------------------------------------------------------------
     * We draw pitch reference lines at +/-5, +/-10, +/-15, +/-20, +/-25
     * degrees.  On the sphere, a pitch line at angle P corresponds to a
     * latitude circle where vy = sin(P).  We pre-compute these 10 sin
     * values so we don't call sinf() for every pixel.
     *
     * Index mapping:
     *   i=0 → +5°    i=1 → -5°
     *   i=2 → +10°   i=3 → -10°
     *   i=4 → +15°   i=5 → -15°
     *   i=6 → +20°   i=7 → -20°
     *   i=8 → +25°   i=9 → -25°
     *
     * The formula: degrees = (i/2 + 1) * 5 * sign
     *   where sign = +1 for even i, -1 for odd i.
     */
    float pitch_sin[10];
    for (int i = 0; i < 10; i++)
        pitch_sin[i] = sinf((float)((i/2+1) * 5 * ((i%2)?-1:1)) * (float)M_PI / 180.0f);

    /* ================================================================== */
    /*  PASS 1: 3D SPHERE RENDERING (per-pixel raycasting)                 */
    /* ================================================================== */
    /*
     * This is the core rendering loop.  For every pixel in the DISP_W x
     * DISP_H instrument area, we determine what color it should be.
     *
     * Conceptually, we are looking straight at a unit sphere centered at
     * the origin.  Our "camera" uses orthographic projection: each pixel
     * casts a ray straight forward (along +Z), and if that ray hits the
     * sphere, we shade the hit point.
     *
     * inv_r = 1/ADI_R, used to normalize pixel offsets to the -1..+1
     * range of the unit sphere.
     */
    float inv_r = 1.0f / (float)ADI_R;
    for (int py = 0; py < DISP_H; py++) {
        /*
         * dy = vertical offset of this pixel from the instrument center,
         * in screen pixels.  Positive dy means the pixel is BELOW center
         * on screen (screen Y increases downward).
         */
        float dy = (float)(py - ADI_CY);
        for (int pxx = 0; pxx < DISP_W; pxx++) {
            /*
             * dx = horizontal offset from instrument center.
             * Positive dx means the pixel is to the RIGHT of center.
             */
            float dx = (float)(pxx - ADI_CX);

            /*
             * dist_sq = squared distance from center.
             * We use this to quickly classify the pixel into three zones:
             *   1. Outside the bezel ring  → black background
             *   2. Inside the bezel ring but outside the sphere → bezel color
             *   3. Inside the sphere → 3D rendered
             *
             * Using squared distance avoids a sqrt() per pixel.
             */
            float dist_sq = dx*dx + dy*dy;

            /* --- ZONE 1: Outside the bezel (beyond radius ADI_R + 18) --- */
            if (dist_sq > (ADI_R+18)*(ADI_R+18)) {
                fb[py*stride+pxx] = COL_BLACK; continue;
            }
            /* --- ZONE 2: The bezel ring itself (between ADI_R and ADI_R+18) --- */
            if (dist_sq > ADI_R*ADI_R) {
                fb[py*stride+pxx] = COL_BEZEL; continue;
            }

            /* ==========================================================
             * ZONE 3: Inside the sphere — this is where the magic happens
             * ========================================================== */

            /* ----------------------------------------------------------
             * STEP 1: MAP PIXEL TO UNIT SPHERE
             * ----------------------------------------------------------
             * We normalize the pixel offset by the sphere radius to get
             * coordinates in the range [-1, +1]:
             *
             *   u = dx / R    (horizontal, -1 at left edge, +1 at right)
             *   v = -dy / R   (vertical, FLIPPED so +1 is UP — matching
             *                  the standard math convention where Y points
             *                  up, not the screen convention where Y
             *                  points down)
             *
             * Since we are inside the circle (u^2 + v^2 < 1), we can
             * compute the Z coordinate of the sphere surface:
             *
             *   z = sqrt(1 - u^2 - v^2)
             *
             * This comes directly from the unit sphere equation:
             *   x^2 + y^2 + z^2 = 1
             *
             * The point (u, v, z) is now a unit vector pointing from the
             * sphere's center to the visible surface point.  z is always
             * positive (we see the front hemisphere).
             *
             * KEY INSIGHT: z is largest at the center of the disc (where
             * u=v=0, so z=1) and approaches 0 at the edges (where
             * u^2+v^2 approaches 1).  This is critical for the edge-
             * darkening shading — see below.
             */
            float u = dx * inv_r;
            float v = -dy * inv_r;       /* flip y to point up */
            float z = sqrtf(1.0f - u*u - v*v);

            /* ----------------------------------------------------------
             * STEP 2a: INVERSE ROLL ROTATION
             * ----------------------------------------------------------
             * The pilot has rolled the aircraft by 'roll' degrees.  On a
             * real gyro horizon, this makes the ball appear to rotate in
             * the opposite direction about the viewing axis (Z).
             *
             * To find what the pilot is "looking at" in world space, we
             * apply the INVERSE roll rotation to the screen-space sphere
             * point.  The inverse of a rotation by angle theta is a
             * rotation by -theta, but equivalently we can transpose the
             * rotation matrix:
             *
             *   Forward roll rotation matrix (about Z):
             *     | cos(r)  -sin(r) |   | u |
             *     | sin(r)   cos(r) | * | v |
             *
             *   Inverse (transpose):
             *     |  cos(r)  sin(r) |   | u |     | u*cr + v*sr  |
             *     | -sin(r)  cos(r) | * | v |  =  | -u*sr + v*cr |
             *
             * After this, (u1, v1) is the sphere point with roll removed.
             * The Z component is unchanged by rotation about Z.
             */
            float u1 =  u*cr + v*sr;
            float v1 = -u*sr + v*cr;

            /* ----------------------------------------------------------
             * STEP 2b: INVERSE PITCH ROTATION
             * ----------------------------------------------------------
             * Now we remove the pitch.  Pitch is a rotation about the
             * X axis (the "right" axis), which mixes the V (up) and Z
             * (forward) components while leaving U (right) unchanged.
             *
             *   Forward pitch rotation matrix (about X):
             *     | cos(p)  -sin(p) |   | v1 |
             *     | sin(p)   cos(p) | * | z  |
             *
             *   Inverse (transpose):
             *     |  cos(p)  sin(p) |   | v1 |     | v1*cp + z*sp |
             *     | -sin(p)  cos(p) | * | z  |  =  | -v1*sp + z*cp|
             *
             * The result is a 3D unit vector (vx, vy, vz) in WORLD space:
             *
             *   vx = u1            — world "right" (unchanged by pitch)
             *   vy = v1*cp + z*sp  — world "up"
             *   vz = -v1*sp + z*cp — world "forward"
             *
             * vy is the critical value: it tells us the elevation angle
             * of this point on the sphere in world coordinates.
             *   vy > 0  → this point is above the horizon → sky
             *   vy < 0  → this point is below the horizon → ground
             *   vy == 0 → exactly on the horizon
             *
             * vy is also the sine of the latitude on the world-sphere,
             * so vy = sin(pitch_angle_of_this_point).  This is what we
             * compare against the pitch_sin[] table to draw pitch lines.
             */
            float vy = v1*cp + z*sp;     /* world "up" component */
            float vx = u1;               /* world "right" component */
            float vz = -v1*sp + z*cp;    /* world "forward" component */

            /* ----------------------------------------------------------
             * STEP 3: EDGE-DARKENING SHADING (the 3D illusion)
             * ----------------------------------------------------------
             * This is what makes the flat disc look like a 3D ball.
             *
             * Recall that z = sqrt(1 - u^2 - v^2):
             *   - At the CENTER of the disc: u=0, v=0 → z=1.0 (maximum)
             *   - At the EDGE of the disc: u^2+v^2=1 → z=0.0 (minimum)
             *
             * z represents the cosine of the angle between the surface
             * normal and the viewing direction.  At the center, the
             * surface faces us directly (normal parallel to view) so z=1.
             * At the edge, the surface curves away (normal perpendicular
             * to view) so z=0.
             *
             * The shading formula:
             *
             *   shade = 0.35 + 0.65 * z
             *
             *   - At center (z=1.0): shade = 0.35 + 0.65 = 1.0 (full brightness)
             *   - At edge   (z=0.0): shade = 0.35 + 0    = 0.35 (35% brightness)
             *
             * This linear falloff from 100% to 35% brightness creates a
             * smooth, convincing darkening at the edges, mimicking the
             * way light falls off on a real sphere (similar to Lambertian
             * shading with a head-on light source).  The 0.35 floor
             * ensures the edges are never completely black, keeping the
             * sky/ground colors visible even at the limb.
             *
             * This single line of code is responsible for the entire
             * "3D ball" illusion that distinguishes this rendering from
             * a simple flat circle with a sliding horizon line.
             */
            float shade = 0.35f + 0.65f * z;

            /* ----------------------------------------------------------
             * STEP 4: SKY OR GROUND COLOR SELECTION
             * ----------------------------------------------------------
             * vy > 0 means this world-space direction is above the
             * equator (horizon), so it gets the sky color.
             * vy <= 0 means it's below, so it gets the ground color.
             *
             * This simple binary choice, combined with the edge shading,
             * creates the hemisphere split that defines the attitude
             * indicator's fundamental display.
             */
            float r, g, b;
            if (vy > 0) { r=sky_r; g=sky_g; b=sky_b; }
            else         { r=gnd_r; g=gnd_g; b=gnd_b; }

            /* ----------------------------------------------------------
             * STEP 5: HORIZON LINE (great circle at the equator)
             * ----------------------------------------------------------
             * The horizon is the set of all sphere points where vy = 0,
             * which forms a great circle around the sphere's equator.
             * In world space, this is the boundary between sky and ground.
             *
             * We draw it as a white band wherever |vy| < 0.012.  This
             * threshold in vy-space corresponds to roughly 0.7 degrees
             * of latitude (arcsin(0.012) ~ 0.69°), which gives a
             * visually crisp but not jaggy line.
             *
             * Because vy is computed on the sphere surface, this line
             * is a true great circle: it curves correctly when the view
             * is pitched, and it always appears as a straight line when
             * viewed at zero pitch (which is geometrically correct —
             * a great circle seen edge-on is a straight line).
             *
             * The shade floor of 0.7 ensures the horizon line remains
             * bright and visible even at the edges of the sphere where
             * the general shading would otherwise dim it heavily.
             */
            if (fabsf(vy) < 0.012f) {
                r=1; g=1; b=1;
                shade = (shade < 0.7f) ? 0.7f : shade;
            }

            /* ----------------------------------------------------------
             * STEP 6: PITCH REFERENCE LINES
             * ----------------------------------------------------------
             * Pitch lines are latitude circles on the sphere at specific
             * pitch angles (every 5 degrees: +/-5, +/-10, ..., +/-25).
             *
             * A latitude circle at angle P is the set of points where
             * vy = sin(P).  We pre-computed these sin values in the
             * pitch_sin[] array.
             *
             * For each of the 10 pitch lines, we check:
             *   (a) Is this pixel close to the line's latitude?
             *       |vy - sin(P)| < thick
             *   (b) Is this pixel within the line's angular extent?
             *       |longitude| < hw_rad
             *
             * LONGITUDE CALCULATION:
             *   lon = |atan2(vx, vz)|
             *
             * This computes the absolute angle around the sphere from
             * the center of the front face.  atan2(vx, vz) gives the
             * azimuthal angle in the horizontal plane:
             *   - lon=0 at the front center (vx=0, vz>0)
             *   - lon=pi/2 at the sides (vx=+/-1, vz=0)
             *   - lon=pi at the back (vx=0, vz<0, not visible)
             *
             * By checking lon < hw_rad, we limit pitch lines to a
             * certain angular width centered on the front of the sphere.
             * 10-degree lines (hw_rad=0.35 radians ~ 20°) are wider
             * than 5-degree lines (hw_rad=0.20 radians ~ 11.5°), just
             * like on a real gyro horizon.
             *
             * LINE THICKNESS:
             * 10-degree lines use thick=0.008 (thicker) and 5-degree
             * lines use thick=0.006 (thinner), providing a visual
             * hierarchy that makes the major pitch lines stand out.
             *
             * The same shade floor trick (0.7) keeps lines visible at
             * the sphere's edges.
             */
            float lon = fabsf(atan2f(vx, vz));
            for (int i = 0; i < 10; i++) {
                float target = pitch_sin[i];
                int deg5 = (i/2 + 1) * 5;
                float hw_rad = (deg5 % 10 == 0) ? 0.35f : 0.20f;  /* half-width in radians */
                float thick  = (deg5 % 10 == 0) ? 0.008f : 0.006f; /* line thickness in vy  */

                /* Does this pixel fall on this pitch line? */
                if (fabsf(vy - target) < thick && lon < hw_rad) {
                    r=1; g=1; b=1;
                    shade = (shade < 0.7f) ? 0.7f : shade;
                }

                /* ----------------------------------------------------------
                 * STEP 7: VERTICAL ENDCAPS ON 10-DEGREE PITCH LINES
                 * ----------------------------------------------------------
                 * On a real attitude indicator, the 10-degree pitch lines
                 * have small vertical "endcaps" (short perpendicular bars)
                 * at their tips, making them easy to distinguish from the
                 * thinner 5-degree lines.
                 *
                 * The endcap is drawn where:
                 *   - dvy < 0.04: within 0.04 vy-units of the line's latitude
                 *     (this is the vertical extent of the endcap)
                 *   - dvy >= thick: NOT on the horizontal line itself
                 *     (so it only draws above/below the main line)
                 *   - lon is near hw_rad: at the very tip of the line
                 *     (between hw_rad-0.03 and hw_rad+0.01)
                 *
                 * The 'above' check ensures the endcap extends toward the
                 * horizon, not away from it:
                 *   - For positive pitch lines (target > 0, above horizon):
                 *     the endcap extends downward (vy < target)
                 *   - For negative pitch lines (target < 0, below horizon):
                 *     the endcap extends upward (vy > target)
                 *
                 * This subtlety makes the endcaps point inward toward the
                 * horizon line, matching the real instrument convention.
                 */
                if (deg5 % 10 == 0) {
                    float dvy = fabsf(vy - target);
                    if (dvy < 0.04f && dvy >= thick &&
                        lon > hw_rad - 0.03f && lon < hw_rad + 0.01f) {
                        int above = (target > 0) ? (vy < target) : (vy > target);
                        if (above) { r=1; g=1; b=1; shade=(shade<0.7f)?0.7f:shade; }
                    }
                }
            }

            /* ----------------------------------------------------------
             * STEP 8: FINAL COLOR ASSEMBLY
             * ----------------------------------------------------------
             * Multiply each color channel by the shade factor, scale to
             * 0-255, and pack into an ARGB8888 pixel.
             *
             * The alpha channel is always 0xFF (fully opaque).
             *
             * Layout in memory: 0xAARRGGBB
             *   A = bits 31-24 = 0xFF
             *   R = bits 23-16
             *   G = bits 15-8
             *   B = bits 7-0
             */
            uint8_t rb = (uint8_t)(r * shade * 255.0f);
            uint8_t gb2 = (uint8_t)(g * shade * 255.0f);
            uint8_t bb = (uint8_t)(b * shade * 255.0f);
            fb[py*stride+pxx] = 0xFF000000u | ((uint32_t)rb<<16) | ((uint32_t)gb2<<8) | bb;
        }
    }

    /* ================================================================== */
    /*  PASS 2: ROLL SCALE TICKS ON THE BEZEL                              */
    /* ================================================================== */
    /*
     * Around the top half of the bezel ring, we draw radial tick marks
     * at standard bank angles: 0, +/-10, +/-20, +/-30, +/-45, +/-60
     * degrees.  These are fixed references — the ball rotates behind
     * them, and the roll pointer (Pass 3) moves along this scale.
     *
     * Each tick is a short radial line segment from just outside the
     * sphere edge (r0 = ADI_R + 2) outward to r1.  The length varies:
     *   - 0° (wings level): longest tick (18 px) — the primary reference
     *   - +/-10°, +/-20°, +/-30°: medium ticks (12 px)
     *   - +/-45°, +/-60°: short ticks (10 px)
     *
     * The angle a_rad is measured from 12 o'clock (straight up), with
     * the formula: a_rad = (-90° + tick_angle) converted to radians.
     * The -90° offset rotates from the standard math convention (0° =
     * right) to the instrument convention (0° = up).
     *
     * Each tick is drawn 2 pixels wide (tx and tx+1) for visibility.
     */
    static const float tick_angles[] = {0,10,20,30,45,60,-10,-20,-30,-45,-60};
    for (int i = 0; i < 11; i++) {
        float a_rad = (-90.0f + tick_angles[i]) * (float)M_PI / 180.0f;
        float ca = cosf(a_rad), sa = sinf(a_rad);
        int r0 = ADI_R + 2;    /* start just outside the sphere */
        int r1 = ADI_R + ((tick_angles[i] == 0) ? 18 :
                 (fabsf(tick_angles[i]) <= 30) ? 12 : 10);
        for (int r = r0; r <= r1; r++) {
            int tx = (int)(ADI_CX + r*ca + 0.5f);
            int ty = (int)(ADI_CY + r*sa + 0.5f);
            if ((unsigned)tx < DISP_W && (unsigned)ty < DISP_H) {
                fb[ty*stride+tx] = COL_WHITE;
                /* 2px wide */
                if ((unsigned)(tx+1) < DISP_W)
                    fb[ty*stride+tx+1] = COL_WHITE;
            }
        }
    }

    /* ================================================================== */
    /*  PASS 3: ROLL POINTER (triangle at current roll angle)              */
    /* ================================================================== */
    /*
     * A small filled white triangle sits on the bezel and points inward
     * toward the sphere.  It rotates with the current roll angle, always
     * indicating the aircraft's bank on the fixed roll scale ticks.
     *
     * The triangle is defined by three vertices:
     *   - tip:  the inward-pointing apex, just outside the sphere edge
     *   - base: two corners, further out on the bezel, spread apart
     *           perpendicular to the radial direction
     *
     * GEOMETRY:
     *   The roll pointer sits at angle (-90 + roll)° from horizontal,
     *   where -90° puts 0° roll at the top (12 o'clock position).
     *
     *   (ca, sa) = unit vector pointing radially outward at this angle
     *   (px_d, py_d) = (-sa, ca) = perpendicular to the radial direction,
     *                  used to spread the base vertices apart.
     *
     *   tip   = center + (ADI_R + 3) * radial    (just outside sphere)
     *   base  = center + (ADI_R + 16) * radial   (further out on bezel)
     *   base_left  = base + 6 * perpendicular
     *   base_right = base - 6 * perpendicular
     *
     * RASTERIZATION (Edge Function Method):
     *   The triangle is filled using the edge function algorithm, a
     *   standard technique in software and GPU rasterization.
     *
     *   For a triangle with vertices V0, V1, V2, the edge function for
     *   edge V0->V1 evaluated at point P is:
     *
     *     e0 = (V1.x - V0.x) * (P.y - V0.y) - (V1.y - V0.y) * (P.x - V0.x)
     *
     *   This is the 2D cross product of vectors (V0->V1) and (V0->P).
     *   Its sign tells which side of the edge the point is on:
     *     e0 > 0 → left side of edge (counterclockwise winding)
     *     e0 < 0 → right side of edge (clockwise winding)
     *     e0 = 0 → exactly on the edge
     *
     *   A point is INSIDE the triangle if and only if all three edge
     *   functions have the same sign (all positive for CCW winding, or
     *   all negative for CW winding).  The condition:
     *     (e0>=0 && e1>=0 && e2>=0) || (e0<=0 && e1<=0 && e2<=0)
     *   handles both winding orders, so we don't need to worry about
     *   which direction the vertices are ordered.
     *
     *   We iterate over the triangle's axis-aligned bounding box and
     *   test each pixel.  This is simple, robust, and fast enough for
     *   a single small triangle.
     */
    {
        float a_rad = (-90.0f + roll) * (float)M_PI / 180.0f;
        float ca = cosf(a_rad), sa = sinf(a_rad);
        /* Perpendicular direction (90° rotated from radial) */
        float px_d = -sa, py_d = ca;

        float tip_r = ADI_R + 3;     /* tip radius — just outside sphere   */
        float base_r = ADI_R + 16;   /* base radius — further out on bezel */
        float tip_x = ADI_CX + tip_r*ca;
        float tip_y = ADI_CY + tip_r*sa;
        float base_x = ADI_CX + base_r*ca;
        float base_y = ADI_CY + base_r*sa;
        float hw = 6.0f;   /* half-width of triangle base in pixels */

        /* Three vertices of the triangle */
        float v0x=tip_x, v0y=tip_y;                  /* apex (tip) */
        float v1x=base_x+hw*px_d, v1y=base_y+hw*py_d;  /* base left  */
        float v2x=base_x-hw*px_d, v2y=base_y-hw*py_d;  /* base right */

        /* Axis-aligned bounding box with 1px margin for safety */
        int mn_x = (int)fminf(fminf(v0x,v1x),v2x) - 1;
        int mx_x = (int)fmaxf(fmaxf(v0x,v1x),v2x) + 1;
        int mn_y = (int)fminf(fminf(v0y,v1y),v2y) - 1;
        int mx_y = (int)fmaxf(fmaxf(v0y,v1y),v2y) + 1;

        /* Rasterize: test every pixel in the bounding box */
        for (int ty=mn_y; ty<=mx_y; ty++) {
            if ((unsigned)ty >= DISP_H) continue;
            for (int tx=mn_x; tx<=mx_x; tx++) {
                if ((unsigned)tx >= DISP_W) continue;
                /* Evaluate edge functions for all three edges */
                float e0 = (v1x-v0x)*((float)ty-v0y) - (v1y-v0y)*((float)tx-v0x);
                float e1 = (v2x-v1x)*((float)ty-v1y) - (v2y-v1y)*((float)tx-v1x);
                float e2 = (v0x-v2x)*((float)ty-v2y) - (v0y-v2y)*((float)tx-v2x);
                /* Inside if all edge functions have the same sign */
                if ((e0>=0&&e1>=0&&e2>=0) || (e0<=0&&e1<=0&&e2<=0))
                    fb[ty*stride+tx] = COL_WHITE;
            }
        }
    }

    /* ================================================================== */
    /*  PASS 4: AIRCRAFT SYMBOL (fixed at center — does not rotate)        */
    /* ================================================================== */
    /*
     * The aircraft symbol represents the pilot's own aircraft and stays
     * fixed at the center of the instrument.  The moving sphere behind
     * it shows the aircraft's attitude relative to the horizon.
     *
     * The symbol is drawn in gold (COL_GOLD) and consists of:
     *   - A center dot (filled circle, radius 4 pixels)
     *   - Horizontal wings (horizontal line from -48 to +48 pixels,
     *     3 pixels thick, with a gap around the center dot)
     *   - Vertical wing tips (12-pixel downward drops at x = +/-48,
     *     2 pixels wide)
     *
     * This matches the classic "W-shape" aircraft reference symbol used
     * on most mechanical and glass-cockpit attitude indicators.
     */

    /* Center dot — filled circle using x^2 + y^2 <= r^2 test */
    for (int dy=-4; dy<=4; dy++)
        for (int dx=-4; dx<=4; dx++)
            if (dx*dx+dy*dy <= 16)      /* radius 4: 4^2 = 16 */
                fb[(ADI_CY+dy)*stride+ADI_CX+dx] = COL_GOLD;

    /* Wings — horizontal bar, 3 pixels thick, with gap around center dot */
    for (int x=-48; x<=48; x++) {
        if (abs(x) < 7) continue;   /* gap around dot */
        for (int t=-1; t<=1; t++)    /* 3px thick (row -1, 0, +1) */
            fb[(ADI_CY+t)*stride+ADI_CX+x] = COL_GOLD;
    }

    /* Wing tips — vertical drops at each end of the wings */
    for (int y=0; y<=12; y++) {
        fb[(ADI_CY+y)*stride+ADI_CX-48] = COL_GOLD;    /* left tip, col 1 */
        fb[(ADI_CY+y)*stride+ADI_CX-49] = COL_GOLD;    /* left tip, col 2 */
        fb[(ADI_CY+y)*stride+ADI_CX+48] = COL_GOLD;    /* right tip, col 1 */
        fb[(ADI_CY+y)*stride+ADI_CX+49] = COL_GOLD;    /* right tip, col 2 */
    }

    /* -----------------------------------------------------------------
     * FIXED TOP INDEX TRIANGLE
     * -----------------------------------------------------------------
     * A small white filled triangle at the 12 o'clock position (top
     * center of the instrument) serves as the fixed roll reference mark.
     * When the roll pointer triangle (Pass 3) aligns with this index
     * mark, the aircraft is wings-level.
     *
     * Drawn row-by-row: row 0 is the bottom (widest), row 7 is the
     * top (single pixel).  For each row, the half-width equals the row
     * number, creating a triangle that points downward (toward the
     * sphere center).
     *
     * Position: starts 8 pixels inside the sphere's top edge, pointing
     * inward.  This places it just below the 0° roll tick mark.
     */
    for (int row=0; row<8; row++) {
        int half_w = row;
        for (int dx=-half_w; dx<=half_w; dx++)
            fb[(ADI_CY-ADI_R+8-row)*stride+ADI_CX+dx] = COL_WHITE;
    }
}

/* ====================================================================== */
/*  Main — application entry point                                         */
/* ====================================================================== */
/*
 * Sets up UDP networking (Winsock), creates an SDL2 window, and runs
 * the main event/render loop at vsync rate (~60 Hz).  Pitch and roll
 * can be controlled by:
 *   - Arrow keys (continuous while held)
 *   - Panel buttons (click or hold)
 *   - Automatic gentle animation (default on, toggle with SPACE)
 *
 * The UDP packet is sent at ~30 Hz (every 33 ms), decoupled from the
 * display refresh rate, to keep network traffic reasonable.
 */
int main(int argc, char *argv[])
{
    /* ---- Parse command-line argument: optional Pi IP address ---- */
    const char *pi_ip = "192.168.1.129";
    if (argc > 1) pi_ip = argv[1];

    /* ---- Initialize Winsock and create a UDP socket ---- */
    WSADATA wsa; WSAStartup(MAKEWORD(2,2), &wsa);
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    /* Set up the Pi's address structure for sendto() */
    struct sockaddr_in pi_addr = {0};
    pi_addr.sin_family = AF_INET;
    pi_addr.sin_port = htons(PI_PORT);
    pi_addr.sin_addr.s_addr = inet_addr(pi_ip);
    printf("=== ADI Host → %s:%d ===\n", pi_ip, PI_PORT);

    /* ---- Allocate the ARGB8888 framebuffer (shared by ADI + panel) ---- */
    uint32_t *fb = calloc(WIN_W*WIN_H, 4);

    /* ---- Initialize SDL2: window, hardware-accelerated renderer, texture ---- */
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window *win = SDL_CreateWindow("ADI Host",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIN_W, WIN_H, 0);
    SDL_Renderer *ren = SDL_CreateRenderer(win, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    SDL_RenderSetLogicalSize(ren, WIN_W, WIN_H);

    /*
     * The streaming texture is updated every frame with the CPU-rendered
     * framebuffer contents.  ARGB8888 matches our pixel format exactly.
     */
    SDL_Texture *tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING, WIN_W, WIN_H);

    /* ---- Application state ---- */
    float pitch = 0, roll_deg = 0;     /* current aircraft attitude      */
    int auto_anim = 1;                 /* 1 = automatic gentle animation */
    int pressed_btn = -1;              /* which panel button is held (-1 = none) */
    Uint32 press_start = 0;            /* timestamp when button was first pressed */
    int initial_step_done = 0;         /* have we done the initial discrete step? */
    Uint32 last_tick = SDL_GetTicks(), last_send = 0;

    static const char *labels[N_CONTROLS] = {"PITCH", "ROLL"};

    /* ================================================================== */
    /*  MAIN LOOP                                                          */
    /* ================================================================== */
    /*
     * Each iteration:
     *   1. Compute delta time (dt) for smooth animation
     *   2. Process SDL events (keyboard, mouse, quit)
     *   3. Apply button/key input or auto-animation to pitch/roll
     *   4. Clamp pitch/roll to safe ranges
     *   5. Send UDP packet to Pi (throttled to ~30 Hz)
     *   6. Render ADI sphere + control panel into framebuffer
     *   7. Upload framebuffer to GPU texture and present
     *
     * The loop runs at vsync rate (~60 Hz) thanks to
     * SDL_RENDERER_PRESENTVSYNC.  dt-based animation ensures smooth
     * motion even if frames are dropped.
     */
    int running = 1;
    while (running) {
        /* Delta time calculation with a 100ms cap to prevent huge jumps
         * after window dragging or debugger pauses */
        Uint32 now = SDL_GetTicks();
        float dt = (now-last_tick)/1000.0f;
        if (dt > 0.1f) dt = 0.1f;
        last_tick = now;

        /* Track mouse position for button hover highlighting */
        int mx, my;
        SDL_GetMouseState(&mx, &my);
        int hovered_btn = hit_test(mx, my);

        /* ---- Event processing ---- */
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            switch (ev.type) {
            case SDL_QUIT: running=0; break;
            case SDL_KEYDOWN:
                switch(ev.key.keysym.sym) {
                case SDLK_ESCAPE: running=0; break;
                case SDLK_SPACE: auto_anim=!auto_anim; break;
                case SDLK_r: pitch=0; roll_deg=0; auto_anim=1; break;
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

        /* ---- Button actions (panel click/hold) ----
         *
         * Button behavior has two phases:
         *   1. On initial click: apply one discrete step (e.g., 1° pitch, 2° roll)
         *   2. After 400ms hold: switch to continuous mode (dt-scaled rate)
         *
         * This gives tactile single-click precision and smooth hold-to-sweep.
         * Any button interaction disables auto-animation.
         */
        if (pressed_btn >= 0) {
            int dir=(pressed_btn&1)?1:-1, group=pressed_btn/2;
            float *target=NULL, step=1, cont=15*dt;
            switch(group) {
            case 0: target=&pitch;    step=1; cont=15*dt; auto_anim=0; break;
            case 1: target=&roll_deg; step=2; cont=30*dt; auto_anim=0; break;
            }
            if (target) {
                if(!initial_step_done){*target+=dir*step; initial_step_done=1;}
                else if(now-press_start>400) *target+=dir*cont;
            }
        }

        /* ---- Arrow key input (continuous, dt-scaled) ----
         *
         * Pitch rate: 15°/sec, Roll rate: 30°/sec.
         * Using SDL_GetKeyboardState() for held-key detection (not events)
         * gives smooth, immediate response.  Any key input disables auto-animation.
         */
        const Uint8 *keys = SDL_GetKeyboardState(NULL);
        if(keys[SDL_SCANCODE_UP])    { pitch+=15*dt; auto_anim=0; }
        if(keys[SDL_SCANCODE_DOWN])  { pitch-=15*dt; auto_anim=0; }
        if(keys[SDL_SCANCODE_LEFT])  { roll_deg-=30*dt; auto_anim=0; }
        if(keys[SDL_SCANCODE_RIGHT]) { roll_deg+=30*dt; auto_anim=0; }

        /* ---- Auto animation: gentle flying ----
         *
         * When enabled, pitch and roll follow a smooth sum-of-sinusoids
         * pattern that simulates gentle maneuvering.  The two frequencies
         * per axis (0.4+1.1 Hz for pitch, 0.25+0.7 Hz for roll) create
         * a non-repeating Lissajous-like motion that looks natural.
         *
         * Pitch: +/-8° slow + +/-3° fast = roughly +/-11° total
         * Roll:  +/-15° slow + +/-8° fast = roughly +/-23° total
         */
        if (auto_anim) {
            float t = now / 1000.0f;
            pitch = 8.0f * sinf(t * 0.4f) + 3.0f * sinf(t * 1.1f);
            roll_deg = 15.0f * sinf(t * 0.25f) + 8.0f * sinf(t * 0.7f);
        }

        /* ---- Clamp to safe display ranges ----
         * Pitch: +/-30° (beyond this the sphere looks odd)
         * Roll:  +/-60° (matches the roll scale tick range)
         */
        if(pitch>30) pitch=30; if(pitch<-30) pitch=-30;
        if(roll_deg>60) roll_deg=60; if(roll_deg<-60) roll_deg=-60;

        /* ---- Send UDP packet to Pi ----
         *
         * Throttled to one packet every SEND_INTERVAL ms (~30 Hz).
         * The packet is a simple 12-byte struct: instrument ID, pitch, roll.
         * sendto() is non-blocking for UDP — it just drops the datagram
         * into the kernel buffer and returns immediately.  If the Pi isn't
         * reachable, the packet is silently lost (which is fine for a
         * real-time instrument display — old data is useless anyway).
         */
        if (now-last_send >= SEND_INTERVAL) {
            AdiPacket pkt = {INSTRUMENT_ADI, pitch, roll_deg};
            sendto(sock,(char*)&pkt,sizeof(pkt),0,
                   (struct sockaddr*)&pi_addr,sizeof(pi_addr));
            last_send = now;
        }

        /* ---- Render ADI (the 3D sphere — see render_adi above) ---- */
        render_adi(fb, pitch, roll_deg, WIN_W);

        /* ---- Render control panel ---- */
        /* Dark background for the panel area below the instrument */
        fill_rect(fb, 0, DISP_H, WIN_W, PANEL_H, COL_PANEL_BG);
        /* Horizontal separator at the top of the panel */
        fill_rect(fb, 0, DISP_H, WIN_W, 1, COL_SEPARATOR);
        /* Vertical separator between PITCH and ROLL groups */
        fill_rect(fb, GROUP_W, DISP_H+4, 1, PANEL_H-8, COL_SEPARATOR);

        /* Draw each control group: label, numeric value, left/right buttons */
        for (int g=0; g<N_CONTROLS; g++) {
            int gcx = g*GROUP_W + GROUP_W/2;   /* horizontal center of this group */
            draw_str_cx(fb, gcx, LABEL_Y, labels[g], COL_LABEL);
            char val[16];
            switch(g) {
            case 0: snprintf(val,sizeof(val),"%+.0f",pitch); break;
            case 1: snprintf(val,sizeof(val),"%+.0f",roll_deg); break;
            }
            draw_str_cx(fb, gcx, VALUE_Y, val, COL_VALUE);
            /* Left arrow button (decrement) */
            draw_button(fb,BTN_L_X(g),BTN_Y,-1,hovered_btn==g*2,pressed_btn==g*2);
            /* Right arrow button (increment) */
            draw_button(fb,BTN_R_X(g),BTN_Y, 1,hovered_btn==g*2+1,pressed_btn==g*2+1);
        }

        /* ---- Upload framebuffer to GPU and present ---- */
        SDL_UpdateTexture(tex,NULL,fb,WIN_W*4);  /* 4 bytes per pixel (ARGB8888) */
        SDL_RenderCopy(ren,tex,NULL,NULL);
        SDL_RenderPresent(ren);  /* blocks until vsync if PRESENTVSYNC is set */
    }

    /* ---- Cleanup ---- */
    closesocket(sock); WSACleanup();
    free(fb);
    SDL_DestroyTexture(tex); SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win); SDL_Quit();
    return 0;
}
