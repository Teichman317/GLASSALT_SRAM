/*
 * GLASSALT VSI (Vertical Speed Indicator) Simulator
 * Desktop SDL2 renderer for 480x480 instrument face
 *
 * The VSI has a non-linear scale — sensitive at low rates,
 * compressed at high rates — just like the real instrument.
 * We use a lookup table measured from the actual dial artwork
 * and interpolate between points.
 *
 * =====================================================================
 *  HOW THE VSI INSTRUMENT WORKS
 * =====================================================================
 *
 *  A Vertical Speed Indicator shows the aircraft's rate of climb or
 *  descent in feet per minute (fpm).  Unlike an altimeter where the
 *  pointer moves linearly, a VSI has a NONLINEAR SCALE:
 *
 *    - At low vertical speeds (0 to +/-500 fpm), the tick marks are
 *      spread far apart, giving the pilot fine resolution where it
 *      matters most (maintaining level flight, gentle climbs/descents).
 *
 *    - At high vertical speeds (+/-1500 to +/-3000 fpm), the tick
 *      marks are compressed together, because precise reading at
 *      extreme rates is less critical.
 *
 *  The zero position is at 9 o'clock on the dial.  Climb (positive
 *  fpm) rotates the pointer clockwise; descent (negative fpm) rotates
 *  it counter-clockwise.
 *
 *  This simulator renders the VSI using the same technique as the
 *  altimeter host: a pre-rendered dial face background with a
 *  rotated pointer needle composited on top using alpha blending.
 *
 * =====================================================================
 *  RENDERING PIPELINE
 * =====================================================================
 *
 *  1. Load the dial face (RGB565) and pointer (ARGB4444) from raw
 *     binary asset files.
 *  2. Upconvert both to ARGB8888 for the SDL display pipeline.
 *  3. Each frame:
 *     a. Copy the clean background into the framebuffer.
 *     b. Convert fpm to pointer angle using the nonlinear scale table.
 *     c. Rotate the pointer sprite and alpha-blend onto the background.
 *     d. Upload to SDL texture and present.
 *
 *  A triangle-wave animation continuously sweeps the pointer from
 *  -1500 to +2000 fpm to demonstrate the full range of the scale.
 */
#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

/* ---- Display dimensions ---- */
/* The VSI dial face is a 480x480 pixel square — matching the round
 * bezel of the real instrument.  This is the same resolution used
 * on the STM32 LTDC display and the Pi HDMI receiver. */
#define IMG_WIDTH   480
#define IMG_HEIGHT  480

/* ---- Pointer source image ---- */
/*
 * The pointer needle sprite is 363 pixels wide by 60 pixels tall,
 * stored as ARGB4444 (16-bit with 4-bit alpha channel).
 *
 * The sprite is oriented horizontally (pointing right = 3 o'clock
 * direction) in its source image.  Rotation is applied at render time
 * to point it in any direction.
 *
 * PIVOT_X (202) is the rotation pivot along the length of the needle.
 * It's shifted 20 pixels left of the geometric center to effectively
 * shorten the visible needle on the hub side — the counterweight end
 * is truncated, and the remaining stub is hidden behind the center cap.
 *
 * PIVOT_Y (30) is the vertical center of the 60-pixel-tall sprite,
 * placing the rotation axis along the needle's centerline.
 */
#define PTR_WIDTH   363
#define PTR_HEIGHT  60
#define PIVOT_X     202   /* shifted left 20px to shorten visible needle */
#define PIVOT_Y     30    /* vertical center */

/* ---- Display center = rotation pivot on screen ---- */
/* The pointer's pivot point on the sprite (PIVOT_X, PIVOT_Y) maps to
 * the center of the dial face (CX, CY) on screen.  All pointer
 * rotation happens around this screen-space point. */
#define CX  240
#define CY  240

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
 * =====================================================================
 *  NON-LINEAR VSI SCALE LOOKUP TABLE
 * =====================================================================
 *
 * This table was measured directly from the real VSI dial artwork by
 * recording the angle of each major tick mark.  The fpm values are
 * the labeled tick marks on the dial; the deg values are the
 * corresponding angular positions measured from the zero (9 o'clock)
 * position, in degrees.
 *
 * Positive degrees = clockwise from 9 o'clock (climb direction).
 * Negative degrees = counter-clockwise from 9 o'clock (descent direction).
 *
 * NONLINEARITY IN DETAIL:
 *
 *   Look at the spacing between consecutive entries:
 *
 *   fpm range       | angle span  | degrees per 500 fpm
 *   ----------------+-------------+---------------------
 *   -2500 to -2000  |   24 deg    |   24 deg/500 fpm  (compressed)
 *   -2000 to -1500  |   37 deg    |   37 deg/500 fpm
 *   -1500 to -1000  |   32.5 deg  |   32.5 deg/500 fpm
 *   -1000 to  -500  |   32.5 deg  |   32.5 deg/500 fpm
 *    -500 to     0  |   25 deg    |   25 deg/500 fpm
 *       0 to   500  |   24 deg    |   24 deg/500 fpm
 *     500 to  1000  |   33 deg    |   33 deg/500 fpm
 *    1000 to  1500  |   33 deg    |   33 deg/500 fpm
 *    1500 to  2000  |   39 deg    |   39 deg/500 fpm
 *    2000 to  2500  |   24 deg    |   24 deg/500 fpm  (compressed)
 *    2500 to  3000  |   21 deg    |   21 deg/500 fpm  (most compressed)
 *
 *   The middle of the scale (-1000 to +1000 fpm) gets the most
 *   angular real estate (~33 deg per 500 fpm), while the extremes
 *   (+/-2000 to +/-3000 fpm) are compressed (~21-24 deg per 500 fpm).
 *
 *   The slight asymmetry between climb and descent is inherent in the
 *   real instrument's dial artwork — the climb side extends to 3000 fpm
 *   while descent only goes to 2500 fpm.
 *
 *   Note: the table is slightly asymmetric (e.g., +500 = 24 deg but
 *   -500 = -25 deg).  This is not a bug — it reflects the actual
 *   measured positions of the tick marks on the real dial face.
 */
typedef struct { float fpm; float deg; } ScalePoint;

static const ScalePoint vsi_scale[] = {
    { -2500, -151.0f },     /* Maximum descent — nearly 5 o'clock position */
    { -2000, -127.0f },
    { -1500,  -90.0f },     /* 6 o'clock position (straight down) */
    { -1000,  -57.5f },
    {  -500,  -25.0f },
    {     0,    0.0f },     /* Zero rate = 9 o'clock position (pointing left) */
    {   500,   24.0f },
    {  1000,   57.0f },
    {  1500,   90.0f },     /* 12 o'clock position (straight up) */
    {  2000,  129.0f },
    {  2500,  153.0f },
    {  3000,  174.0f },     /* Maximum climb — nearly 3 o'clock position */
};
#define SCALE_COUNT (sizeof(vsi_scale) / sizeof(vsi_scale[0]))

/*
 * vsi_fpm_to_deg — convert vertical speed in fpm to pointer angle in degrees
 *
 * PIECEWISE LINEAR INTERPOLATION:
 *
 *   The lookup table defines a series of (fpm, deg) points.  Between
 *   any two adjacent points, we assume the relationship is linear.
 *   This is a standard technique for approximating a nonlinear curve
 *   with a series of connected line segments.
 *
 *   For an input fpm value that falls between table entries i and i+1:
 *
 *     t = (fpm - table[i].fpm) / (table[i+1].fpm - table[i].fpm)
 *
 *   't' is the interpolation parameter, ranging from 0.0 (at entry i)
 *   to 1.0 (at entry i+1).  The output angle is then:
 *
 *     angle = table[i].deg + t * (table[i+1].deg - table[i].deg)
 *
 *   This is the textbook "lerp" (linear interpolation) formula.
 *
 * CLAMPING:
 *
 *   If fpm is below the minimum table entry (-2500) or above the
 *   maximum (+3000), we clamp to the corresponding endpoint angle.
 *   The pointer just pegs against the stop, like the real instrument.
 *
 * ACCURACY:
 *
 *   With 12 data points spanning the full range, the maximum error
 *   between the piecewise linear approximation and the real dial
 *   markings is well under 1 degree — imperceptible at display
 *   resolution.  More points could be added for finer fidelity,
 *   but the current table matches the major tick marks exactly.
 */
static float vsi_fpm_to_deg(float fpm)
{
    /* Clamp to scale range — pointer can't go past the end stops */
    if (fpm <= vsi_scale[0].fpm) return vsi_scale[0].deg;
    if (fpm >= vsi_scale[SCALE_COUNT-1].fpm) return vsi_scale[SCALE_COUNT-1].deg;

    /* Find the two surrounding points and interpolate.
     * We do a simple linear scan since there are only 12 entries —
     * binary search would be overkill here. */
    for (int i = 0; i < (int)SCALE_COUNT - 1; i++) {
        if (fpm >= vsi_scale[i].fpm && fpm <= vsi_scale[i+1].fpm) {
            /* Compute interpolation parameter t in [0.0, 1.0] */
            float t = (fpm - vsi_scale[i].fpm)
                    / (vsi_scale[i+1].fpm - vsi_scale[i].fpm);
            /* Linearly interpolate the angle between the two endpoints */
            return vsi_scale[i].deg + t * (vsi_scale[i+1].deg - vsi_scale[i].deg);
        }
    }
    return 0.0f;    /* Fallback (should never reach here) */
}

/*
 * rgb565_to_argb8888 — convert RGB565 pixels to ARGB8888
 *
 * RGB565 is a compact 16-bit pixel format commonly used in embedded
 * display controllers (like the STM32 LTDC).  It packs color into
 * 16 bits as: [RRRRR GGGGGG BBBBB] — 5 bits red, 6 bits green,
 * 5 bits blue.  Green gets the extra bit because the human eye is
 * most sensitive to green.
 *
 * To expand to 8 bits per channel without a dark bias, we use the
 * "bit replication" trick:
 *
 *   For a 5-bit value like red:
 *     Step 1: Shift left by 3 to put it in the upper 5 bits of a byte
 *             e.g., 10110 -> 10110000 (0xB0)
 *     Step 2: OR with itself shifted right by 5 to fill the lower bits
 *             e.g., 10110000 | 00010 -> 10110110 (0xB6)
 *
 *   This maps 0x00 -> 0x00 and 0x1F -> 0xFF with even spacing,
 *   rather than leaving a gap at the top (0x1F << 3 = 0xF8, not 0xFF).
 *
 * Alpha is always set to 0xFF (fully opaque) since RGB565 has no
 * alpha channel — the dial face background is always opaque.
 */
/* Convert RGB565 to ARGB8888 */
static void rgb565_to_argb8888(const uint16_t *src, uint32_t *dst, int count)
{
    for (int i = 0; i < count; i++) {
        uint16_t p = src[i];
        uint8_t r = ((p >> 11) & 0x1F) << 3; r |= r >> 5;  /* 5-bit red   -> 8-bit */
        uint8_t g = ((p >>  5) & 0x3F) << 2; g |= g >> 6;  /* 6-bit green -> 8-bit */
        uint8_t b = ( p        & 0x1F) << 3; b |= b >> 5;  /* 5-bit blue  -> 8-bit */
        dst[i] = 0xFF000000 | (r << 16) | (g << 8) | b;
    }
}

/*
 * argb4444_to_argb8888 — convert ARGB4444 pixels to ARGB8888
 *
 * ARGB4444 is a 16-bit format with 4 bits per channel:
 *   [AAAA RRRR GGGG BBBB]
 *
 * Each 4-bit nibble is expanded to 8 bits by multiplying by 17 (0x11).
 * This works because 0xF * 17 = 255 = 0xFF and 0x0 * 17 = 0.
 * Equivalently, it duplicates the nibble: 0xA -> 0xAA.
 *
 * The 4-bit alpha channel is particularly important for the pointer
 * sprite: it provides 16 levels of transparency (0=fully transparent,
 * 15=fully opaque) which is enough for clean anti-aliased edges
 * around the needle shape.  When expanded to 8-bit, these become
 * 0, 17, 34, 51, ... 238, 255 — sufficient for smooth blending.
 */
/* Convert ARGB4444 to ARGB8888 */
static void argb4444_to_argb8888(const uint16_t *src, uint32_t *dst, int count)
{
    for (int i = 0; i < count; i++) {
        uint16_t p = src[i];
        uint8_t a = ((p >> 12) & 0xF) * 17;    /* 4-bit alpha -> 8-bit */
        uint8_t r = ((p >>  8) & 0xF) * 17;    /* 4-bit red   -> 8-bit */
        uint8_t g = ((p >>  4) & 0xF) * 17;    /* 4-bit green -> 8-bit */
        uint8_t b = ( p        & 0xF) * 17;    /* 4-bit blue  -> 8-bit */
        dst[i] = (a << 24) | (r << 16) | (g << 8) | b;
    }
}

/*
 * rotate_pointer — rotate the pointer needle and alpha-blend onto the dial
 *
 * This function composites the pointer sprite onto the dial face
 * framebuffer at an arbitrary rotation angle.  It uses the INVERSE
 * MAPPING technique with per-pixel ALPHA BLENDING.
 *
 * =====================================================================
 *  INVERSE MAPPING (why we go "backwards")
 * =====================================================================
 *
 *  The naive approach to rotating a sprite would be FORWARD MAPPING:
 *  for each source pixel, compute where it lands on the destination.
 *  But this leaves gaps — when a source pixel at (sx, sy) maps to
 *  destination (3.7, 5.2), we'd round to (4, 5), and the neighboring
 *  destination pixel (3, 5) might never get written.
 *
 *  INVERSE MAPPING solves this perfectly: we iterate over every
 *  DESTINATION pixel and ask "which source pixel maps here?"  Since
 *  we visit every destination pixel exactly once, there are no gaps.
 *
 *  The rotation math:
 *
 *    Forward (source -> destination):
 *      dx = CX + (sx - PIVOT_X) * cos(a) - (sy - PIVOT_Y) * sin(a)
 *      dy = CY + (sx - PIVOT_X) * sin(a) + (sy - PIVOT_Y) * cos(a)
 *
 *    Inverse (destination -> source):
 *      sx = PIVOT_X + (dx - CX) * cos(a) + (dy - CY) * sin(a)
 *      sy = PIVOT_Y - (dx - CX) * sin(a) + (dy - CY) * cos(a)
 *
 *    The inverse rotation is simply rotation by -a.  Since
 *    cos(-a) = cos(a) and sin(-a) = -sin(a), we get the sign flip
 *    on the sin terms in the inverse formula.
 *
 * =====================================================================
 *  BOUNDING BOX OPTIMIZATION
 * =====================================================================
 *
 *  We don't want to inverse-map all 480*480 = 230,400 pixels when
 *  the pointer only covers a fraction of the screen.  Instead:
 *
 *  1. Forward-transform the four corners of the pointer sprite to
 *     find the axis-aligned bounding box (AABB) on screen.
 *
 *  2. Clamp the AABB to the display bounds.
 *
 *  3. Only iterate over pixels within that AABB.
 *
 *  For the 363x60 pointer, the AABB is at most ~367 pixels on a
 *  side (when rotated 45 degrees), so we process roughly 135K pixels
 *  instead of 230K — a meaningful speedup.
 *
 * =====================================================================
 *  ALPHA BLENDING (source-over compositing)
 * =====================================================================
 *
 *  The pointer sprite has per-pixel alpha from its ARGB4444 encoding.
 *  For each pixel, we perform "source over" compositing:
 *
 *    output_color = src_color * src_alpha + dst_color * (1 - src_alpha)
 *
 *  In integer math (0-255 scale):
 *    out_r = (src_r * sa + dst_r * (255 - sa)) / 255
 *    out_g = (src_g * sa + dst_g * (255 - sa)) / 255
 *    out_b = (src_b * sa + dst_b * (255 - sa)) / 255
 *
 *  Fast paths for common cases:
 *    - sa == 0:   Fully transparent — skip entirely (background shows through)
 *    - sa == 255: Fully opaque — overwrite directly (no blending math needed)
 *
 *  These fast paths avoid the expensive multiply-and-divide for the
 *  majority of pixels (most are either fully transparent background
 *  or fully opaque needle body).  Only the anti-aliased EDGE pixels
 *  require the full blending computation.
 *
 *  The output alpha is forced to 0xFF (fully opaque) since the
 *  framebuffer is always opaque — we're compositing onto a solid
 *  background, not building a layered transparency stack.
 */
/* Rotate pointer and alpha-blend onto framebuffer */
static void rotate_pointer(uint32_t *fb, const uint32_t *ptr, float angle_rad)
{
    float cosA = cosf(angle_rad);
    float sinA = sinf(angle_rad);

    /* Step 1: Compute the screen-space bounding box of the rotated pointer.
     * Forward-transform all four corners of the source sprite. */
    int corners_sx[4] = {0, PTR_WIDTH-1, 0,          PTR_WIDTH-1};
    int corners_sy[4] = {0, 0,           PTR_HEIGHT-1, PTR_HEIGHT-1};
    int nx0 = IMG_WIDTH, ny0 = IMG_HEIGHT, nx1 = 0, ny1 = 0;
    for (int c = 0; c < 4; c++) {
        float fx = (float)(corners_sx[c] - PIVOT_X);
        float fy = (float)(corners_sy[c] - PIVOT_Y);
        int dx = CX + (int)(fx * cosA - fy * sinA);
        int dy = CY + (int)(fx * sinA + fy * cosA);
        if (dx < nx0) nx0 = dx;  if (dx > nx1) nx1 = dx;
        if (dy < ny0) ny0 = dy;  if (dy > ny1) ny1 = dy;
    }

    /* Step 2: Clamp bounding box to display boundaries */
    if (nx0 < 0) nx0 = 0;
    if (ny0 < 0) ny0 = 0;
    if (nx1 >= IMG_WIDTH)  nx1 = IMG_WIDTH - 1;
    if (ny1 >= IMG_HEIGHT) ny1 = IMG_HEIGHT - 1;

    /* Step 3: Iterate over every pixel in the bounding box */
    /* Inverse-map each destination pixel back to source */
    for (int dy = ny0; dy <= ny1; dy++) {
        for (int dx = nx0; dx <= nx1; dx++) {
            /* Offset from screen center (the rotation pivot point) */
            float fx = (float)(dx - CX);
            float fy = (float)(dy - CY);

            /* Apply inverse rotation to find the corresponding source pixel.
             * The +0.5f provides rounding to nearest pixel (instead of
             * truncation toward zero) for better visual quality. */
            int sx = PIVOT_X + (int)( fx * cosA + fy * sinA + 0.5f);
            int sy = PIVOT_Y + (int)(-fx * sinA + fy * cosA + 0.5f);

            /* If the inverse-mapped coordinate falls outside the pointer
             * sprite bounds, this destination pixel doesn't correspond to
             * any part of the needle — skip it. */
            if (sx < 0 || sx >= PTR_WIDTH || sy < 0 || sy >= PTR_HEIGHT)
                continue;

            /* Read the source pixel from the pointer sprite */
            uint32_t sp = ptr[sy * PTR_WIDTH + sx];
            uint8_t sa = (sp >> 24) & 0xFF;     /* source alpha (0=transparent, 255=opaque) */

            /* Fast path: fully transparent — nothing to draw here */
            if (sa == 0) continue;

            /* Fast path: fully opaque — overwrite destination directly */
            if (sa == 255) {
                fb[dy * IMG_WIDTH + dx] = sp;
            } else {
                /* Partial transparency: perform per-channel alpha blending.
                 * This only applies to the anti-aliased edge pixels of the
                 * pointer needle — typically a thin border, so this expensive
                 * path is hit infrequently. */
                uint32_t dp = fb[dy * IMG_WIDTH + dx];  /* read existing destination */
                uint8_t sr = (sp >> 16) & 0xFF, sg = (sp >> 8) & 0xFF, sb = sp & 0xFF;
                uint8_t dr = (dp >> 16) & 0xFF, dg = (dp >> 8) & 0xFF, db = dp & 0xFF;
                uint8_t ia = 255 - sa;  /* inverse alpha = destination weight */
                uint8_t or_ = (sr * sa + dr * ia) / 255;   /* blended red */
                uint8_t og  = (sg * sa + dg * ia) / 255;   /* blended green */
                uint8_t ob  = (sb * sa + db * ia) / 255;   /* blended blue */
                fb[dy * IMG_WIDTH + dx] = 0xFF000000 | (or_ << 16) | (og << 8) | ob;
            }
        }
    }
}

/*
 * load_bin — load a raw binary asset file of exact expected size
 *
 * The instrument artwork files are headerless raw pixel arrays —
 * just width*height*bytes_per_pixel bytes of pixel data, no metadata.
 * The caller knows the exact expected size from compile-time constants.
 *
 * Returns a malloc'd buffer on success, NULL on failure.
 * Failure cases: file not found, allocation failure, or file size
 * mismatch (which likely means the asset was regenerated at a
 * different resolution).
 */
/* Load a raw binary file */
static void *load_bin(const char *path, size_t expected)
{
    FILE *f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "Cannot open %s\n", path); return NULL; }
    void *buf = malloc(expected);
    if (!buf) { fclose(f); return NULL; }
    size_t got = fread(buf, 1, expected, f);
    fclose(f);
    if (got != expected) {
        fprintf(stderr, "%s: expected %zu bytes, got %zu\n", path, expected, got);
        free(buf); return NULL;
    }
    return buf;
}

/* ================================================================== */
/*  Main — application entry point                                    */
/* ================================================================== */
/*
 * The main function orchestrates:
 *   1. Asset loading (dial face background + pointer needle)
 *   2. Pixel format conversion (RGB565/ARGB4444 -> ARGB8888)
 *   3. SDL window/renderer/texture setup
 *   4. The main rendering loop with triangle-wave animation
 *   5. Cleanup on exit
 */
int main(int argc, char *argv[])
{
    (void)argc; (void)argv;     /* Suppress unused parameter warnings */

    /* ---- Load artwork assets ---- */
    /*
     * The artwork lives in "../Vertical speed/" relative to the
     * simulator/ directory.  These are raw binary pixel dumps with
     * no file headers — dimensions are known at compile time.
     *
     * vsi_bg.bin:      480x480 RGB565  — the dial face with tick marks and labels
     * vsi_pointer.bin: 363x60  ARGB4444 — the needle with alpha for blending
     */
    /* Load assets from "Vertical speed" subdirectory */
    uint16_t *bg565 = load_bin("../Vertical speed/vsi_bg.bin",
                               IMG_WIDTH * IMG_HEIGHT * 2);
    if (!bg565) return 1;

    uint16_t *ptr4444 = load_bin("../Vertical speed/vsi_pointer.bin",
                                 PTR_WIDTH * PTR_HEIGHT * 2);
    if (!ptr4444) return 1;

    /* ---- Convert pixel formats from embedded (16-bit) to desktop (32-bit) ---- */
    /*
     * bg_clean: the pristine dial face in ARGB8888.  This is never modified —
     *           it's memcpy'd into the framebuffer each frame as a fresh canvas.
     *
     * fb:       the working framebuffer.  Each frame, it receives a copy of
     *           bg_clean, then the pointer is composited on top.
     *
     * ptr8888:  the pointer sprite in ARGB8888 with alpha channel preserved.
     *           Used by rotate_pointer() for the inverse-mapping blit.
     *
     * The original 16-bit buffers are freed immediately after conversion
     * since they're no longer needed — only the 32-bit versions are used
     * during rendering.
     */
    /* Convert to ARGB8888 */
    uint32_t *bg_clean = malloc(IMG_WIDTH * IMG_HEIGHT * 4);
    uint32_t *fb       = malloc(IMG_WIDTH * IMG_HEIGHT * 4);
    if (!bg_clean || !fb) return 1;
    rgb565_to_argb8888(bg565, bg_clean, IMG_WIDTH * IMG_HEIGHT);
    free(bg565);    /* 16-bit background no longer needed */

    uint32_t *ptr8888 = malloc(PTR_WIDTH * PTR_HEIGHT * 4);
    if (!ptr8888) return 1;
    argb4444_to_argb8888(ptr4444, ptr8888, PTR_WIDTH * PTR_HEIGHT);
    free(ptr4444);  /* 16-bit pointer no longer needed */

    /* ---- Initialize SDL2 ---- */
    /*
     * SDL provides the windowing system, GPU-accelerated texture upload,
     * and vsync'd presentation.  We create a streaming texture that we
     * update every frame with the software-rendered framebuffer — this
     * approach matches the STM32 LTDC workflow where the CPU renders
     * into a framebuffer that the display controller scans out.
     */
    /* Init SDL */
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window *win = SDL_CreateWindow(
        "GLASSALT VSI — Vertical Speed Indicator",
        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
        IMG_WIDTH, IMG_HEIGHT, SDL_WINDOW_SHOWN);
    if (!win) return 1;

    SDL_Renderer *ren = SDL_CreateRenderer(win, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!ren) return 1;
    SDL_RenderSetLogicalSize(ren, IMG_WIDTH, IMG_HEIGHT);  /* maintain aspect ratio */

    /* Create a streaming ARGB8888 texture — "streaming" means we will
     * update it frequently (every frame) via SDL_UpdateTexture. */
    SDL_Texture *tex = SDL_CreateTexture(ren,
        SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
        IMG_WIDTH, IMG_HEIGHT);
    if (!tex) return 1;

    /* ---- Animation state ---- */
    /*
     * TRIANGLE WAVE ANIMATION
     *
     * The VSI pointer continuously sweeps back and forth to demonstrate
     * the full nonlinear scale.  A "triangle wave" is used: the value
     * increases linearly at a constant rate, then when it hits the upper
     * bound it reverses and decreases at the same rate, forming a
     * zig-zag pattern over time.
     *
     * vsi_fpm:  current simulated vertical speed in feet per minute
     * vsi_dir:  current sweep direction (+1.0 = increasing, -1.0 = decreasing)
     *
     * The sweep range is -1500 to +2000 fpm at 300 fpm/sec.
     * A full cycle takes (1500+2000)/300 * 2 = 23.3 seconds.
     * This range was chosen to exercise the most visually interesting
     * part of the nonlinear scale without hitting the extreme compression
     * at +/-2500 fpm.
     */
    /* State: vertical speed in fpm, sweeps -1500 to +2000 */
    float vsi_fpm = 0.0f;
    float vsi_dir = 1.0f;
    Uint32 last_tick = SDL_GetTicks();

    /* ================================================================
     *  MAIN RENDERING LOOP
     *
     *  Each frame:
     *    1. Poll SDL events (quit, keyboard)
     *    2. Compute delta time for frame-rate-independent animation
     *    3. Update the triangle-wave animation
     *    4. Convert fpm to pointer angle via the nonlinear scale table
     *    5. Render: copy clean background, composite rotated pointer
     *    6. Upload framebuffer to GPU texture and present
     *
     *  The loop runs at the display's refresh rate (typically 60 Hz)
     *  due to SDL_RENDERER_PRESENTVSYNC.  Delta-time animation ensures
     *  the sweep speed is consistent regardless of frame rate.
     * ================================================================ */
    int running = 1;
    while (running) {
        /* ---- Event processing ---- */
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT) running = 0;
            if (ev.type == SDL_KEYDOWN) {
                if (ev.key.keysym.sym == SDLK_ESCAPE) running = 0;
            }
        }

        /* ---- Delta time calculation ---- */
        /* Compute the elapsed time since the last frame in seconds.
         * This ensures the animation speed is independent of frame rate —
         * it will sweep at the same fpm/sec whether running at 30 or 120 Hz. */
        Uint32 now = SDL_GetTicks();
        float dt = (now - last_tick) / 1000.0f;
        last_tick = now;

        /* ---- Triangle wave animation ---- */
        /* Advance the simulated vertical speed by 300 fpm per second
         * in the current direction.  Reverse direction when we hit
         * either boundary, clamping to the exact limit to prevent
         * overshoot.  The asymmetric range (-1500 to +2000) means
         * the pointer spends more time on the climb side of the dial. */
        /* Animate: triangle wave -1500 to +2000 fpm at 300 fpm/sec */
        vsi_fpm += dt * 300.0f * vsi_dir;
        if (vsi_fpm >=  2000.0f) { vsi_fpm =  2000.0f; vsi_dir = -1.0f; }
        if (vsi_fpm <= -1500.0f) { vsi_fpm = -1500.0f; vsi_dir =  1.0f; }

        /* ---- FPM to pointer angle conversion ---- */
        /* Look up the pointer angle from the nonlinear scale table.
         * vsi_fpm_to_deg() performs piecewise linear interpolation
         * through the measured tick mark positions, returning degrees
         * from the 9 o'clock zero position.  We then convert to
         * radians for the rotation math in rotate_pointer(). */
        /* Convert vertical speed to pointer angle */
        float angle_deg = vsi_fpm_to_deg(vsi_fpm);
        float angle_rad = angle_deg * (float)M_PI / 180.0f;

        /* ---- Render frame ---- */
        /* Start with a fresh copy of the dial face background.
         * This effectively "erases" the previous frame's pointer
         * position.  The memcpy is fast — just 480*480*4 = 921,600
         * bytes, well within L2 cache on modern CPUs. */
        /* Render: clean background + rotated pointer */
        memcpy(fb, bg_clean, IMG_WIDTH * IMG_HEIGHT * 4);

        /* Composite the rotated pointer needle on top of the background.
         * The pointer's alpha channel provides smooth anti-aliased edges. */
        rotate_pointer(fb, ptr8888, angle_rad);

        /* ---- Present to screen ---- */
        /* Upload the software-rendered framebuffer to the GPU texture,
         * then copy the texture to the screen and flip the display.
         * RenderClear is called before RenderCopy to ensure a clean
         * slate (matters if the window is resized and letterboxed). */
        SDL_UpdateTexture(tex, NULL, fb, IMG_WIDTH * 4);
        SDL_RenderClear(ren);
        SDL_RenderCopy(ren, tex, NULL, NULL);
        SDL_RenderPresent(ren);
    }

    /* ---- Cleanup ---- */
    /* Free all allocated memory and tear down SDL resources in reverse
     * order of creation.  SDL_Quit() handles any remaining subsystem
     * shutdown. */
    free(bg_clean); free(fb); free(ptr8888);
    SDL_DestroyTexture(tex);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
