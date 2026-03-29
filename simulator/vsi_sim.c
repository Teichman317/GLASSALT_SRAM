/*
 * GLASSALT VSI (Vertical Speed Indicator) Simulator
 * Desktop SDL2 renderer for 480x480 instrument face
 *
 * The VSI has a non-linear scale — sensitive at low rates,
 * compressed at high rates — just like the real instrument.
 * We use a lookup table measured from the actual dial artwork
 * and interpolate between points.
 */
#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#define IMG_WIDTH   480
#define IMG_HEIGHT  480

/* Pointer source image */
#define PTR_WIDTH   363
#define PTR_HEIGHT  60
#define PIVOT_X     202   /* shifted left 20px to shorten visible needle */
#define PIVOT_Y     30    /* vertical center */

/* Display center = rotation pivot on screen */
#define CX  240
#define CY  240

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
 * Non-linear VSI scale lookup table.
 * Maps vertical speed (fpm) to angle offset in degrees
 * from the zero position (9 o'clock).
 * Positive = clockwise (climb), negative = counter-clockwise (descent).
 * Measured from the actual dial tick mark positions.
 */
typedef struct { float fpm; float deg; } ScalePoint;

static const ScalePoint vsi_scale[] = {
    { -2500, -151.0f },
    { -2000, -127.0f },
    { -1500,  -90.0f },
    { -1000,  -57.5f },
    {  -500,  -25.0f },
    {     0,    0.0f },
    {   500,   24.0f },
    {  1000,   57.0f },
    {  1500,   90.0f },
    {  2000,  129.0f },
    {  2500,  153.0f },
    {  3000,  174.0f },
};
#define SCALE_COUNT (sizeof(vsi_scale) / sizeof(vsi_scale[0]))

/* Piecewise linear interpolation through the scale table */
static float vsi_fpm_to_deg(float fpm)
{
    /* Clamp to scale range */
    if (fpm <= vsi_scale[0].fpm) return vsi_scale[0].deg;
    if (fpm >= vsi_scale[SCALE_COUNT-1].fpm) return vsi_scale[SCALE_COUNT-1].deg;

    /* Find the two surrounding points and interpolate */
    for (int i = 0; i < (int)SCALE_COUNT - 1; i++) {
        if (fpm >= vsi_scale[i].fpm && fpm <= vsi_scale[i+1].fpm) {
            float t = (fpm - vsi_scale[i].fpm)
                    / (vsi_scale[i+1].fpm - vsi_scale[i].fpm);
            return vsi_scale[i].deg + t * (vsi_scale[i+1].deg - vsi_scale[i].deg);
        }
    }
    return 0.0f;
}

/* Convert RGB565 to ARGB8888 */
static void rgb565_to_argb8888(const uint16_t *src, uint32_t *dst, int count)
{
    for (int i = 0; i < count; i++) {
        uint16_t p = src[i];
        uint8_t r = ((p >> 11) & 0x1F) << 3; r |= r >> 5;
        uint8_t g = ((p >>  5) & 0x3F) << 2; g |= g >> 6;
        uint8_t b = ( p        & 0x1F) << 3; b |= b >> 5;
        dst[i] = 0xFF000000 | (r << 16) | (g << 8) | b;
    }
}

/* Convert ARGB4444 to ARGB8888 */
static void argb4444_to_argb8888(const uint16_t *src, uint32_t *dst, int count)
{
    for (int i = 0; i < count; i++) {
        uint16_t p = src[i];
        uint8_t a = ((p >> 12) & 0xF) * 17;
        uint8_t r = ((p >>  8) & 0xF) * 17;
        uint8_t g = ((p >>  4) & 0xF) * 17;
        uint8_t b = ( p        & 0xF) * 17;
        dst[i] = (a << 24) | (r << 16) | (g << 8) | b;
    }
}

/* Rotate pointer and alpha-blend onto framebuffer */
static void rotate_pointer(uint32_t *fb, const uint32_t *ptr, float angle_rad)
{
    float cosA = cosf(angle_rad);
    float sinA = sinf(angle_rad);

    /* Compute bounding box of rotated pointer */
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
    if (nx0 < 0) nx0 = 0;
    if (ny0 < 0) ny0 = 0;
    if (nx1 >= IMG_WIDTH)  nx1 = IMG_WIDTH - 1;
    if (ny1 >= IMG_HEIGHT) ny1 = IMG_HEIGHT - 1;

    /* Inverse-map each destination pixel back to source */
    for (int dy = ny0; dy <= ny1; dy++) {
        for (int dx = nx0; dx <= nx1; dx++) {
            float fx = (float)(dx - CX);
            float fy = (float)(dy - CY);
            int sx = PIVOT_X + (int)( fx * cosA + fy * sinA + 0.5f);
            int sy = PIVOT_Y + (int)(-fx * sinA + fy * cosA + 0.5f);

            if (sx < 0 || sx >= PTR_WIDTH || sy < 0 || sy >= PTR_HEIGHT)
                continue;

            uint32_t sp = ptr[sy * PTR_WIDTH + sx];
            uint8_t sa = (sp >> 24) & 0xFF;
            if (sa == 0) continue;

            if (sa == 255) {
                fb[dy * IMG_WIDTH + dx] = sp;
            } else {
                uint32_t dp = fb[dy * IMG_WIDTH + dx];
                uint8_t sr = (sp >> 16) & 0xFF, sg = (sp >> 8) & 0xFF, sb = sp & 0xFF;
                uint8_t dr = (dp >> 16) & 0xFF, dg = (dp >> 8) & 0xFF, db = dp & 0xFF;
                uint8_t ia = 255 - sa;
                uint8_t or_ = (sr * sa + dr * ia) / 255;
                uint8_t og  = (sg * sa + dg * ia) / 255;
                uint8_t ob  = (sb * sa + db * ia) / 255;
                fb[dy * IMG_WIDTH + dx] = 0xFF000000 | (or_ << 16) | (og << 8) | ob;
            }
        }
    }
}

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

int main(int argc, char *argv[])
{
    (void)argc; (void)argv;

    /* Load assets from "Vertical speed" subdirectory */
    uint16_t *bg565 = load_bin("../Vertical speed/vsi_bg.bin",
                               IMG_WIDTH * IMG_HEIGHT * 2);
    if (!bg565) return 1;

    uint16_t *ptr4444 = load_bin("../Vertical speed/vsi_pointer.bin",
                                 PTR_WIDTH * PTR_HEIGHT * 2);
    if (!ptr4444) return 1;

    /* Convert to ARGB8888 */
    uint32_t *bg_clean = malloc(IMG_WIDTH * IMG_HEIGHT * 4);
    uint32_t *fb       = malloc(IMG_WIDTH * IMG_HEIGHT * 4);
    if (!bg_clean || !fb) return 1;
    rgb565_to_argb8888(bg565, bg_clean, IMG_WIDTH * IMG_HEIGHT);
    free(bg565);

    uint32_t *ptr8888 = malloc(PTR_WIDTH * PTR_HEIGHT * 4);
    if (!ptr8888) return 1;
    argb4444_to_argb8888(ptr4444, ptr8888, PTR_WIDTH * PTR_HEIGHT);
    free(ptr4444);

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

    SDL_Texture *tex = SDL_CreateTexture(ren,
        SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
        IMG_WIDTH, IMG_HEIGHT);
    if (!tex) return 1;

    /* State: vertical speed in fpm, sweeps -1500 to +2000 */
    float vsi_fpm = 0.0f;
    float vsi_dir = 1.0f;
    Uint32 last_tick = SDL_GetTicks();

    int running = 1;
    while (running) {
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT) running = 0;
            if (ev.type == SDL_KEYDOWN) {
                if (ev.key.keysym.sym == SDLK_ESCAPE) running = 0;
            }
        }

        Uint32 now = SDL_GetTicks();
        float dt = (now - last_tick) / 1000.0f;
        last_tick = now;

        /* Animate: triangle wave -1500 to +2000 fpm at 300 fpm/sec */
        vsi_fpm += dt * 300.0f * vsi_dir;
        if (vsi_fpm >=  2000.0f) { vsi_fpm =  2000.0f; vsi_dir = -1.0f; }
        if (vsi_fpm <= -1500.0f) { vsi_fpm = -1500.0f; vsi_dir =  1.0f; }

        /* Convert vertical speed to pointer angle */
        float angle_deg = vsi_fpm_to_deg(vsi_fpm);
        float angle_rad = angle_deg * (float)M_PI / 180.0f;

        /* Render: clean background + rotated pointer */
        memcpy(fb, bg_clean, IMG_WIDTH * IMG_HEIGHT * 4);
        rotate_pointer(fb, ptr8888, angle_rad);

        SDL_UpdateTexture(tex, NULL, fb, IMG_WIDTH * 4);
        SDL_RenderClear(ren);
        SDL_RenderCopy(ren, tex, NULL, NULL);
        SDL_RenderPresent(ren);
    }

    free(bg_clean); free(fb); free(ptr8888);
    SDL_DestroyTexture(tex);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
