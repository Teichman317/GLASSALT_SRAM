/*
 * GLASSALT Altimeter Display Simulator
 * Desktop SDL2 renderer for 480x480 AAU-19/A altimeter face
 */
#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#define IMG_WIDTH   480
#define IMG_HEIGHT  480
#define IMG_BPP     2   /* RGB565 */
#define TOTAL_BYTES (IMG_WIDTH * IMG_HEIGHT * IMG_BPP)

/* 100s counter drum strip (reused for baro drums) */
#define STRIP_WIDTH   28
#define STRIP_HEIGHT  567
#define STRIP_DIGITS  10
#define DIGIT_HEIGHT  ((float)STRIP_HEIGHT / STRIP_DIGITS)  /* 56.7 px */

/* Altitude drum position (non-mirrored background) */
#define DRUM_X        164
#define DRUM_Y        174
#define DRUM_VIS_H    104

/* 1000s counter drum strip (cell 0 = hatch, cells 1-10 = digits 0-9) */
#define STRIP1K_WIDTH   45
#define STRIP1K_FULL_H  583   /* full strip including hatch */
#define STRIP1K_HEIGHT  530   /* digit-only portion (10 * 53) */
#define STRIP1K_HATCH   53    /* hatch cell height to skip */
#define STRIP1K_DIGITS  10
#define DIGIT1K_HEIGHT  53.0f /* scaled down 5px per digit */
#define DRUM1K_X        (DRUM_X - 10 - STRIP1K_WIDTH)  /* 10px gap outside 100s */
#define DRUM10K_X       (DRUM1K_X - STRIP1K_WIDTH)     /* immediately left of 1000s */
#define DRUM1K_Y        (DRUM_Y + (DRUM_VIS_H - DRUM1K_VIS_H) / 2)  /* vertically centered with 100s */
#define DRUM1K_VIS_H    51   /* glyph ~41px + 5px top/bottom padding */

/* Baro drums — 4 wheels, 5px gap between each, decimal between drum 1 and 2
 * Layout: [drum0][5px][drum1][dot][drum2][5px][drum3]
 * Total width: 4*28 + 3*5 = 127px (dot sits in the gap between drum1 and drum2) */
#define BARO_GAP      0
#define BARO_Y        280   /* vertical position */
#define BARO_VIS_H    104   /* same visible height as altitude drum */
/* Center the 4 drums horizontally: total = 4*28 + 3*5 = 127, center at 240 */
#define BARO_X0       (240 - (4 * STRIP_WIDTH + 3 * BARO_GAP) / 2)  /* ~176 */

/* Pointer source image */
#define PTR_WIDTH   63
#define PTR_HEIGHT  240
#define PIVOT_X     32
#define PIVOT_Y     50

/* Display center = rotation pivot */
#define CX  240
#define CY  240

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

/* Blit one drum strip into ARGB8888 framebuffer at given x,y position */
static void blit_drum(uint32_t *fb, const uint16_t *strip,
                      int strip_w, int strip_h, float digit_h,
                      int x, int y, int vis_h, float value)
{
    float y_offset_f = value * digit_h;
    int y_offset = (int)y_offset_f;
    int strip_y_start = y_offset - vis_h / 2 + (int)(digit_h / 2);

    for (int row = 0; row < vis_h; row++) {
        int sy = strip_y_start + row;
        while (sy < 0)            sy += strip_h;
        while (sy >= strip_h)     sy -= strip_h;

        int bg_y = y + row;
        if (bg_y < 0 || bg_y >= IMG_HEIGHT) continue;

        for (int col = 0; col < strip_w; col++) {
            int bg_x = x + col;
            if (bg_x < 0 || bg_x >= IMG_WIDTH) continue;

            uint16_t p = strip[sy * strip_w + col];
            uint8_t r = ((p >> 11) & 0x1F) << 3; r |= r >> 5;
            uint8_t g = ((p >>  5) & 0x3F) << 2; g |= g >> 6;
            uint8_t b = ( p        & 0x1F) << 3; b |= b >> 5;
            fb[bg_y * IMG_WIDTH + bg_x] = 0xFF000000 | (r << 16) | (g << 8) | b;
        }
    }
}

/* Draw decimal point (small white square) */
static void draw_decimal(uint32_t *fb, int x, int y, int size)
{
    for (int row = 0; row < size; row++) {
        for (int col = 0; col < size; col++) {
            int px = x + col;
            int py = y + row;
            if (px >= 0 && px < IMG_WIDTH && py >= 0 && py < IMG_HEIGHT)
                fb[py * IMG_WIDTH + px] = 0xFFFFFFFF;  /* white */
        }
    }
}

/* Render all 4 baro drums with Geneva drive carry + decimal point */
static void update_baro_drums(uint32_t *fb, const uint16_t *strip,
                              int strip_w, int strip_h, float digit_h, int vis_h,
                              float baro_inhg)
{
    /* Convert to hundredths: 29.92 → 2992 */
    float baro_val = baro_inhg * 100.0f;

    /* Geneva drive positions */
    float drum3_pos = fmodf(baro_val, 10.0f);          /* hundredths — direct */
    float drum2_raw = fmodf(baro_val / 10.0f, 10.0f);  /* tenths */
    float drum1_raw = fmodf(baro_val / 100.0f, 10.0f); /* ones */
    float drum0_raw = fmodf(baro_val / 1000.0f, 10.0f);/* tens */

    /* Geneva carry: drum gradually advances while right neighbor crosses 9→0.
     * Carry fraction ramps linearly as the right digit goes from 9.0 to 10.0. */
    float carry3 = (drum3_pos >= 9.0f) ? (drum3_pos - 9.0f) : 0.0f;
    float drum2_pos = floorf(drum2_raw) + carry3;
    float carry2 = (drum2_pos >= 9.0f && drum2_pos < 10.0f) ? (drum2_pos - 9.0f) : 0.0f;
    float drum1_pos = floorf(drum1_raw) + carry2;
    float carry1 = (drum1_pos >= 9.0f && drum1_pos < 10.0f) ? (drum1_pos - 9.0f) : 0.0f;
    float drum0_pos = floorf(drum0_raw) + carry1;

    /* Wrap positions to 0-9.999 range */
    while (drum0_pos < 0) drum0_pos += 10.0f;
    while (drum1_pos < 0) drum1_pos += 10.0f;
    while (drum2_pos < 0) drum2_pos += 10.0f;
    while (drum3_pos < 0) drum3_pos += 10.0f;
    drum0_pos = fmodf(drum0_pos, 10.0f);
    drum1_pos = fmodf(drum1_pos, 10.0f);
    drum2_pos = fmodf(drum2_pos, 10.0f);
    drum3_pos = fmodf(drum3_pos, 10.0f);

    /* X positions for each drum: drum0 is leftmost (tens digit) */
    int total_w = 4 * strip_w + 3 * BARO_GAP;
    int baro_x0 = 321 - total_w / 2;  /* center horizontally, shifted right */
    int x0 = baro_x0;
    int x1 = x0 + strip_w + BARO_GAP;
    int x2 = x1 + strip_w + BARO_GAP;
    int x3 = x2 + strip_w + BARO_GAP;

    blit_drum(fb, strip, strip_w, strip_h, digit_h, x0, BARO_Y, vis_h, drum0_pos);
    blit_drum(fb, strip, strip_w, strip_h, digit_h, x1, BARO_Y, vis_h, drum1_pos);
    blit_drum(fb, strip, strip_w, strip_h, digit_h, x2, BARO_Y, vis_h, drum2_pos);
    blit_drum(fb, strip, strip_w, strip_h, digit_h, x3, BARO_Y, vis_h, drum3_pos);
}

/* Rotate pointer and alpha-blend onto framebuffer */
static void rotate_pointer(uint32_t *fb, const uint32_t *ptr, float angle_rad)
{
    float cosA = cosf(angle_rad);
    float sinA = sinf(angle_rad);

    int corners_sx[4] = {0, PTR_WIDTH - 1, 0,             PTR_WIDTH - 1};
    int corners_sy[4] = {0, 0,             PTR_HEIGHT - 1, PTR_HEIGHT - 1};
    int nx0 = IMG_WIDTH, ny0 = IMG_HEIGHT, nx1 = 0, ny1 = 0;
    for (int c = 0; c < 4; c++) {
        float fx = (float)(corners_sx[c] - PIVOT_X);
        float fy = (float)(corners_sy[c] - PIVOT_Y);
        int dx = CX + (int)(fx * cosA - fy * sinA);
        int dy = CY + (int)(fx * sinA + fy * cosA);
        if (dx < nx0) nx0 = dx;
        if (dx > nx1) nx1 = dx;
        if (dy < ny0) ny0 = dy;
        if (dy > ny1) ny1 = dy;
    }
    if (nx0 < 0) nx0 = 0;
    if (ny0 < 0) ny0 = 0;
    if (nx1 >= IMG_WIDTH)  nx1 = IMG_WIDTH - 1;
    if (ny1 >= IMG_HEIGHT) ny1 = IMG_HEIGHT - 1;

    for (int dy = ny0; dy <= ny1; dy++) {
        for (int dx = nx0; dx <= nx1; dx++) {
            float fx = (float)(dx - CX);
            float fy = (float)(dy - CY);
            int sx = PIVOT_X + (int)(fx * cosA + fy * sinA + 0.5f);
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
        free(buf);
        return NULL;
    }
    return buf;
}

int main(int argc, char *argv[])
{
    (void)argc; (void)argv;

    /* Load assets */
    uint16_t *bg565 = load_bin("../image.bin", TOTAL_BYTES);
    if (!bg565) return 1;

    uint16_t *strip = load_bin("../100sWheel.bin", STRIP_WIDTH * STRIP_HEIGHT * 2);
    if (!strip) return 1;

    uint16_t *ptr4444 = load_bin("../pointerargb4444.bin", PTR_WIDTH * PTR_HEIGHT * 2);
    if (!ptr4444) return 1;

    uint16_t *strip1k_full = load_bin("../1000sWheel.bin", STRIP1K_WIDTH * STRIP1K_FULL_H * 2);
    if (!strip1k_full) return 1;
    /* Skip hatch cell — point to digit 0 onward */
    uint16_t *strip1k = strip1k_full + STRIP1K_WIDTH * STRIP1K_HATCH;

    /* 10,000s strip: [hatch][1][2]...[9] — hatch replaces 0 */
    uint16_t *strip10k = load_bin("../10000sWheel.bin", STRIP1K_WIDTH * STRIP1K_HEIGHT * 2);
    if (!strip10k) return 1;

    /* Create inverted + repacked + cropped strip for baro drums.
     * White digits on black, 10px vertical gap, trimmed horizontal padding.
     * Original strip has 5px padding on each side — keep 1px each side. */
    int orig_cell = (int)(DIGIT_HEIGHT + 0.5f);
    int crop_left = 2;    /* skip 2px from left (keep 3px padding) */
    int crop_right = 25;  /* last column to keep (keep 3px padding) */
    int baro_strip_w = crop_right - crop_left + 1;  /* 20px wide */

    /* First pass: find glyph vertical bounds */
    int glyph_top = orig_cell, glyph_bot = 0;
    for (int d = 0; d < STRIP_DIGITS; d++) {
        int cell_y = (int)(d * DIGIT_HEIGHT);
        for (int row = 0; row < orig_cell && (cell_y + row) < STRIP_HEIGHT; row++) {
            int has_pixel = 0;
            for (int col = 0; col < STRIP_WIDTH; col++) {
                uint16_t p = strip[(cell_y + row) * STRIP_WIDTH + col];
                uint8_t r = ((p >> 11) & 0x1F) << 3;
                uint8_t g = ((p >>  5) & 0x3F) << 2;
                uint8_t b = ( p        & 0x1F) << 3;
                int lum = (r * 77 + g * 150 + b * 29) >> 8;
                if (lum < 128) { has_pixel = 1; break; }
            }
            if (has_pixel) {
                if (row < glyph_top) glyph_top = row;
                if (row > glyph_bot) glyph_bot = row;
            }
        }
    }
    int glyph_h = glyph_bot - glyph_top + 1;
    int baro_gap_v = 10;
    int baro_cell = glyph_h + baro_gap_v;
    int baro_strip_h = baro_cell * STRIP_DIGITS;

    uint16_t *baro_strip = calloc(baro_strip_w * baro_strip_h, 2);
    if (!baro_strip) return 1;

    for (int d = 0; d < STRIP_DIGITS; d++) {
        int src_y = (int)(d * DIGIT_HEIGHT) + glyph_top;
        int dst_y = d * baro_cell + baro_gap_v / 2;
        for (int row = 0; row < glyph_h; row++) {
            int sy = src_y + row;
            if (sy >= STRIP_HEIGHT) break;
            for (int col = 0; col < baro_strip_w; col++) {
                int src_col = crop_left + col;
                uint16_t p = strip[sy * STRIP_WIDTH + src_col];
                uint8_t r = ((p >> 11) & 0x1F) << 3;
                uint8_t g = ((p >>  5) & 0x3F) << 2;
                uint8_t b = ( p        & 0x1F) << 3;
                int lum = (r * 77 + g * 150 + b * 29) >> 8;
                baro_strip[(dst_y + row) * baro_strip_w + col] = (lum < 128) ? 0xFFFF : 0x0000;
            }
        }
    }

    int baro_strip_height = baro_strip_h;
    float baro_digit_height = (float)baro_cell;

    /* Convert background to ARGB8888 (pristine copy) */
    uint32_t *bg_clean = malloc(IMG_WIDTH * IMG_HEIGHT * 4);
    uint32_t *fb       = malloc(IMG_WIDTH * IMG_HEIGHT * 4);
    if (!bg_clean || !fb) return 1;
    rgb565_to_argb8888(bg565, bg_clean, IMG_WIDTH * IMG_HEIGHT);
    free(bg565);

    /* Convert pointer to ARGB8888 */
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
        "GLASSALT AAU-19/A Altimeter",
        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
        IMG_WIDTH, IMG_HEIGHT,
        SDL_WINDOW_SHOWN);
    if (!win) return 1;

    SDL_Renderer *ren = SDL_CreateRenderer(win, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!ren) return 1;

    SDL_Texture *tex = SDL_CreateTexture(ren,
        SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
        IMG_WIDTH, IMG_HEIGHT);
    if (!tex) return 1;

    /* State */
    float altitude = 8000.0f;
    float baro_inhg = 29.92f;  /* default barometric pressure */
    float baro_anim = 0.0f;    /* animated baro offset in hundredths */
    Uint32 last_tick = SDL_GetTicks();

    int running = 1;
    while (running) {
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT) running = 0;
            if (ev.type == SDL_KEYDOWN) {
                switch (ev.key.keysym.sym) {
                case SDLK_ESCAPE: running = 0; break;
                case SDLK_UP:    baro_inhg += 0.01f; break;
                case SDLK_DOWN:  baro_inhg -= 0.01f; break;
                default: break;
                }
                /* Clamp baro range */
                if (baro_inhg > 31.00f) baro_inhg = 31.00f;
                if (baro_inhg < 28.10f) baro_inhg = 28.10f;
            }
        }

        Uint32 now = SDL_GetTicks();
        float dt = (now - last_tick) / 1000.0f;
        last_tick = now;

        /* Triangle wave 8000→12000→8000 at 200 ft/sec */
        static float alt_dir = 1.0f;
        altitude += dt * 200.0f * alt_dir;
        if (altitude >= 12000.0f) { altitude = 12000.0f; alt_dir = -1.0f; }
        if (altitude <= 8000.0f)  { altitude = 8000.0f;  alt_dir =  1.0f; }

        /* Animate baro — slowly rotate the hundredths digit */
        baro_anim += dt * 0.5f;  /* +0.5 hundredths per second */

        float pointer_angle = (altitude / 1000.0f) * 2.0f * (float)M_PI;
        float drum_val = fmodf(altitude / 100.0f + 5.0f, 10.0f);

        /* 1000s drum: transition as pointer sweeps 0→1 at each 1k boundary */
        float drum1k_raw = fmodf(altitude / 1000.0f, 10.0f);
        float within_1k = fmodf(altitude, 1000.0f);
        float drum1k_val;
        if (within_1k < 100.0f && altitude >= 100.0f) {
            float t = within_1k / 100.0f;
            drum1k_val = fmodf(floorf(drum1k_raw) - 1.0f + t + 10.0f, 10.0f);
        } else {
            drum1k_val = floorf(drum1k_raw);
        }

        /* 10,000s drum: transition as pointer sweeps 0→1 at each 10k boundary */
        float drum10k_raw = fmodf(altitude / 10000.0f, 10.0f);
        float within_10k = fmodf(altitude, 10000.0f);
        float drum10k_val;
        if (within_10k < 100.0f && altitude >= 100.0f) {
            float t = within_10k / 100.0f;
            drum10k_val = fmodf(floorf(drum10k_raw) - 1.0f + t + 10.0f, 10.0f);
        } else {
            drum10k_val = floorf(drum10k_raw);
        }

        /* Render */
        memcpy(fb, bg_clean, IMG_WIDTH * IMG_HEIGHT * 4);
        blit_drum(fb, strip, STRIP_WIDTH, STRIP_HEIGHT, DIGIT_HEIGHT, DRUM_X, DRUM_Y, DRUM_VIS_H, drum_val);
        blit_drum(fb, strip1k, STRIP1K_WIDTH, STRIP1K_HEIGHT, DIGIT1K_HEIGHT, DRUM1K_X, DRUM1K_Y, DRUM1K_VIS_H, drum1k_val);
        blit_drum(fb, strip10k, STRIP1K_WIDTH, STRIP1K_HEIGHT, DIGIT1K_HEIGHT, DRUM10K_X, DRUM1K_Y, DRUM1K_VIS_H, drum10k_val);
        int baro_vis = (int)(baro_digit_height * 1.1f);
        float baro_display = baro_inhg + baro_anim * 0.01f;
        update_baro_drums(fb, baro_strip, baro_strip_w, baro_strip_height, baro_digit_height, baro_vis, baro_display);
        rotate_pointer(fb, ptr8888, pointer_angle);

        SDL_UpdateTexture(tex, NULL, fb, IMG_WIDTH * 4);
        SDL_RenderClear(ren);
        SDL_RenderCopy(ren, tex, NULL, NULL);
        SDL_RenderPresent(ren);
    }

    free(bg_clean); free(fb); free(ptr8888); free(strip); free(strip1k_full); free(strip10k); free(baro_strip);
    SDL_DestroyTexture(tex);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
