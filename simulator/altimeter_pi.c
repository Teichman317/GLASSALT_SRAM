/*
 * Altimeter Receiver — runs on Raspberry Pi, receives UDP from host
 * 480x480 AAU-19/A altimeter face with counter drums + rotating pointer
 *
 * Usage: ./altimeter_pi [port]
 *        Default port: 5557
 *        ESC to quit
 *
 * Runs demo animation until first UDP packet received.
 */
#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <unistd.h>

/* ---- UDP packet ---- */
typedef struct {
    uint32_t instrument_id;     /* 1 = altimeter */
    float    altitude;          /* feet */
    float    baro_inhg;         /* inches Hg */
} AltPacket;
#define INSTRUMENT_ALT  1

/* ---- Display ---- */
#define W   480
#define H   480
#define CX  240
#define CY  240

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* 100s counter drum strip */
#define STRIP_WIDTH   28
#define STRIP_HEIGHT  567
#define STRIP_DIGITS  10
#define DIGIT_HEIGHT  ((float)STRIP_HEIGHT / STRIP_DIGITS)

/* Altitude drum position */
#define DRUM_X        164
#define DRUM_Y        174
#define DRUM_VIS_H    104

/* 1000s / 10000s drum strip */
#define STRIP1K_WIDTH   45
#define STRIP1K_FULL_H  583
#define STRIP1K_HEIGHT  530
#define STRIP1K_HATCH   53
#define STRIP1K_DIGITS  10
#define DIGIT1K_HEIGHT  53.0f
#define DRUM1K_X        (DRUM_X - 10 - STRIP1K_WIDTH)
#define DRUM10K_X       (DRUM1K_X - STRIP1K_WIDTH)
#define DRUM1K_Y        (DRUM_Y + (DRUM_VIS_H - DRUM1K_VIS_H) / 2)
#define DRUM1K_VIS_H    51

/* Baro drums */
#define BARO_GAP      0
#define BARO_Y        280
#define BARO_VIS_H    104

/* Pointer source image */
#define PTR_WIDTH   63
#define PTR_HEIGHT  240
#define PIVOT_X     32
#define PIVOT_Y     50

/* ================================================================== */
/*  Image helpers                                                     */
/* ================================================================== */

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

/* ================================================================== */
/*  Drum rendering                                                    */
/* ================================================================== */

static void blit_drum(uint32_t *fb, const uint16_t *strip,
                      int strip_w, int strip_h, float digit_h,
                      int x, int y, int vis_h, float value)
{
    float y_offset_f = value * digit_h;
    int y_offset = (int)y_offset_f;
    int strip_y_start = y_offset - vis_h / 2 + (int)(digit_h / 2);

    for (int row = 0; row < vis_h; row++) {
        int sy = strip_y_start + row;
        while (sy < 0)        sy += strip_h;
        while (sy >= strip_h) sy -= strip_h;
        int bg_y = y + row;
        if (bg_y < 0 || bg_y >= H) continue;
        for (int col = 0; col < strip_w; col++) {
            int bg_x = x + col;
            if (bg_x < 0 || bg_x >= W) continue;
            uint16_t p = strip[sy * strip_w + col];
            uint8_t r = ((p >> 11) & 0x1F) << 3; r |= r >> 5;
            uint8_t g = ((p >>  5) & 0x3F) << 2; g |= g >> 6;
            uint8_t b = ( p        & 0x1F) << 3; b |= b >> 5;
            fb[bg_y * W + bg_x] = 0xFF000000 | (r << 16) | (g << 8) | b;
        }
    }
}

static void update_baro_drums(uint32_t *fb, const uint16_t *strip,
                              int strip_w, int strip_h, float digit_h, int vis_h,
                              float baro_inhg)
{
    float baro_val = baro_inhg * 100.0f;
    float drum3_pos = fmodf(baro_val, 10.0f);
    float drum2_raw = fmodf(baro_val / 10.0f, 10.0f);
    float drum1_raw = fmodf(baro_val / 100.0f, 10.0f);
    float drum0_raw = fmodf(baro_val / 1000.0f, 10.0f);

    float carry3 = (drum3_pos >= 9.0f) ? (drum3_pos - 9.0f) : 0.0f;
    float drum2_pos = floorf(drum2_raw) + carry3;
    float carry2 = (drum2_pos >= 9.0f && drum2_pos < 10.0f) ? (drum2_pos - 9.0f) : 0.0f;
    float drum1_pos = floorf(drum1_raw) + carry2;
    float carry1 = (drum1_pos >= 9.0f && drum1_pos < 10.0f) ? (drum1_pos - 9.0f) : 0.0f;
    float drum0_pos = floorf(drum0_raw) + carry1;

    drum0_pos = fmodf(drum0_pos + 10.0f, 10.0f);
    drum1_pos = fmodf(drum1_pos + 10.0f, 10.0f);
    drum2_pos = fmodf(drum2_pos + 10.0f, 10.0f);
    drum3_pos = fmodf(drum3_pos + 10.0f, 10.0f);

    int total_w = 4 * strip_w + 3 * BARO_GAP;
    int baro_x0 = 321 - total_w / 2;
    int x0 = baro_x0;
    int x1 = x0 + strip_w + BARO_GAP;
    int x2 = x1 + strip_w + BARO_GAP;
    int x3 = x2 + strip_w + BARO_GAP;

    blit_drum(fb, strip, strip_w, strip_h, digit_h, x0, BARO_Y, vis_h, drum0_pos);
    blit_drum(fb, strip, strip_w, strip_h, digit_h, x1, BARO_Y, vis_h, drum1_pos);
    blit_drum(fb, strip, strip_w, strip_h, digit_h, x2, BARO_Y, vis_h, drum2_pos);
    blit_drum(fb, strip, strip_w, strip_h, digit_h, x3, BARO_Y, vis_h, drum3_pos);
}

/* ================================================================== */
/*  Pointer rotation                                                  */
/* ================================================================== */

static void rotate_pointer(uint32_t *fb, const uint32_t *ptr, float angle_rad)
{
    float cosA = cosf(angle_rad), sinA = sinf(angle_rad);
    int corners_sx[4] = {0, PTR_WIDTH-1, 0, PTR_WIDTH-1};
    int corners_sy[4] = {0, 0, PTR_HEIGHT-1, PTR_HEIGHT-1};
    int nx0 = W, ny0 = H, nx1 = 0, ny1 = 0;
    for (int c = 0; c < 4; c++) {
        float fx = (float)(corners_sx[c] - PIVOT_X);
        float fy = (float)(corners_sy[c] - PIVOT_Y);
        int dx = CX + (int)(fx*cosA - fy*sinA);
        int dy = CY + (int)(fx*sinA + fy*cosA);
        if (dx < nx0) nx0 = dx; if (dx > nx1) nx1 = dx;
        if (dy < ny0) ny0 = dy; if (dy > ny1) ny1 = dy;
    }
    if (nx0 < 0) nx0 = 0; if (ny0 < 0) ny0 = 0;
    if (nx1 >= W) nx1 = W-1; if (ny1 >= H) ny1 = H-1;

    for (int dy = ny0; dy <= ny1; dy++) {
        for (int dx = nx0; dx <= nx1; dx++) {
            float fx = (float)(dx-CX), fy = (float)(dy-CY);
            int sx = PIVOT_X + (int)( fx*cosA + fy*sinA + 0.5f);
            int sy = PIVOT_Y + (int)(-fx*sinA + fy*cosA + 0.5f);
            if (sx < 0 || sx >= PTR_WIDTH || sy < 0 || sy >= PTR_HEIGHT) continue;
            uint32_t sp = ptr[sy*PTR_WIDTH+sx];
            uint8_t sa = (sp >> 24) & 0xFF;
            if (sa == 0) continue;
            if (sa == 255) { fb[dy*W+dx] = sp; continue; }
            uint32_t dp = fb[dy*W+dx];
            uint8_t ia = 255 - sa;
            fb[dy*W+dx] = 0xFF000000 |
                ((((sp>>16&0xFF)*sa + (dp>>16&0xFF)*ia)/255) << 16) |
                ((((sp>>8&0xFF)*sa  + (dp>>8&0xFF)*ia)/255) << 8) |
                (((sp&0xFF)*sa + (dp&0xFF)*ia)/255);
        }
    }
}

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

/* ================================================================== */
int main(int argc, char *argv[])
{
    int port = 5557;
    if (argc > 1) port = atoi(argv[1]);

    /* ---- UDP socket ---- */
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); return 1; }
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind"); close(sock); return 1;
    }
    fcntl(sock, F_SETFL, O_NONBLOCK);
    printf("=== Altimeter Receiver on port %d ===\n", port);

    /* ---- Load assets ---- */
    uint16_t *bg565 = load_bin("../image.bin", W*H*2);
    if (!bg565) return 1;
    uint16_t *strip = load_bin("../100sWheel.bin", STRIP_WIDTH*STRIP_HEIGHT*2);
    if (!strip) return 1;
    uint16_t *ptr4444 = load_bin("../pointerargb4444.bin", PTR_WIDTH*PTR_HEIGHT*2);
    if (!ptr4444) return 1;
    uint16_t *strip1k_full = load_bin("../1000sWheel.bin", STRIP1K_WIDTH*STRIP1K_FULL_H*2);
    if (!strip1k_full) return 1;
    uint16_t *strip1k = strip1k_full + STRIP1K_WIDTH * STRIP1K_HATCH;
    uint16_t *strip10k = load_bin("../10000sWheel.bin", STRIP1K_WIDTH*STRIP1K_HEIGHT*2);
    if (!strip10k) return 1;

    /* ---- Build baro strip (inverted + repacked + cropped) ---- */
    int orig_cell = (int)(DIGIT_HEIGHT + 0.5f);
    int crop_left = 2, crop_right = 25;
    int baro_strip_w = crop_right - crop_left + 1;

    int glyph_top = orig_cell, glyph_bot = 0;
    for (int d = 0; d < STRIP_DIGITS; d++) {
        int cell_y = (int)(d * DIGIT_HEIGHT);
        for (int row = 0; row < orig_cell && (cell_y+row) < STRIP_HEIGHT; row++) {
            int has_pixel = 0;
            for (int col = 0; col < STRIP_WIDTH; col++) {
                uint16_t p = strip[(cell_y+row)*STRIP_WIDTH+col];
                int lum = ((((p>>11)&0x1F)<<3)*77 + (((p>>5)&0x3F)<<2)*150 + (((p&0x1F)<<3))*29) >> 8;
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
    float baro_digit_height = (float)baro_cell;

    uint16_t *baro_strip = calloc(baro_strip_w * baro_strip_h, 2);
    for (int d = 0; d < STRIP_DIGITS; d++) {
        int src_y = (int)(d * DIGIT_HEIGHT) + glyph_top;
        int dst_y = d * baro_cell + baro_gap_v / 2;
        for (int row = 0; row < glyph_h; row++) {
            int sy = src_y + row;
            if (sy >= STRIP_HEIGHT) break;
            for (int col = 0; col < baro_strip_w; col++) {
                uint16_t p = strip[(sy)*STRIP_WIDTH + crop_left + col];
                int lum = ((((p>>11)&0x1F)<<3)*77 + (((p>>5)&0x3F)<<2)*150 + (((p&0x1F)<<3))*29) >> 8;
                baro_strip[(dst_y+row)*baro_strip_w+col] = (lum < 128) ? 0xFFFF : 0x0000;
            }
        }
    }

    /* ---- Convert to ARGB8888 ---- */
    uint32_t *bg_clean = malloc(W*H*4);
    uint32_t *fb = malloc(W*H*4);
    uint32_t *ptr8888 = malloc(PTR_WIDTH*PTR_HEIGHT*4);
    rgb565_to_argb8888(bg565, bg_clean, W*H); free(bg565);
    argb4444_to_argb8888(ptr4444, ptr8888, PTR_WIDTH*PTR_HEIGHT); free(ptr4444);

    /* ---- SDL ---- */
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window *win = SDL_CreateWindow("Altimeter",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, W, H, 0);
    SDL_Renderer *ren = SDL_CreateRenderer(win, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    SDL_RenderSetLogicalSize(ren, W, H);
    SDL_Texture *tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING, W, H);

    /* ---- State ---- */
    float altitude = 8000.0f;
    float baro_inhg = 29.92f;
    int got_data = 0;
    float alt_dir = 1.0f;
    Uint32 last_tick = SDL_GetTicks();

    int running = 1;
    while (running) {
        Uint32 now = SDL_GetTicks();
        float dt = (now - last_tick) / 1000.0f;
        if (dt > 0.1f) dt = 0.1f;
        last_tick = now;

        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT) running = 0;
            if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_ESCAPE)
                running = 0;
        }

        /* ---- Receive UDP ---- */
        AltPacket pkt;
        ssize_t n;
        while ((n = recvfrom(sock, &pkt, sizeof(pkt), 0, NULL, NULL)) > 0) {
            if (n >= (ssize_t)sizeof(pkt) && pkt.instrument_id == INSTRUMENT_ALT) {
                altitude = pkt.altitude;
                baro_inhg = pkt.baro_inhg;
                got_data = 1;
            }
        }

        /* Demo animation if no UDP */
        if (!got_data) {
            altitude += dt * 200.0f * alt_dir;
            if (altitude >= 12000) { altitude = 12000; alt_dir = -1; }
            if (altitude <= 8000)  { altitude = 8000;  alt_dir = 1; }
        }

        /* ---- Compute drum positions ---- */
        float pointer_angle = (altitude / 1000.0f) * 2.0f * (float)M_PI;
        float drum_val = fmodf(altitude / 100.0f + 5.0f, 10.0f);

        float drum1k_raw = fmodf(altitude / 1000.0f, 10.0f);
        float within_1k = fmodf(altitude, 1000.0f);
        float drum1k_val;
        if (within_1k < 100.0f && altitude >= 100.0f) {
            float t = within_1k / 100.0f;
            drum1k_val = fmodf(floorf(drum1k_raw) - 1.0f + t + 10.0f, 10.0f);
        } else {
            drum1k_val = floorf(drum1k_raw);
        }

        float drum10k_raw = fmodf(altitude / 10000.0f, 10.0f);
        float within_10k = fmodf(altitude, 10000.0f);
        float drum10k_val;
        if (within_10k < 100.0f && altitude >= 100.0f) {
            float t = within_10k / 100.0f;
            drum10k_val = fmodf(floorf(drum10k_raw) - 1.0f + t + 10.0f, 10.0f);
        } else {
            drum10k_val = floorf(drum10k_raw);
        }

        /* ---- Render ---- */
        memcpy(fb, bg_clean, W*H*4);
        blit_drum(fb, strip, STRIP_WIDTH, STRIP_HEIGHT, DIGIT_HEIGHT,
                  DRUM_X, DRUM_Y, DRUM_VIS_H, drum_val);
        blit_drum(fb, strip1k, STRIP1K_WIDTH, STRIP1K_HEIGHT, DIGIT1K_HEIGHT,
                  DRUM1K_X, DRUM1K_Y, DRUM1K_VIS_H, drum1k_val);
        blit_drum(fb, strip10k, STRIP1K_WIDTH, STRIP1K_HEIGHT, DIGIT1K_HEIGHT,
                  DRUM10K_X, DRUM1K_Y, DRUM1K_VIS_H, drum10k_val);

        int baro_vis = (int)(baro_digit_height * 1.1f);
        update_baro_drums(fb, baro_strip, baro_strip_w, baro_strip_h,
                          baro_digit_height, baro_vis, baro_inhg);
        rotate_pointer(fb, ptr8888, pointer_angle);

        SDL_UpdateTexture(tex, NULL, fb, W*4);
        SDL_RenderCopy(ren, tex, NULL, NULL);
        SDL_RenderPresent(ren);
    }

    close(sock);
    free(bg_clean); free(fb); free(ptr8888);
    free(strip); free(strip1k_full); free(strip10k); free(baro_strip);
    SDL_DestroyTexture(tex); SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win); SDL_Quit();
    return 0;
}
