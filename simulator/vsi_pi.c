/*
 * VSI Receiver — runs on Raspberry Pi, receives UDP from host
 * 480x480 Vertical Speed Indicator, no controls
 *
 * Usage: ./vsi_pi [port]
 *        Default port: 5555
 *        ESC or close window to quit
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

#define W   480
#define H   480
#define CX  240
#define CY  240

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---- UDP packet (must match vsi_host.c) ---- */
typedef struct {
    uint32_t instrument_id;     /* 2 = VSI */
    float    value;             /* vertical speed in fpm */
} SimPacket;

#define INSTRUMENT_VSI  2

/* ---- Pointer source image ---- */
#define PTR_WIDTH   363
#define PTR_HEIGHT  60
#define PIVOT_X     202
#define PIVOT_Y     30

/* ---- Non-linear VSI scale ---- */
typedef struct { float fpm; float deg; } ScalePoint;

static const ScalePoint vsi_scale[] = {
    { -2000, -151.0f }, { -1500, -127.0f }, { -1000, -90.0f },
    { -500, -25.0f }, { 0, 0.0f }, { 500, 24.0f },
    { 1000, 90.0f }, { 1500, 129.0f }, { 2000, 153.0f },
    { 2500, 174.0f }, { 3000, 188.0f },
};
#define SCALE_COUNT (sizeof(vsi_scale) / sizeof(vsi_scale[0]))

static float vsi_fpm_to_deg(float fpm)
{
    if (fpm <= vsi_scale[0].fpm) return vsi_scale[0].deg;
    if (fpm >= vsi_scale[SCALE_COUNT-1].fpm) return vsi_scale[SCALE_COUNT-1].deg;
    for (int i = 0; i < (int)SCALE_COUNT - 1; i++) {
        if (fpm >= vsi_scale[i].fpm && fpm <= vsi_scale[i+1].fpm) {
            float t = (fpm - vsi_scale[i].fpm)
                    / (vsi_scale[i+1].fpm - vsi_scale[i].fpm);
            return vsi_scale[i].deg + t * (vsi_scale[i+1].deg - vsi_scale[i].deg);
        }
    }
    return 0.0f;
}

/* ---- Image helpers ---- */

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

static void rotate_pointer(uint32_t *fb, const uint32_t *ptr, float angle_rad)
{
    float cosA = cosf(angle_rad), sinA = sinf(angle_rad);
    int corners_sx[4] = {0, PTR_WIDTH-1, 0, PTR_WIDTH-1};
    int corners_sy[4] = {0, 0, PTR_HEIGHT-1, PTR_HEIGHT-1};
    int nx0 = W, ny0 = H, nx1 = 0, ny1 = 0;
    for (int c = 0; c < 4; c++) {
        float fx = (float)(corners_sx[c] - PIVOT_X);
        float fy = (float)(corners_sy[c] - PIVOT_Y);
        int dx = CX + (int)(fx * cosA - fy * sinA);
        int dy = CY + (int)(fx * sinA + fy * cosA);
        if (dx < nx0) nx0 = dx; if (dx > nx1) nx1 = dx;
        if (dy < ny0) ny0 = dy; if (dy > ny1) ny1 = dy;
    }
    if (nx0 < 0) nx0 = 0; if (ny0 < 0) ny0 = 0;
    if (nx1 >= W) nx1 = W-1; if (ny1 >= H) ny1 = H-1;

    for (int dy = ny0; dy <= ny1; dy++) {
        for (int dx = nx0; dx <= nx1; dx++) {
            float fx = (float)(dx - CX), fy = (float)(dy - CY);
            int sx = PIVOT_X + (int)( fx*cosA + fy*sinA + 0.5f);
            int sy = PIVOT_Y + (int)(-fx*sinA + fy*cosA + 0.5f);
            if (sx < 0 || sx >= PTR_WIDTH || sy < 0 || sy >= PTR_HEIGHT) continue;
            uint32_t sp = ptr[sy * PTR_WIDTH + sx];
            uint8_t sa = (sp >> 24) & 0xFF;
            if (sa == 0) continue;
            if (sa == 255) { fb[dy*W+dx] = sp; continue; }
            uint32_t dp = fb[dy*W+dx];
            uint8_t ia = 255 - sa;
            uint8_t or_ = ((sp>>16&0xFF)*sa + (dp>>16&0xFF)*ia) / 255;
            uint8_t og  = ((sp>>8&0xFF)*sa  + (dp>>8&0xFF)*ia)  / 255;
            uint8_t ob  = ((sp&0xFF)*sa     + (dp&0xFF)*ia)     / 255;
            fb[dy*W+dx] = 0xFF000000 | (or_<<16) | (og<<8) | ob;
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
    int port = 5555;
    if (argc > 1) port = atoi(argv[1]);

    /* ---- UDP socket (non-blocking) ---- */
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
    printf("=== VSI Receiver on port %d ===\n", port);

    /* ---- Load assets ---- */
    uint16_t *bg565 = load_bin("../Vertical speed/vsi_bg.bin", W*H*2);
    if (!bg565) return 1;
    uint16_t *ptr4444 = load_bin("../Vertical speed/vsi_pointer.bin", PTR_WIDTH*PTR_HEIGHT*2);
    if (!ptr4444) return 1;

    uint32_t *bg_clean = malloc(W*H*4);
    uint32_t *fb = malloc(W*H*4);
    uint32_t *ptr8888 = malloc(PTR_WIDTH*PTR_HEIGHT*4);
    if (!bg_clean || !fb || !ptr8888) return 1;

    rgb565_to_argb8888(bg565, bg_clean, W*H); free(bg565);
    argb4444_to_argb8888(ptr4444, ptr8888, PTR_WIDTH*PTR_HEIGHT); free(ptr4444);

    /* ---- SDL ---- */
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window *win = SDL_CreateWindow("VSI",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, W, H, 0);
    SDL_Renderer *ren = SDL_CreateRenderer(win, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    SDL_RenderSetLogicalSize(ren, W, H);
    SDL_Texture *tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING, W, H);

    /* ---- State ---- */
    float vsi_fpm = 0.0f;
    int got_data = 0;
    float demo_dir = 1.0f;
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
        SimPacket pkt;
        ssize_t n;
        while ((n = recvfrom(sock, &pkt, sizeof(pkt), 0, NULL, NULL)) > 0) {
            if (n >= (ssize_t)sizeof(pkt) && pkt.instrument_id == INSTRUMENT_VSI) {
                vsi_fpm = pkt.value;
                got_data = 1;
            }
        }

        /* Demo animation if no UDP data yet */
        if (!got_data) {
            vsi_fpm += dt * 300.0f * demo_dir;
            if (vsi_fpm >= 2000) { vsi_fpm = 2000; demo_dir = -1; }
            if (vsi_fpm <= -1500) { vsi_fpm = -1500; demo_dir = 1; }
        }

        /* ---- Render ---- */
        memcpy(fb, bg_clean, W*H*4);
        float angle_deg = vsi_fpm_to_deg(vsi_fpm);
        float angle_rad = angle_deg * (float)M_PI / 180.0f;
        rotate_pointer(fb, ptr8888, angle_rad);

        SDL_UpdateTexture(tex, NULL, fb, W*4);
        SDL_RenderCopy(ren, tex, NULL, NULL);
        SDL_RenderPresent(ren);
    }

    close(sock);
    free(bg_clean); free(fb); free(ptr8888);
    SDL_DestroyTexture(tex); SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win); SDL_Quit();
    return 0;
}
