/*
 * VSI Calibration Tool
 * Sends fixed FPM values to the Pi, waits for Enter between each.
 * Also displays the pointer locally so you can compare both screens.
 */
#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <winsock2.h>

#define IMG_WIDTH   480
#define IMG_HEIGHT  480
#define WIN_WIDTH   IMG_WIDTH
#define WIN_HEIGHT  IMG_HEIGHT

#define PTR_WIDTH   363
#define PTR_HEIGHT  60
#define PIVOT_X     202
#define PIVOT_Y     30
#define CX  240
#define CY  240

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    uint32_t instrument_id;
    float    value;
} SimPacket;

#define INSTRUMENT_VSI  2
#define PI_PORT         5555

/* Current scale table (to be corrected after calibration) */
typedef struct { float fpm; float deg; } ScalePoint;
static const ScalePoint vsi_scale[] = {
    { -2500, -151.0f }, { -2000, -127.0f }, { -1500, -90.0f },
    { -1000, -57.5f }, { -500, -25.0f }, { 0, 0.0f },
    { 500, 24.0f }, { 1000, 57.0f }, { 1500, 90.0f },
    { 2000, 129.0f }, { 2500, 153.0f }, { 3000, 174.0f },
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

/* Test values: -2000 to +2000 in 500 FPM steps */
static const float test_values[] = {
    -2000, -1500, -1000, -500, 0, 500, 1000, 1500, 2000
};
#define NUM_TESTS (sizeof(test_values) / sizeof(test_values[0]))

/* --- Asset loaders --- */
static void *load_bin(const char *path, size_t expected)
{
    FILE *f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "Cannot open %s\n", path); return NULL; }
    void *buf = malloc(expected);
    fread(buf, 1, expected, f);
    fclose(f);
    return buf;
}

static void rgb565_to_argb8888(const uint16_t *src, uint32_t *dst, int count)
{
    for (int i = 0; i < count; i++) {
        uint16_t p = src[i];
        uint8_t r = (p >> 11) & 0x1F; r = (r << 3) | (r >> 2);
        uint8_t g = (p >>  5) & 0x3F; g = (g << 2) | (g >> 4);
        uint8_t b =  p        & 0x1F; b = (b << 3) | (b >> 2);
        dst[i] = 0xFF000000 | (r << 16) | (g << 8) | b;
    }
}

static void argb4444_to_argb8888(const uint16_t *src, uint32_t *dst, int count)
{
    for (int i = 0; i < count; i++) {
        uint16_t p = src[i];
        uint8_t a = (p >> 12) & 0xF; a = (a << 4) | a;
        uint8_t r = (p >>  8) & 0xF; r = (r << 4) | r;
        uint8_t g = (p >>  4) & 0xF; g = (g << 4) | g;
        uint8_t b =  p        & 0xF; b = (b << 4) | b;
        dst[i] = (a << 24) | (r << 16) | (g << 8) | b;
    }
}

static void rotate_pointer(uint32_t *fb, const uint32_t *ptr, float angle_rad)
{
    float cosA = cosf(angle_rad);
    float sinA = sinf(angle_rad);
    int r = (PTR_WIDTH > PTR_HEIGHT ? PTR_WIDTH : PTR_HEIGHT);
    for (int dy = -r; dy <= r; dy++) {
        for (int dx = -r; dx <= r; dx++) {
            float sx = cosA * dx + sinA * dy + PIVOT_X;
            float sy = -sinA * dx + cosA * dy + PIVOT_Y;
            int isx = (int)sx, isy = (int)sy;
            if (isx < 0 || isx >= PTR_WIDTH || isy < 0 || isy >= PTR_HEIGHT) continue;
            uint32_t p = ptr[isy * PTR_WIDTH + isx];
            uint8_t a = (p >> 24) & 0xFF;
            if (a < 128) continue;
            int fx = CX + dx, fy = CY + dy;
            if (fx < 0 || fx >= IMG_WIDTH || fy < 0 || fy >= IMG_HEIGHT) continue;
            fb[fy * IMG_WIDTH + fx] = p | 0xFF000000;
        }
    }
}

/* Tiny 5x7 font - digits and minus only */
static const uint8_t font5x7[][5] = {
    ['0'] = {0x3E,0x51,0x49,0x45,0x3E},
    ['1'] = {0x00,0x42,0x7F,0x40,0x00},
    ['2'] = {0x42,0x61,0x51,0x49,0x46},
    ['3'] = {0x21,0x41,0x45,0x4B,0x31},
    ['4'] = {0x18,0x14,0x12,0x7F,0x10},
    ['5'] = {0x27,0x45,0x45,0x45,0x39},
    ['6'] = {0x3C,0x4A,0x49,0x49,0x30},
    ['7'] = {0x01,0x71,0x09,0x05,0x03},
    ['8'] = {0x36,0x49,0x49,0x49,0x36},
    ['9'] = {0x06,0x49,0x49,0x29,0x1E},
    ['-'] = {0x08,0x08,0x08,0x08,0x08},
    ['+'] = {0x08,0x08,0x3E,0x08,0x08},
    [' '] = {0x00,0x00,0x00,0x00,0x00},
    ['F'] = {0x7F,0x09,0x09,0x09,0x01},
    ['P'] = {0x7F,0x09,0x09,0x09,0x06},
    ['M'] = {0x7F,0x02,0x0C,0x02,0x7F},
    [':'] = {0x00,0x36,0x36,0x00,0x00},
    ['E'] = {0x7F,0x49,0x49,0x49,0x41},
    ['N'] = {0x7F,0x04,0x08,0x10,0x7F},
    ['T'] = {0x01,0x01,0x7F,0x01,0x01},
    ['R'] = {0x7F,0x09,0x19,0x29,0x46},
};

static void draw_char(uint32_t *fb, int stride, int x, int y, char c, int scale, uint32_t color)
{
    if ((unsigned char)c >= sizeof(font5x7)/sizeof(font5x7[0])) return;
    const uint8_t *glyph = font5x7[(unsigned char)c];
    for (int col = 0; col < 5; col++) {
        uint8_t bits = glyph[col];
        for (int row = 0; row < 7; row++) {
            if (bits & (1 << row)) {
                for (int sy = 0; sy < scale; sy++)
                    for (int sx = 0; sx < scale; sx++) {
                        int px = x + col * scale + sx;
                        int py = y + row * scale + sy;
                        if (px >= 0 && px < stride && py >= 0 && py < WIN_HEIGHT)
                            fb[py * stride + px] = color;
                    }
            }
        }
    }
}

static void draw_text(uint32_t *fb, int stride, int x, int y, const char *s, int scale, uint32_t color)
{
    for (; *s; s++, x += 6 * scale)
        draw_char(fb, stride, x, y, *s, scale, color);
}

int main(int argc, char *argv[])
{
    (void)argc; (void)argv;

    const char *pi_ip = "192.168.1.129";
    if (argc > 1) pi_ip = argv[1];

    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in pi_addr;
    memset(&pi_addr, 0, sizeof(pi_addr));
    pi_addr.sin_family = AF_INET;
    pi_addr.sin_port = htons(PI_PORT);
    pi_addr.sin_addr.s_addr = inet_addr(pi_ip);

    /* Load assets */
    uint16_t *bg565 = load_bin("../Vertical speed/vsi_bg.bin",
                               IMG_WIDTH * IMG_HEIGHT * 2);
    if (!bg565) return 1;
    uint16_t *ptr4444 = load_bin("../Vertical speed/vsi_pointer.bin",
                                 PTR_WIDTH * PTR_HEIGHT * 2);
    if (!ptr4444) return 1;

    uint32_t *bg_clean = malloc(IMG_WIDTH * IMG_HEIGHT * 4);
    uint32_t *fb = malloc(WIN_WIDTH * WIN_HEIGHT * 4);
    rgb565_to_argb8888(bg565, bg_clean, IMG_WIDTH * IMG_HEIGHT);
    free(bg565);

    uint32_t *ptr8888 = malloc(PTR_WIDTH * PTR_HEIGHT * 4);
    argb4444_to_argb8888(ptr4444, ptr8888, PTR_WIDTH * PTR_HEIGHT);
    free(ptr4444);

    /* Init SDL */
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window *win = SDL_CreateWindow("VSI Calibration",
        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
        WIN_WIDTH, WIN_HEIGHT, SDL_WINDOW_SHOWN);
    SDL_Renderer *ren = SDL_CreateRenderer(win, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    SDL_Texture *tex = SDL_CreateTexture(ren,
        SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
        WIN_WIDTH, WIN_HEIGHT);

    printf("=== VSI Calibration Tool ===\n");
    printf("Sending to %s:%d\n", pi_ip, PI_PORT);
    printf("Press ENTER in this console to advance to next test value.\n");
    printf("Press ESC in the SDL window to quit.\n\n");

    for (int t = 0; t < (int)NUM_TESTS; t++) {
        float fpm = test_values[t];
        float angle_deg = vsi_fpm_to_deg(fpm);
        float angle_rad = angle_deg * (float)M_PI / 180.0f;

        printf("Test %d/%d: Sending %+.0f FPM  (current table says %.1f deg)\n",
               t + 1, (int)NUM_TESTS, fpm, angle_deg);
        printf("  >> Note where the pointer actually points, then press ENTER...\n");

        /* Send packets and render until user presses Enter */
        int waiting = 1;
        while (waiting) {
            /* Check SDL events */
            SDL_Event ev;
            while (SDL_PollEvent(&ev)) {
                if (ev.type == SDL_QUIT) goto done;
                if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_ESCAPE) goto done;
                if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym != SDLK_ESCAPE) waiting = 0;
            }

            /* Also check console stdin (non-blocking) */
            /* We'll just rely on SDL RETURN key for simplicity */

            /* Send UDP packet */
            SimPacket pkt;
            pkt.instrument_id = INSTRUMENT_VSI;
            pkt.value = fpm;
            sendto(sock, (const char *)&pkt, sizeof(pkt), 0,
                   (struct sockaddr *)&pi_addr, sizeof(pi_addr));

            /* Render */
            memcpy(fb, bg_clean, IMG_WIDTH * IMG_HEIGHT * 4);
            rotate_pointer(fb, ptr8888, angle_rad);

            /* Draw FPM label */
            char label[32];
            snprintf(label, sizeof(label), "%+.0f FPM", fpm);
            draw_text(fb, WIN_WIDTH, 10, 10, label, 3, 0xFF00FF00);

            char step[32];
            snprintf(label, sizeof(label), "%d:%d", t + 1, (int)NUM_TESTS);
            draw_text(fb, WIN_WIDTH, 10, 450, label, 2, 0xFFFFFF00);

            SDL_UpdateTexture(tex, NULL, fb, WIN_WIDTH * 4);
            SDL_RenderCopy(ren, tex, NULL, NULL);
            SDL_RenderPresent(ren);
            SDL_Delay(33);
        }

        printf("  >> Got it. Moving on...\n\n");
    }

    printf("=== Calibration complete! ===\n");
    printf("Report the pointer reading for each FPM value so we can fix the scale table.\n");

done:
    SDL_DestroyTexture(tex);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    closesocket(sock);
    WSACleanup();
    free(bg_clean); free(fb); free(ptr8888);
    return 0;
}
