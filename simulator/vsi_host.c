/*
 * GLASSALT VSI Host Simulator
 * Runs on the PC — displays the VSI instrument AND sends UDP packets
 * to the Raspberry Pi at 30 updates/sec.
 *
 * Click buttons or use keyboard:
 *   Model buttons = select flight profile
 *   GO/STOP button = start/stop sending
 *   Escape = Quit
 */
#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <winsock2.h>

#pragma comment(lib, "ws2_32.lib")

#define IMG_WIDTH   480
#define IMG_HEIGHT  480
#define PANEL_H     100   /* control panel height below instrument */
#define WIN_WIDTH   IMG_WIDTH
#define WIN_HEIGHT  (IMG_HEIGHT + PANEL_H)

/* Pointer source image */
#define PTR_WIDTH   363
#define PTR_HEIGHT  60
#define PIVOT_X     202
#define PIVOT_Y     30

#define CX  240
#define CY  240

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* UDP packet sent to the instrument */
typedef struct {
    uint32_t instrument_id;   /* 1=altimeter, 2=VSI */
    float    value;           /* vertical speed in fpm */
} SimPacket;

#define INSTRUMENT_VSI  2
#define PI_PORT         5555

/* ================================================================
 * Tiny 5x7 bitmap font — just enough characters for our buttons
 * Each glyph is 5 columns x 7 rows, packed as 7 bytes (1 bit per col)
 * ================================================================ */
static const uint8_t font5x7[][7] = {
    /* ' ' (space) */  {0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    /* 0 */  {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E},
    /* 1 */  {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E},
    /* 2 */  {0x0E,0x11,0x01,0x06,0x08,0x10,0x1F},
    /* 3 */  {0x0E,0x11,0x01,0x06,0x01,0x11,0x0E},
    /* 4 */  {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02},
    /* 5 */  {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E},
    /* 6 */  {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E},
    /* 7 */  {0x1F,0x01,0x02,0x04,0x08,0x08,0x08},
    /* 8 */  {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E},
    /* 9 */  {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C},
    /* A */  {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11},
    /* B */  {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E},
    /* C */  {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E},
    /* D */  {0x1E,0x11,0x11,0x11,0x11,0x11,0x1E},
    /* E */  {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F},
    /* F */  {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10},
    /* G */  {0x0E,0x11,0x10,0x17,0x11,0x11,0x0F},
    /* H */  {0x11,0x11,0x11,0x1F,0x11,0x11,0x11},
    /* I */  {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E},
    /* K */  {0x11,0x12,0x14,0x18,0x14,0x12,0x11},
    /* L */  {0x10,0x10,0x10,0x10,0x10,0x10,0x1F},
    /* M */  {0x11,0x1B,0x15,0x15,0x11,0x11,0x11},
    /* N */  {0x11,0x19,0x15,0x13,0x11,0x11,0x11},
    /* O */  {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E},
    /* P */  {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10},
    /* R */  {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11},
    /* S */  {0x0E,0x11,0x10,0x0E,0x01,0x11,0x0E},
    /* T */  {0x1F,0x04,0x04,0x04,0x04,0x04,0x04},
    /* U */  {0x11,0x11,0x11,0x11,0x11,0x11,0x0E},
    /* V */  {0x11,0x11,0x11,0x11,0x0A,0x0A,0x04},
    /* W */  {0x11,0x11,0x11,0x15,0x15,0x1B,0x11},
    /* + */  {0x00,0x04,0x04,0x1F,0x04,0x04,0x00},
    /* - */  {0x00,0x00,0x00,0x1F,0x00,0x00,0x00},
    /* : */  {0x00,0x04,0x04,0x00,0x04,0x04,0x00},
    /* / */  {0x01,0x02,0x02,0x04,0x08,0x08,0x10},
};

/* Character index lookup */
static int font_index(char c)
{
    if (c == ' ') return 0;
    if (c >= '0' && c <= '9') return 1 + (c - '0');
    if (c >= 'A' && c <= 'I') return 11 + (c - 'A');  /* A=11..I=19 */
    if (c == 'K') return 20;
    if (c == 'L') return 21;
    if (c == 'M') return 22;
    if (c == 'N') return 23;
    if (c == 'O') return 24;
    if (c == 'P') return 25;
    if (c == 'R') return 26;
    if (c == 'S') return 27;
    if (c == 'T') return 28;
    if (c == 'U') return 29;
    if (c == 'V') return 30;
    if (c == 'W') return 31;
    if (c == '+') return 32;
    if (c == '-') return 33;
    if (c == ':') return 34;
    if (c == '/') return 35;
    /* lowercase → uppercase */
    if (c >= 'a' && c <= 'z') return font_index(c - 32);
    return 0;  /* default to space */
}

/* Draw a string at (x,y) with given scale and color */
static void draw_text(uint32_t *fb, int stride, int x, int y,
                      const char *str, int scale, uint32_t color)
{
    for (int ci = 0; str[ci]; ci++) {
        int gi = font_index(str[ci]);
        for (int row = 0; row < 7; row++) {
            uint8_t bits = font5x7[gi][row];
            for (int col = 0; col < 5; col++) {
                if (bits & (0x10 >> col)) {
                    /* Draw a scale x scale block */
                    for (int sy = 0; sy < scale; sy++) {
                        for (int sx = 0; sx < scale; sx++) {
                            int px = x + ci * 6 * scale + col * scale + sx;
                            int py = y + row * scale + sy;
                            if (px >= 0 && px < stride && py >= 0 && py < WIN_HEIGHT)
                                fb[py * stride + px] = color;
                        }
                    }
                }
            }
        }
    }
}

/* Measure string width in pixels */
static int text_width(const char *str, int scale)
{
    int len = (int)strlen(str);
    return len * 6 * scale - scale;  /* 5px char + 1px gap, minus trailing gap */
}

/* --- Non-linear VSI scale --- */
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

/* --- Flight models --- */
typedef enum {
    MODEL_TAKEOFF = 0,
    MODEL_CRUISE,
    MODEL_APPROACH,
    MODEL_TURBULENCE,
    MODEL_COUNT
} FlightModel;

static const char *model_labels[] = {
    "TAKEOFF", "CRUISE", "APPROACH", "TURB"
};

static float model_compute(FlightModel model, float t)
{
    switch (model) {
    case MODEL_TAKEOFF:
        t = fmodf(t, 12.0f);
        if (t < 3.0f) return (t / 3.0f) * 1500.0f;
        if (t < 8.0f) return 1500.0f;
        return 1500.0f - (t - 8.0f) / 4.0f * 1000.0f;
    case MODEL_CRUISE:
        return 100.0f * sinf(t * 0.5f) + 30.0f * sinf(t * 1.3f);
    case MODEL_APPROACH:
        if (t < 2.0f) return -(t / 2.0f) * 700.0f;
        return -700.0f + 50.0f * sinf(t * 0.8f);
    case MODEL_TURBULENCE: {
        float v = 800.0f * sinf(t * 2.1f) + 400.0f * sinf(t * 5.7f)
                + 200.0f * sinf(t * 11.3f) + 100.0f * sinf(t * 23.7f);
        if (v >  2000.0f) v =  2000.0f;
        if (v < -1500.0f) v = -1500.0f;
        return v;
    }
    default: return 0.0f;
    }
}

/* --- Image helpers --- */
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
    float cosA = cosf(angle_rad);
    float sinA = sinf(angle_rad);
    int corners_sx[4] = {0, PTR_WIDTH-1, 0, PTR_WIDTH-1};
    int corners_sy[4] = {0, 0, PTR_HEIGHT-1, PTR_HEIGHT-1};
    int nx0 = IMG_WIDTH, ny0 = IMG_HEIGHT, nx1 = 0, ny1 = 0;
    for (int c = 0; c < 4; c++) {
        float fx = (float)(corners_sx[c] - PIVOT_X);
        float fy = (float)(corners_sy[c] - PIVOT_Y);
        int dx = CX + (int)(fx * cosA - fy * sinA);
        int dy = CY + (int)(fx * sinA + fy * cosA);
        if (dx < nx0) nx0 = dx; if (dx > nx1) nx1 = dx;
        if (dy < ny0) ny0 = dy; if (dy > ny1) ny1 = dy;
    }
    if (nx0 < 0) nx0 = 0; if (ny0 < 0) ny0 = 0;
    if (nx1 >= IMG_WIDTH) nx1 = IMG_WIDTH - 1;
    if (ny1 >= IMG_HEIGHT) ny1 = IMG_HEIGHT - 1;

    for (int dy = ny0; dy <= ny1; dy++) {
        for (int dx = nx0; dx <= nx1; dx++) {
            float fx = (float)(dx - CX);
            float fy = (float)(dy - CY);
            int sx = PIVOT_X + (int)(fx * cosA + fy * sinA + 0.5f);
            int sy = PIVOT_Y + (int)(-fx * sinA + fy * cosA + 0.5f);
            if (sx < 0 || sx >= PTR_WIDTH || sy < 0 || sy >= PTR_HEIGHT) continue;
            uint32_t sp = ptr[sy * PTR_WIDTH + sx];
            uint8_t sa = (sp >> 24) & 0xFF;
            if (sa == 0) continue;
            if (sa == 255) {
                fb[dy * IMG_WIDTH + dx] = sp;
            } else {
                uint32_t dp = fb[dy * IMG_WIDTH + dx];
                uint8_t ia = 255 - sa;
                uint8_t or_ = ((sp>>16&0xFF)*sa + (dp>>16&0xFF)*ia) / 255;
                uint8_t og  = ((sp>>8&0xFF)*sa  + (dp>>8&0xFF)*ia)  / 255;
                uint8_t ob  = ((sp&0xFF)*sa     + (dp&0xFF)*ia)     / 255;
                fb[dy * IMG_WIDTH + dx] = 0xFF000000 | (or_<<16) | (og<<8) | ob;
            }
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
        fprintf(stderr, "%s: expected %zu got %zu\n", path, expected, got);
        free(buf); return NULL;
    }
    return buf;
}

static void fill_rect(uint32_t *fb, int stride,
                      int x, int y, int w, int h, uint32_t color)
{
    for (int row = y; row < y + h && row < WIN_HEIGHT; row++)
        for (int col = x; col < x + w && col < stride; col++)
            if (row >= 0 && col >= 0)
                fb[row * stride + col] = color;
}

/* Button hit test */
typedef struct { int x, y, w, h; } Rect;

static int rect_contains(Rect r, int mx, int my)
{
    return mx >= r.x && mx < r.x + r.w && my >= r.y && my < r.y + r.h;
}

int main(int argc, char *argv[])
{
    (void)argc; (void)argv;

    const char *pi_ip = "192.168.1.129";
    if (argc > 1) pi_ip = argv[1];

    /* Init Winsock */
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        fprintf(stderr, "WSAStartup failed\n");
        return 1;
    }
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
        fprintf(stderr, "socket() failed\n");
        return 1;
    }
    struct sockaddr_in pi_addr;
    memset(&pi_addr, 0, sizeof(pi_addr));
    pi_addr.sin_family = AF_INET;
    pi_addr.sin_port = htons(PI_PORT);
    pi_addr.sin_addr.s_addr = inet_addr(pi_ip);

    printf("=== GLASSALT VSI Host ===\n");
    printf("Sending to %s:%d\n", pi_ip, PI_PORT);

    /* Load assets */
    uint16_t *bg565 = load_bin("../Vertical speed/vsi_bg.bin",
                               IMG_WIDTH * IMG_HEIGHT * 2);
    if (!bg565) return 1;
    uint16_t *ptr4444 = load_bin("../Vertical speed/vsi_pointer.bin",
                                 PTR_WIDTH * PTR_HEIGHT * 2);
    if (!ptr4444) return 1;

    uint32_t *bg_clean = malloc(IMG_WIDTH * IMG_HEIGHT * 4);
    uint32_t *fb = malloc(WIN_WIDTH * WIN_HEIGHT * 4);
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
        "GLASSALT VSI Host",
        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
        WIN_WIDTH, WIN_HEIGHT, SDL_WINDOW_SHOWN);
    if (!win) return 1;
    SDL_Renderer *ren = SDL_CreateRenderer(win, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!ren) return 1;
    SDL_Texture *tex = SDL_CreateTexture(ren,
        SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
        WIN_WIDTH, WIN_HEIGHT);
    if (!tex) return 1;

    /* Button layout */
    int btn_w = 88, btn_h = 30;
    int btn_y = IMG_HEIGHT + 8;
    int gap = (WIN_WIDTH - 4 * btn_w - 70) / 5;  /* leave room for GO button */
    Rect model_btns[MODEL_COUNT];
    for (int i = 0; i < MODEL_COUNT; i++) {
        model_btns[i].x = gap + i * (btn_w + gap);
        model_btns[i].y = btn_y;
        model_btns[i].w = btn_w;
        model_btns[i].h = btn_h;
    }
    Rect go_btn = { WIN_WIDTH - 70 - gap, btn_y, 70, btn_h };

    /* State */
    FlightModel model = MODEL_TAKEOFF;
    int sending = 0;
    float model_time = 0.0f;
    float vsi_fpm = 0.0f;
    Uint32 last_tick = SDL_GetTicks();
    Uint32 last_send = 0;
    int packets_sent = 0;

    /* Colors */
    uint32_t col_panel    = 0xFF1A1A2E;
    uint32_t col_btn      = 0xFF16213E;
    uint32_t col_btn_sel  = 0xFF0F3460;
    uint32_t col_btn_edge = 0xFF333355;
    uint32_t col_sel_edge = 0xFF4488FF;
    uint32_t col_go_on    = 0xFF006622;
    uint32_t col_go_off   = 0xFF662222;
    uint32_t col_go_edge  = 0xFF00CC44;
    uint32_t col_stop_edge= 0xFFCC4444;
    uint32_t col_white    = 0xFFE0E0E0;
    uint32_t col_dim      = 0xFF888899;

    int running = 1;
    while (running) {
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT) running = 0;

            /* Mouse clicks */
            if (ev.type == SDL_MOUSEBUTTONDOWN && ev.button.button == SDL_BUTTON_LEFT) {
                int mx = ev.button.x, my = ev.button.y;
                for (int i = 0; i < MODEL_COUNT; i++) {
                    if (rect_contains(model_btns[i], mx, my)) {
                        model = (FlightModel)i;
                        model_time = 0;
                    }
                }
                if (rect_contains(go_btn, mx, my)) {
                    sending = !sending;
                    model_time = 0;
                    packets_sent = sending ? 0 : packets_sent;
                }
            }

            /* Keyboard still works too */
            if (ev.type == SDL_KEYDOWN) {
                switch (ev.key.keysym.sym) {
                case SDLK_ESCAPE: running = 0; break;
                case SDLK_1: model = MODEL_TAKEOFF;   model_time = 0; break;
                case SDLK_2: model = MODEL_CRUISE;    model_time = 0; break;
                case SDLK_3: model = MODEL_APPROACH;   model_time = 0; break;
                case SDLK_4: model = MODEL_TURBULENCE; model_time = 0; break;
                case SDLK_SPACE:
                    sending = !sending;
                    model_time = 0;
                    packets_sent = sending ? 0 : packets_sent;
                    break;
                default: break;
                }
            }
        }

        Uint32 now = SDL_GetTicks();
        float dt = (now - last_tick) / 1000.0f;
        last_tick = now;

        if (sending) {
            model_time += dt;
            vsi_fpm = model_compute(model, model_time);
            if (now - last_send >= 33) {
                SimPacket pkt;
                pkt.instrument_id = INSTRUMENT_VSI;
                pkt.value = vsi_fpm;
                sendto(sock, (const char *)&pkt, sizeof(pkt), 0,
                       (struct sockaddr *)&pi_addr, sizeof(pi_addr));
                last_send = now;
                packets_sent++;
            }
        } else {
            vsi_fpm *= 0.95f;
            if (fabsf(vsi_fpm) < 1.0f) vsi_fpm = 0.0f;
        }

        /* --- Render instrument --- */
        memcpy(fb, bg_clean, IMG_WIDTH * IMG_HEIGHT * 4);
        float angle_deg = vsi_fpm_to_deg(vsi_fpm);
        float angle_rad = angle_deg * (float)M_PI / 180.0f;
        rotate_pointer(fb, ptr8888, angle_rad);

        /* --- Render control panel --- */
        fill_rect(fb, WIN_WIDTH, 0, IMG_HEIGHT, WIN_WIDTH, PANEL_H, col_panel);

        /* Thin separator line */
        fill_rect(fb, WIN_WIDTH, 0, IMG_HEIGHT, WIN_WIDTH, 1, 0xFF333355);

        /* Model buttons */
        for (int i = 0; i < MODEL_COUNT; i++) {
            Rect r = model_btns[i];
            int sel = (i == (int)model);
            fill_rect(fb, WIN_WIDTH, r.x, r.y, r.w, r.h,
                      sel ? col_btn_sel : col_btn);
            fill_rect(fb, WIN_WIDTH, r.x, r.y, r.w, 2,
                      sel ? col_sel_edge : col_btn_edge);
            /* Label centered in button */
            int tw = text_width(model_labels[i], 1);
            int tx = r.x + (r.w - tw) / 2;
            int ty = r.y + (r.h - 7) / 2;
            draw_text(fb, WIN_WIDTH, tx, ty, model_labels[i], 1,
                      sel ? col_white : col_dim);
        }

        /* GO / STOP button */
        {
            Rect r = go_btn;
            const char *label = sending ? "STOP" : "GO";
            fill_rect(fb, WIN_WIDTH, r.x, r.y, r.w, r.h,
                      sending ? col_go_off : col_go_on);
            fill_rect(fb, WIN_WIDTH, r.x, r.y, r.w, 2,
                      sending ? col_stop_edge : col_go_edge);
            int tw = text_width(label, 2);
            int tx = r.x + (r.w - tw) / 2;
            int ty = r.y + (r.h - 14) / 2;
            draw_text(fb, WIN_WIDTH, tx, ty, label, 2, col_white);
        }

        /* Status bar: bar graph + FPM readout */
        int status_y = IMG_HEIGHT + 48;

        /* Sending dot */
        uint32_t dot_col = sending ? 0xFF00CC44 : 0xFF663333;
        for (int dy2 = 0; dy2 < 12; dy2++)
            for (int dx2 = 0; dx2 < 12; dx2++) {
                int cx2 = dx2 - 6, cy2 = dy2 - 6;
                if (cx2*cx2 + cy2*cy2 <= 25)
                    fb[(status_y + dy2 + 16) * WIN_WIDTH + (10 + dx2)] = dot_col;
            }

        /* Bar graph */
        int bar_x = 30, bar_w = 350, bar_h = 14;
        int bar_y2 = status_y + 16;
        fill_rect(fb, WIN_WIDTH, bar_x, bar_y2, bar_w, bar_h, 0xFF222222);
        fill_rect(fb, WIN_WIDTH, bar_x + bar_w/2, bar_y2, 2, bar_h, 0xFF666666);
        float bar_frac = vsi_fpm / 2000.0f;
        if (bar_frac > 1.0f) bar_frac = 1.0f;
        if (bar_frac < -1.0f) bar_frac = -1.0f;
        int bar_pixels = (int)(bar_frac * (bar_w / 2));
        uint32_t bar_col = (vsi_fpm >= 0) ? 0xFF00CC66 : 0xFFCC6600;
        if (bar_pixels > 0)
            fill_rect(fb, WIN_WIDTH, bar_x + bar_w/2, bar_y2 + 2,
                      bar_pixels, bar_h - 4, bar_col);
        else if (bar_pixels < 0)
            fill_rect(fb, WIN_WIDTH, bar_x + bar_w/2 + bar_pixels, bar_y2 + 2,
                      -bar_pixels, bar_h - 4, bar_col);

        /* FPM number readout */
        char fpm_str[16];
        int fpm_int = (int)vsi_fpm;
        if (fpm_int >= 0)
            snprintf(fpm_str, sizeof(fpm_str), "+%d", fpm_int);
        else
            snprintf(fpm_str, sizeof(fpm_str), "%d", fpm_int);
        draw_text(fb, WIN_WIDTH, 390, bar_y2 + 2, fpm_str, 1,
                  (fpm_int >= 0) ? 0xFF00CC66 : 0xFFCC6600);

        /* Packet count */
        char pkt_str[32];
        snprintf(pkt_str, sizeof(pkt_str), "%d", packets_sent);
        draw_text(fb, WIN_WIDTH, 390, bar_y2 + 14, pkt_str, 1, col_dim);

        SDL_UpdateTexture(tex, NULL, fb, WIN_WIDTH * 4);
        SDL_RenderClear(ren);
        SDL_RenderCopy(ren, tex, NULL, NULL);
        SDL_RenderPresent(ren);
    }

    closesocket(sock);
    WSACleanup();
    free(bg_clean); free(fb); free(ptr8888);
    SDL_DestroyTexture(tex);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
