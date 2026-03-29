/*
 * HSI (Horizontal Situation Indicator) Simulator
 * Desktop SDL2 renderer for 480x480 KI-525A HSI face
 * with clickable GUI control panel
 *
 * Layer stack (bottom to top):
 *   1. Background      (L8, static)
 *   2. CDI Dots Disk   (AL44, rotates with course)
 *   3. Compass Card    (AL44, rotates with heading)
 *   4. Heading Bug     (ARGB1555 sprite, rotates with heading bug)
 *   5. Course Pointer  (ARGB1555 sprite, rotates with course)
 *   6. CDI Bar         (ARGB1555 sprite, rotates with course + translates)
 *   7. Lubber/Aircraft (ARGB1555 sprite, static)
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
#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

/* ---- Display geometry ---- */
#define DISP_W    480       /* instrument face width/height */
#define DISP_H    480
#define PANEL_H   110       /* control panel height */
#define WIN_W     DISP_W
#define WIN_H     (DISP_H + PANEL_H)   /* 590 */
#define CX        (DISP_W / 2)
#define CY        (DISP_H / 2)

/* ---- Panel layout ---- */
#define N_CONTROLS  4
#define GROUP_W     (WIN_W / N_CONTROLS)    /* 120 px per group */
#define BTN_W       32
#define BTN_H       28
#define BTN_Y       (DISP_H + 58)           /* button row y */
#define LABEL_Y     (DISP_H + 14)           /* label row y */
#define VALUE_Y     (DISP_H + 38)           /* value row y */
/* Left button x within group */
#define BTN_L_X(g)  ((g) * GROUP_W + 6)
/* Right button x within group */
#define BTN_R_X(g)  ((g) * GROUP_W + GROUP_W - 6 - BTN_W)

/* ---- Panel colors ---- */
#define COL_PANEL_BG    0xFF1A1A1Au
#define COL_SEPARATOR   0xFF333333u
#define COL_BTN_NORMAL  0xFF3A3A3Au
#define COL_BTN_HOVER   0xFF505050u
#define COL_BTN_PRESS   0xFF686868u
#define COL_ARROW       0xFFDDDDDDu
#define COL_LABEL       0xFF888888u
#define COL_VALUE       0xFF44DD44u     /* green avionics look */

/* ================================================================== */
/*  5x7 bitmap font                                                   */
/* ================================================================== */
#define FONT_W  5
#define FONT_H  7
#define FONT_SCALE  2
#define CHAR_W  ((FONT_W + 1) * FONT_SCALE)    /* 12 px per char */
#define CHAR_H  (FONT_H * FONT_SCALE)          /* 14 px */

/* Each glyph: 7 rows, bits 4..0 = pixels left to right */
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
                        int px = x + col * FONT_SCALE + sx;
                        int py = y + row * FONT_SCALE + sy;
                        if ((unsigned)px < WIN_W && (unsigned)py < WIN_H)
                            fb[py * WIN_W + px] = color;
                    }
}

static void draw_str(uint32_t *fb, int x, int y, const char *s, uint32_t color)
{
    while (*s) { draw_char(fb, x, y, *s++, color); x += CHAR_W; }
}

/* Draw string centered horizontally in a range */
static void draw_str_cx(uint32_t *fb, int center_x, int y,
                         const char *s, uint32_t color)
{
    int len = (int)strlen(s);
    int total_w = len * CHAR_W - (FONT_SCALE); /* subtract trailing gap */
    draw_str(fb, center_x - total_w / 2, y, s, color);
}

/* ================================================================== */
/*  GUI button drawing                                                */
/* ================================================================== */

/* Fill a rect */
static void fill_rect(uint32_t *fb, int rx, int ry, int rw, int rh, uint32_t c)
{
    for (int y = ry; y < ry + rh; y++) {
        if ((unsigned)y >= WIN_H) continue;
        for (int x = rx; x < rx + rw; x++)
            if ((unsigned)x < WIN_W)
                fb[y * WIN_W + x] = c;
    }
}

/* Draw a filled triangle arrow inside a button rect */
static void draw_arrow(uint32_t *fb, int bx, int by, int bw, int bh,
                        int dir, uint32_t color)
{
    /* dir: -1 = left-pointing (◄), +1 = right-pointing (►) */
    int pad = 9;
    int th = bh - 2 * pad;     /* triangle height */
    int tw = bw - 2 * pad;     /* triangle base width */
    if (th < 2 || tw < 2) return;
    int half = th / 2;

    for (int row = 0; row < th; row++) {
        int dist = abs(row - half);
        int w = tw * (half - dist) / half;
        if (w < 1) w = 1;
        int y = by + pad + row;
        int x0;
        if (dir < 0)        /* ◄  base on right, tip on left */
            x0 = bx + bw - pad - w;
        else                /* ►  base on left, tip on right */
            x0 = bx + pad;
        for (int c = 0; c < w; c++)
            if ((unsigned)(x0 + c) < WIN_W && (unsigned)y < WIN_H)
                fb[y * WIN_W + x0 + c] = color;
    }
}

/* Draw a single button (rect + arrow) and return its hit state */
static void draw_button(uint32_t *fb, int bx, int by, int dir,
                         int hovered, int pressed)
{
    uint32_t bg = pressed ? COL_BTN_PRESS : hovered ? COL_BTN_HOVER : COL_BTN_NORMAL;
    fill_rect(fb, bx, by, BTN_W, BTN_H, bg);
    draw_arrow(fb, bx, by, BTN_W, BTN_H, dir, COL_ARROW);
}

/* ================================================================== */
/*  ARGB1555 sprite                                                   */
/* ================================================================== */
typedef struct {
    int ox, oy, w, h;
    uint16_t *px;
} Sprite;

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

static int load_sprite(const char *path, Sprite *s)
{
    FILE *f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "Cannot open %s\n", path); return -1; }
    uint16_t hdr[4];
    if (fread(hdr, 2, 4, f) != 4) { fclose(f); return -1; }
    s->ox = hdr[0]; s->oy = hdr[1]; s->w = hdr[2]; s->h = hdr[3];
    size_t npx = (size_t)s->w * s->h;
    s->px = malloc(npx * 2);
    if (!s->px) { fclose(f); return -1; }
    if (fread(s->px, 2, npx, f) != npx) {
        fprintf(stderr, "%s: short pixel read\n", path);
        free(s->px); fclose(f); return -1;
    }
    fclose(f);
    printf("  %s: offset(%d,%d) size(%dx%d)\n", path, s->ox, s->oy, s->w, s->h);
    return 0;
}

/* ================================================================== */
/*  Instrument compositing                                            */
/* ================================================================== */

static inline void blend_pixel(uint32_t *dst,
                               uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    if (a == 0) return;
    if (a == 255) {
        *dst = 0xFF000000u | ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
        return;
    }
    uint32_t d = *dst;
    uint8_t dr = (d >> 16) & 0xFF;
    uint8_t dg = (d >>  8) & 0xFF;
    uint8_t db =  d        & 0xFF;
    dr = (uint8_t)((r * a + dr * (255 - a)) / 255);
    dg = (uint8_t)((g * a + dg * (255 - a)) / 255);
    db = (uint8_t)((b * a + db * (255 - a)) / 255);
    *dst = 0xFF000000u | ((uint32_t)dr << 16) | ((uint32_t)dg << 8) | db;
}

/* Layer 1 — Background (L8, 480x480, static) */
static void blit_background(uint32_t *fb, const uint8_t *bg)
{
    for (int i = 0; i < DISP_W * DISP_H; i++) {
        uint8_t l = bg[i];
        fb[i] = 0xFF000000u | ((uint32_t)l << 16) | ((uint32_t)l << 8) | l;
    }
}

/* Rotate + blend a full 480x480 AL44 layer around display center */
static void rotate_blend_al44(uint32_t *fb, const uint8_t *src, float deg)
{
    float rad = deg * (float)M_PI / 180.0f;
    float cs = cosf(rad), sn = sinf(rad);

    for (int dy = 0; dy < DISP_H; dy++) {
        float fy = (float)(dy - CY);
        for (int dx = 0; dx < DISP_W; dx++) {
            float fx = (float)(dx - CX);
            int sx = (int)(CX + fx * cs + fy * sn + 0.5f);
            int sy = (int)(CY - fx * sn + fy * cs + 0.5f);
            if ((unsigned)sx >= DISP_W || (unsigned)sy >= DISP_H) continue;

            uint8_t p = src[sy * DISP_W + sx];
            uint8_t a4 = p >> 4;
            if (a4 == 0) continue;
            uint8_t l = (p & 0x0F) * 17;
            uint8_t a = a4 * 17;
            blend_pixel(&fb[dy * WIN_W + dx], l, l, l, a);
        }
    }
}

/* Rotate + blend an ARGB1555 sprite around display center */
static void rotate_blend_sprite(uint32_t *fb, const Sprite *s,
                                float deg, float tx, float ty)
{
    float rad = deg * (float)M_PI / 180.0f;
    float cs = cosf(rad), sn = sinf(rad);

    float corners[4][2] = {
        { s->ox + tx,            s->oy + ty            },
        { s->ox + s->w + tx,     s->oy + ty            },
        { s->ox + tx,            s->oy + s->h + ty     },
        { s->ox + s->w + tx,     s->oy + s->h + ty     }
    };
    int min_x = DISP_W, min_y = DISP_H, max_x = 0, max_y = 0;
    for (int i = 0; i < 4; i++) {
        float fx = corners[i][0] - CX;
        float fy = corners[i][1] - CY;
        int rx = (int)(CX + fx * cs - fy * sn);
        int ry = (int)(CY + fx * sn + fy * cs);
        if (rx < min_x) min_x = rx;
        if (rx > max_x) max_x = rx;
        if (ry < min_y) min_y = ry;
        if (ry > max_y) max_y = ry;
    }
    if (min_x < 0) min_x = 0;        else if (min_x > 2) min_x -= 2;
    if (min_y < 0) min_y = 0;        else if (min_y > 2) min_y -= 2;
    if (max_x >= DISP_W) max_x = DISP_W - 1; else max_x += 2;
    if (max_y >= DISP_H) max_y = DISP_H - 1; else max_y += 2;
    if (max_x >= DISP_W) max_x = DISP_W - 1;
    if (max_y >= DISP_H) max_y = DISP_H - 1;

    for (int dy = min_y; dy <= max_y; dy++) {
        float fy = (float)(dy - CY);
        for (int dx = min_x; dx <= max_x; dx++) {
            float fx = (float)(dx - CX);
            float sx_f = CX + fx * cs + fy * sn - tx;
            float sy_f = CY - fx * sn + fy * cs - ty;
            int lx = (int)(sx_f - s->ox + 0.5f);
            int ly = (int)(sy_f - s->oy + 0.5f);
            if ((unsigned)lx >= (unsigned)s->w ||
                (unsigned)ly >= (unsigned)s->h) continue;

            uint16_t p = s->px[ly * s->w + lx];
            if (!(p & 0x8000)) continue;
            uint8_t r = ((p >> 10) & 0x1F) << 3;
            uint8_t g = ((p >>  5) & 0x1F) << 3;
            uint8_t b = ( p        & 0x1F) << 3;
            fb[dy * WIN_W + dx] = 0xFF000000u |
                ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
        }
    }
}

/* Blit ARGB1555 sprite with no rotation (static overlay) */
static void blit_sprite(uint32_t *fb, const Sprite *s)
{
    for (int y = 0; y < s->h; y++) {
        int dy = s->oy + y;
        if ((unsigned)dy >= DISP_H) continue;
        for (int x = 0; x < s->w; x++) {
            int dx = s->ox + x;
            if ((unsigned)dx >= DISP_W) continue;
            uint16_t p = s->px[y * s->w + x];
            if (!(p & 0x8000)) continue;
            uint8_t r = ((p >> 10) & 0x1F) << 3;
            uint8_t g = ((p >>  5) & 0x1F) << 3;
            uint8_t b = ( p        & 0x1F) << 3;
            fb[dy * WIN_W + dx] = 0xFF000000u |
                ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
        }
    }
}

/* ================================================================== */
/*  Button hit testing                                                */
/* ================================================================== */

/* Button IDs: 0=HDG_L 1=HDG_R 2=CRS_L 3=CRS_R
 *             4=DEV_L 5=DEV_R 6=BUG_L 7=BUG_R */
#define N_BUTTONS (N_CONTROLS * 2)

static void get_btn_rect(int id, int *bx, int *by, int *bw, int *bh)
{
    int group = id / 2;
    int is_right = id & 1;
    *bx = is_right ? BTN_R_X(group) : BTN_L_X(group);
    *by = BTN_Y;
    *bw = BTN_W;
    *bh = BTN_H;
}

static int hit_test(int mx, int my)
{
    for (int i = 0; i < N_BUTTONS; i++) {
        int bx, by, bw, bh;
        get_btn_rect(i, &bx, &by, &bw, &bh);
        if (mx >= bx && mx < bx + bw && my >= by && my < by + bh)
            return i;
    }
    return -1;
}

/* ================================================================== */
/*  Main                                                              */
/* ================================================================== */
int main(int argc, char *argv[])
{
    (void)argc; (void)argv;

    /* ---- Load assets ---- */
    printf("Loading HSI layers from ../HSI/\n");

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

    printf("All layers loaded OK\n");

    /* ---- Framebuffer (full window including panel) ---- */
    uint32_t *fb = calloc(WIN_W * WIN_H, sizeof(uint32_t));
    if (!fb) { fprintf(stderr, "framebuffer alloc failed\n"); return 1; }

    /* ---- SDL init ---- */
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 1;
    }
    SDL_Window *win = SDL_CreateWindow("HSI Simulator",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIN_W, WIN_H, 0);
    SDL_Renderer *ren = SDL_CreateRenderer(win, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    SDL_RenderSetLogicalSize(ren, WIN_W, WIN_H);  /* maintain aspect ratio */
    SDL_Texture *tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING, WIN_W, WIN_H);

    /* ---- Instrument state ---- */
    float heading     = 0.0f;       /* aircraft magnetic heading */
    float course      = 45.0f;      /* selected course / OBS */
    float hdg_bug_deg = 90.0f;      /* heading bug setting */
    float cdi_dev     = 0.0f;       /* CDI deviation in pixels, ±75 */
    int   auto_hdg    = 1;          /* demo: auto-rotate heading */
    int   auto_cdi    = 1;          /* demo: auto-oscillate CDI */

    /* ---- Mouse state ---- */
    int pressed_btn = -1;           /* which button is held, -1 = none */
    Uint32 press_start = 0;         /* tick when button was first pressed */
    int initial_step_done = 0;      /* did we apply the first click step? */

    int running = 1;
    Uint32 last_tick = SDL_GetTicks();

    /* Control labels and value format strings */
    static const char *labels[N_CONTROLS] = { "HDG", "CRS", "DEV", "BUG" };

    while (running) {
        Uint32 now = SDL_GetTicks();
        float dt = (now - last_tick) / 1000.0f;
        if (dt > 0.1f) dt = 0.1f;
        last_tick = now;

        /* ---- Mouse position for hover ---- */
        int mx, my;
        SDL_GetMouseState(&mx, &my);
        int hovered_btn = hit_test(mx, my);

        /* ---- Events ---- */
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            switch (ev.type) {
            case SDL_QUIT:
                running = 0;
                break;

            case SDL_KEYDOWN:
                switch (ev.key.keysym.sym) {
                case SDLK_ESCAPE: running = 0; break;
                case SDLK_SPACE:  auto_hdg = !auto_hdg; break;
                case SDLK_TAB:    auto_cdi = !auto_cdi; break;
                case SDLK_r:
                    heading = 0; course = 45; hdg_bug_deg = 90;
                    cdi_dev = 0; auto_hdg = 1; auto_cdi = 1;
                    break;
                default: break;
                }
                break;

            case SDL_MOUSEBUTTONDOWN:
                if (ev.button.button == SDL_BUTTON_LEFT) {
                    pressed_btn = hit_test(ev.button.x, ev.button.y);
                    press_start = now;
                    initial_step_done = 0;
                }
                break;

            case SDL_MOUSEBUTTONUP:
                if (ev.button.button == SDL_BUTTON_LEFT)
                    pressed_btn = -1;
                break;
            }
        }

        /* ---- Apply button actions ---- */
        /* Single step on first click, then continuous after 400ms hold */
        if (pressed_btn >= 0) {
            float step_deg = 1.0f;          /* 1° per click */
            float step_dev = 5.0f;          /* 5px per click (0.2 dots) */
            float cont_deg = 45.0f * dt;    /* 45°/s continuous */
            float cont_dev = 75.0f * dt;    /* 75px/s continuous */

            int dir = (pressed_btn & 1) ? 1 : -1;  /* even=left/minus, odd=right/plus */
            int group = pressed_btn / 2;
            float *target = NULL;
            float step = 0, cont = 0;

            switch (group) {
            case 0: target = &heading;     step = step_deg; cont = cont_deg; auto_hdg = 0; break;
            case 1: target = &course;      step = step_deg; cont = cont_deg; break;
            case 2: target = &cdi_dev;     step = step_dev; cont = cont_dev; auto_cdi = 0; break;
            case 3: target = &hdg_bug_deg; step = step_deg; cont = cont_deg; break;
            }

            if (target) {
                if (!initial_step_done) {
                    *target += dir * step;
                    initial_step_done = 1;
                } else if (now - press_start > 400) {
                    *target += dir * cont;
                }
            }
        }

        /* ---- Continuous key input ---- */
        const Uint8 *keys = SDL_GetKeyboardState(NULL);
        float rate = 30.0f;
        float dev_rate = 50.0f;

        if (keys[SDL_SCANCODE_LEFT])  { heading -= rate * dt; auto_hdg = 0; }
        if (keys[SDL_SCANCODE_RIGHT]) { heading += rate * dt; auto_hdg = 0; }
        if (keys[SDL_SCANCODE_UP])    course  -= rate * dt;
        if (keys[SDL_SCANCODE_DOWN])  course  += rate * dt;
        if (keys[SDL_SCANCODE_Q])     hdg_bug_deg -= rate * dt;
        if (keys[SDL_SCANCODE_W])     hdg_bug_deg += rate * dt;
        if (keys[SDL_SCANCODE_A])     { cdi_dev -= dev_rate * dt; auto_cdi = 0; }
        if (keys[SDL_SCANCODE_D])     { cdi_dev += dev_rate * dt; auto_cdi = 0; }

        /* ---- Demo animations ---- */
        if (auto_hdg) heading += 6.0f * dt;
        if (auto_cdi) {
            float t = fmodf(now / 1000.0f, 8.0f);
            cdi_dev = (t < 4.0f) ? -50.0f + 25.0f * t : 150.0f - 25.0f * t;
        }

        /* ---- Wrap angles 0-360, clamp CDI ---- */
        heading     = fmodf(heading, 360.0f);
        course      = fmodf(course, 360.0f);
        hdg_bug_deg = fmodf(hdg_bug_deg, 360.0f);
        if (heading < 0)     heading     += 360.0f;
        if (course < 0)      course      += 360.0f;
        if (hdg_bug_deg < 0) hdg_bug_deg += 360.0f;
        if (cdi_dev >  75.0f) cdi_dev =  75.0f;
        if (cdi_dev < -75.0f) cdi_dev = -75.0f;

        /* ---- Rotation angles (positive = CW on screen) ---- */
        float card_rot = -heading;
        float crs_rot  = course - heading;
        float bug_rot  = hdg_bug_deg - heading;

        /* ============================================================ */
        /*  RENDER INSTRUMENT (top 480x480)                             */
        /* ============================================================ */

        blit_background(fb, bg_l8);
        rotate_blend_al44(fb, cdi_dots_al44, crs_rot);
        rotate_blend_al44(fb, compass_al44, card_rot);
        rotate_blend_sprite(fb, &hdg_bug, bug_rot, 0, 0);
        rotate_blend_sprite(fb, &course_ptr, crs_rot, 0, 0);
        rotate_blend_sprite(fb, &cdi_bar, crs_rot, cdi_dev, 0);
        blit_sprite(fb, &lubber);

        /* ============================================================ */
        /*  RENDER CONTROL PANEL (bottom 110px)                         */
        /* ============================================================ */

        /* Panel background */
        fill_rect(fb, 0, DISP_H, WIN_W, PANEL_H, COL_PANEL_BG);

        /* Separator line */
        fill_rect(fb, 0, DISP_H, WIN_W, 1, COL_SEPARATOR);

        /* Vertical separators between groups */
        for (int g = 1; g < N_CONTROLS; g++)
            fill_rect(fb, g * GROUP_W, DISP_H + 4, 1, PANEL_H - 8, COL_SEPARATOR);

        /* Draw each control group */
        for (int g = 0; g < N_CONTROLS; g++) {
            int gcx = g * GROUP_W + GROUP_W / 2;   /* group center x */

            /* Label */
            draw_str_cx(fb, gcx, LABEL_Y, labels[g], COL_LABEL);

            /* Value */
            char val[16];
            switch (g) {
            case 0: snprintf(val, sizeof(val), "%03.0f", heading);            break;
            case 1: snprintf(val, sizeof(val), "%03.0f", course);             break;
            case 2: snprintf(val, sizeof(val), "%+.1f", cdi_dev / 25.0f);    break;
            case 3: snprintf(val, sizeof(val), "%03.0f", hdg_bug_deg);       break;
            }
            draw_str_cx(fb, gcx, VALUE_Y, val, COL_VALUE);

            /* Buttons */
            int btn_l = g * 2;
            int btn_r = g * 2 + 1;
            draw_button(fb, BTN_L_X(g), BTN_Y, -1,
                        hovered_btn == btn_l, pressed_btn == btn_l);
            draw_button(fb, BTN_R_X(g), BTN_Y,  1,
                        hovered_btn == btn_r, pressed_btn == btn_r);
        }

        /* ---- Present ---- */
        SDL_UpdateTexture(tex, NULL, fb, WIN_W * 4);
        SDL_RenderCopy(ren, tex, NULL, NULL);
        SDL_RenderPresent(ren);
    }

    /* ---- Cleanup ---- */
    SDL_DestroyTexture(tex);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();

    free(fb);
    free(bg_l8);
    free(compass_al44);
    free(cdi_dots_al44);
    free(course_ptr.px);
    free(cdi_bar.px);
    free(hdg_bug.px);
    free(lubber.px);

    return 0;
}
