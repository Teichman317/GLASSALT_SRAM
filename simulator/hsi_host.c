/*
 * HSI Host — runs on PC, sends UDP to Raspberry Pi receiver
 * Desktop SDL2 renderer for 480x480 KI-525A HSI face
 * with clickable GUI control panel
 *
 * Usage: hsi_host.exe [pi_ip_address]
 *        Default IP: 192.168.1.129, port 5555
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
#include <winsock2.h>

#pragma comment(lib, "ws2_32.lib")

/* ---- UDP packet ---- */
typedef struct {
    uint32_t instrument_id;     /* 3 = HSI */
    float    heading;           /* degrees, 0-360 */
    float    course;            /* degrees, 0-360 */
    float    heading_bug;       /* degrees, 0-360 */
    float    cdi_dev;           /* dots, -3 to +3 */
} HsiPacket;

#define INSTRUMENT_HSI  3
#define PI_PORT         5555
#define SEND_INTERVAL   33      /* ~30 Hz */

/* ---- Display geometry ---- */
#define DISP_W    480
#define DISP_H    480
#define PANEL_H   110
#define WIN_W     DISP_W
#define WIN_H     (DISP_H + PANEL_H)
#define CX        (DISP_W / 2)
#define CY        (DISP_H / 2)

/* ---- Panel layout ---- */
#define N_CONTROLS  4
#define GROUP_W     (WIN_W / N_CONTROLS)
#define BTN_W       32
#define BTN_H       28
#define BTN_Y       (DISP_H + 58)
#define LABEL_Y     (DISP_H + 14)
#define VALUE_Y     (DISP_H + 38)
#define BTN_L_X(g)  ((g) * GROUP_W + 6)
#define BTN_R_X(g)  ((g) * GROUP_W + GROUP_W - 6 - BTN_W)

/* ---- Panel colors ---- */
#define COL_PANEL_BG    0xFF1A1A1Au
#define COL_SEPARATOR   0xFF333333u
#define COL_BTN_NORMAL  0xFF3A3A3Au
#define COL_BTN_HOVER   0xFF505050u
#define COL_BTN_PRESS   0xFF686868u
#define COL_ARROW       0xFFDDDDDDu
#define COL_LABEL       0xFF888888u
#define COL_VALUE       0xFF44DD44u

/* ================================================================== */
/*  5x7 bitmap font                                                   */
/* ================================================================== */
#define FONT_W  5
#define FONT_H  7
#define FONT_SCALE  2
#define CHAR_W  ((FONT_W + 1) * FONT_SCALE)
#define CHAR_H  (FONT_H * FONT_SCALE)

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
{ while (*s) { draw_char(fb, x, y, *s++, color); x += CHAR_W; } }

static void draw_str_cx(uint32_t *fb, int cx, int y, const char *s, uint32_t color)
{
    int len = (int)strlen(s);
    int total_w = len * CHAR_W - FONT_SCALE;
    draw_str(fb, cx - total_w / 2, y, s, color);
}

/* ================================================================== */
/*  GUI helpers                                                       */
/* ================================================================== */

static void fill_rect(uint32_t *fb, int rx, int ry, int rw, int rh, uint32_t c)
{
    for (int y = ry; y < ry + rh; y++) {
        if ((unsigned)y >= WIN_H) continue;
        for (int x = rx; x < rx + rw; x++)
            if ((unsigned)x < WIN_W) fb[y * WIN_W + x] = c;
    }
}

static void draw_arrow(uint32_t *fb, int bx, int by, int bw, int bh,
                        int dir, uint32_t color)
{
    int pad = 9, th = bh - 2*pad, tw = bw - 2*pad;
    if (th < 2 || tw < 2) return;
    int half = th / 2;
    for (int row = 0; row < th; row++) {
        int w = tw * (half - abs(row - half)) / half;
        if (w < 1) w = 1;
        int y = by + pad + row;
        int x0 = (dir < 0) ? bx + bw - pad - w : bx + pad;
        for (int c2 = 0; c2 < w; c2++)
            if ((unsigned)(x0+c2) < WIN_W && (unsigned)y < WIN_H)
                fb[y * WIN_W + x0 + c2] = color;
    }
}

static void draw_button(uint32_t *fb, int bx, int by, int dir, int hover, int press)
{
    uint32_t bg = press ? COL_BTN_PRESS : hover ? COL_BTN_HOVER : COL_BTN_NORMAL;
    fill_rect(fb, bx, by, BTN_W, BTN_H, bg);
    draw_arrow(fb, bx, by, BTN_W, BTN_H, dir, COL_ARROW);
}

/* ================================================================== */
/*  ARGB1555 sprite                                                   */
/* ================================================================== */
typedef struct { int ox, oy, w, h; uint16_t *px; } Sprite;

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
    if (fread(s->px, 2, npx, f) != npx) { free(s->px); fclose(f); return -1; }
    fclose(f);
    return 0;
}

/* ================================================================== */
/*  Instrument compositing (identical to hsi_sim.c)                   */
/* ================================================================== */

static inline void blend_pixel(uint32_t *dst, uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    if (a == 0) return;
    if (a == 255) { *dst = 0xFF000000u|((uint32_t)r<<16)|((uint32_t)g<<8)|b; return; }
    uint32_t d = *dst;
    uint8_t dr = (d>>16)&0xFF, dg = (d>>8)&0xFF, db = d&0xFF;
    *dst = 0xFF000000u |
        ((uint32_t)((r*a + dr*(255-a))/255) << 16) |
        ((uint32_t)((g*a + dg*(255-a))/255) <<  8) |
        (uint32_t)((b*a + db*(255-a))/255);
}

static void blit_background(uint32_t *fb, const uint8_t *bg)
{
    for (int i = 0; i < DISP_W * DISP_H; i++) {
        uint8_t l = bg[i];
        fb[i] = 0xFF000000u | ((uint32_t)l<<16) | ((uint32_t)l<<8) | l;
    }
}

static void rotate_blend_al44(uint32_t *fb, const uint8_t *src, float deg)
{
    float rad = deg * (float)M_PI / 180.0f;
    float cs = cosf(rad), sn = sinf(rad);
    for (int dy = 0; dy < DISP_H; dy++) {
        float fy = (float)(dy - CY);
        for (int dx = 0; dx < DISP_W; dx++) {
            float fx = (float)(dx - CX);
            int sx = (int)(CX + fx*cs + fy*sn + 0.5f);
            int sy = (int)(CY - fx*sn + fy*cs + 0.5f);
            if ((unsigned)sx >= DISP_W || (unsigned)sy >= DISP_H) continue;
            uint8_t p = src[sy * DISP_W + sx];
            uint8_t a4 = p >> 4;
            if (a4 == 0) continue;
            blend_pixel(&fb[dy*WIN_W+dx], (p&0xF)*17, (p&0xF)*17, (p&0xF)*17, a4*17);
        }
    }
}

static void rotate_blend_sprite(uint32_t *fb, const Sprite *s,
                                float deg, float tx, float ty)
{
    float rad = deg*(float)M_PI/180.0f;
    float cs = cosf(rad), sn = sinf(rad);
    float corners[4][2] = {
        {s->ox+tx, s->oy+ty}, {s->ox+s->w+tx, s->oy+ty},
        {s->ox+tx, s->oy+s->h+ty}, {s->ox+s->w+tx, s->oy+s->h+ty}
    };
    int mn_x=DISP_W, mn_y=DISP_H, mx_x=0, mx_y=0;
    for (int i=0;i<4;i++) {
        float fx=corners[i][0]-CX, fy=corners[i][1]-CY;
        int rx=(int)(CX+fx*cs-fy*sn), ry=(int)(CY+fx*sn+fy*cs);
        if(rx<mn_x)mn_x=rx; if(rx>mx_x)mx_x=rx;
        if(ry<mn_y)mn_y=ry; if(ry>mx_y)mx_y=ry;
    }
    if(mn_x<0)mn_x=0; else if(mn_x>2)mn_x-=2;
    if(mn_y<0)mn_y=0; else if(mn_y>2)mn_y-=2;
    if(mx_x>=DISP_W)mx_x=DISP_W-1; else mx_x+=2;
    if(mx_y>=DISP_H)mx_y=DISP_H-1; else mx_y+=2;
    if(mx_x>=DISP_W)mx_x=DISP_W-1;
    if(mx_y>=DISP_H)mx_y=DISP_H-1;

    for (int dy=mn_y; dy<=mx_y; dy++) {
        float fy=(float)(dy-CY);
        for (int dx=mn_x; dx<=mx_x; dx++) {
            float fx=(float)(dx-CX);
            float sx_f=CX+fx*cs+fy*sn-tx, sy_f=CY-fx*sn+fy*cs-ty;
            int lx=(int)(sx_f-s->ox+0.5f), ly=(int)(sy_f-s->oy+0.5f);
            if((unsigned)lx>=(unsigned)s->w||(unsigned)ly>=(unsigned)s->h) continue;
            uint16_t p=s->px[ly*s->w+lx];
            if(!(p&0x8000)) continue;
            fb[dy*WIN_W+dx] = 0xFF000000u |
                ((uint32_t)(((p>>10)&0x1F)<<3)<<16) |
                ((uint32_t)(((p>>5)&0x1F)<<3)<<8) |
                (uint32_t)((p&0x1F)<<3);
        }
    }
}

static void blit_sprite(uint32_t *fb, const Sprite *s)
{
    for (int y=0; y<s->h; y++) {
        int dy=s->oy+y; if((unsigned)dy>=DISP_H) continue;
        for (int x=0; x<s->w; x++) {
            int dx=s->ox+x; if((unsigned)dx>=DISP_W) continue;
            uint16_t p=s->px[y*s->w+x]; if(!(p&0x8000)) continue;
            fb[dy*WIN_W+dx] = 0xFF000000u |
                ((uint32_t)(((p>>10)&0x1F)<<3)<<16) |
                ((uint32_t)(((p>>5)&0x1F)<<3)<<8) |
                (uint32_t)((p&0x1F)<<3);
        }
    }
}

/* ================================================================== */
/*  Button hit testing                                                */
/* ================================================================== */
#define N_BUTTONS (N_CONTROLS * 2)

static int hit_test(int mx, int my)
{
    for (int i = 0; i < N_BUTTONS; i++) {
        int g = i/2, is_r = i&1;
        int bx = is_r ? BTN_R_X(g) : BTN_L_X(g);
        if (mx >= bx && mx < bx+BTN_W && my >= BTN_Y && my < BTN_Y+BTN_H)
            return i;
    }
    return -1;
}

/* ================================================================== */
/*  Main                                                              */
/* ================================================================== */
int main(int argc, char *argv[])
{
    const char *pi_ip = "192.168.1.129";
    if (argc > 1) pi_ip = argv[1];

    /* ---- Winsock + UDP socket ---- */
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2,2), &wsa) != 0) {
        fprintf(stderr, "WSAStartup failed\n"); return 1;
    }
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
        fprintf(stderr, "socket() failed\n"); return 1;
    }
    struct sockaddr_in pi_addr;
    memset(&pi_addr, 0, sizeof(pi_addr));
    pi_addr.sin_family = AF_INET;
    pi_addr.sin_port = htons(PI_PORT);
    pi_addr.sin_addr.s_addr = inet_addr(pi_ip);

    printf("=== HSI Host ===\n");
    printf("Sending UDP to %s:%d\n\n", pi_ip, PI_PORT);

    /* ---- Load assets ---- */
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

    /* ---- Framebuffer ---- */
    uint32_t *fb = calloc(WIN_W * WIN_H, sizeof(uint32_t));
    if (!fb) return 1;

    /* ---- SDL ---- */
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window *win = SDL_CreateWindow("HSI Host",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIN_W, WIN_H, 0);
    SDL_Renderer *ren = SDL_CreateRenderer(win, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    SDL_RenderSetLogicalSize(ren, WIN_W, WIN_H);
    SDL_Texture *tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING, WIN_W, WIN_H);

    /* ---- State ---- */
    float heading = 0, course = 45, hdg_bug_deg = 90, cdi_dev = 0;
    int auto_hdg = 1, auto_cdi = 1;
    int pressed_btn = -1;
    Uint32 press_start = 0;
    int initial_step_done = 0;
    Uint32 last_send = 0;
    int packets_sent = 0;

    static const char *labels[N_CONTROLS] = { "HDG", "CRS", "DEV", "BUG" };

    int running = 1;
    Uint32 last_tick = SDL_GetTicks();

    while (running) {
        Uint32 now = SDL_GetTicks();
        float dt = (now - last_tick) / 1000.0f;
        if (dt > 0.1f) dt = 0.1f;
        last_tick = now;

        int mx, my;
        SDL_GetMouseState(&mx, &my);
        int hovered_btn = hit_test(mx, my);

        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            switch (ev.type) {
            case SDL_QUIT: running = 0; break;
            case SDL_KEYDOWN:
                switch (ev.key.keysym.sym) {
                case SDLK_ESCAPE: running = 0; break;
                case SDLK_SPACE:  auto_hdg = !auto_hdg; break;
                case SDLK_TAB:    auto_cdi = !auto_cdi; break;
                case SDLK_r:
                    heading=0; course=45; hdg_bug_deg=90;
                    cdi_dev=0; auto_hdg=1; auto_cdi=1; break;
                default: break;
                } break;
            case SDL_MOUSEBUTTONDOWN:
                if (ev.button.button == SDL_BUTTON_LEFT) {
                    pressed_btn = hit_test(ev.button.x, ev.button.y);
                    press_start = now; initial_step_done = 0;
                } break;
            case SDL_MOUSEBUTTONUP:
                if (ev.button.button == SDL_BUTTON_LEFT) pressed_btn = -1;
                break;
            }
        }

        /* Button actions */
        if (pressed_btn >= 0) {
            int dir = (pressed_btn&1) ? 1 : -1;
            int group = pressed_btn/2;
            float *target = NULL;
            float step=1, cont=45*dt;
            switch (group) {
            case 0: target=&heading;     auto_hdg=0; break;
            case 1: target=&course;      break;
            case 2: target=&cdi_dev;     step=5; cont=75*dt; auto_cdi=0; break;
            case 3: target=&hdg_bug_deg; break;
            }
            if (target) {
                if (!initial_step_done) { *target+=dir*step; initial_step_done=1; }
                else if (now-press_start>400) *target+=dir*cont;
            }
        }

        /* Keyboard */
        const Uint8 *keys = SDL_GetKeyboardState(NULL);
        if (keys[SDL_SCANCODE_LEFT])  { heading-=30*dt; auto_hdg=0; }
        if (keys[SDL_SCANCODE_RIGHT]) { heading+=30*dt; auto_hdg=0; }
        if (keys[SDL_SCANCODE_UP])    course-=30*dt;
        if (keys[SDL_SCANCODE_DOWN])  course+=30*dt;
        if (keys[SDL_SCANCODE_Q])     hdg_bug_deg-=30*dt;
        if (keys[SDL_SCANCODE_W])     hdg_bug_deg+=30*dt;
        if (keys[SDL_SCANCODE_A])     { cdi_dev-=50*dt; auto_cdi=0; }
        if (keys[SDL_SCANCODE_D])     { cdi_dev+=50*dt; auto_cdi=0; }

        /* Auto animations */
        if (auto_hdg) heading += 6.0f * dt;
        if (auto_cdi) {
            float t = fmodf(now/1000.0f, 8.0f);
            cdi_dev = (t<4) ? -50+25*t : 150-25*t;
        }

        /* Wrap & clamp */
        heading=fmodf(heading,360); if(heading<0) heading+=360;
        course=fmodf(course,360);   if(course<0)  course+=360;
        hdg_bug_deg=fmodf(hdg_bug_deg,360); if(hdg_bug_deg<0) hdg_bug_deg+=360;
        if(cdi_dev>75) cdi_dev=75; if(cdi_dev<-75) cdi_dev=-75;

        /* ---- Send UDP at 30 Hz ---- */
        if (now - last_send >= SEND_INTERVAL) {
            HsiPacket pkt;
            pkt.instrument_id = INSTRUMENT_HSI;
            pkt.heading = heading;
            pkt.course = course;
            pkt.heading_bug = hdg_bug_deg;
            pkt.cdi_dev = cdi_dev / 25.0f;   /* convert pixels to dots */
            sendto(sock, (const char *)&pkt, sizeof(pkt), 0,
                   (struct sockaddr *)&pi_addr, sizeof(pi_addr));
            last_send = now;
            packets_sent++;
        }

        /* ---- Rotation angles ---- */
        float card_rot = -heading;
        float crs_rot  = course - heading;
        float bug_rot  = hdg_bug_deg - heading;

        /* ---- Render instrument ---- */
        blit_background(fb, bg_l8);
        rotate_blend_al44(fb, cdi_dots_al44, crs_rot);
        rotate_blend_al44(fb, compass_al44, card_rot);
        rotate_blend_sprite(fb, &hdg_bug, bug_rot, 0, 0);
        rotate_blend_sprite(fb, &course_ptr, crs_rot, 0, 0);
        rotate_blend_sprite(fb, &cdi_bar, crs_rot, cdi_dev, 0);
        blit_sprite(fb, &lubber);

        /* ---- Render control panel ---- */
        fill_rect(fb, 0, DISP_H, WIN_W, PANEL_H, COL_PANEL_BG);
        fill_rect(fb, 0, DISP_H, WIN_W, 1, COL_SEPARATOR);
        for (int g=1; g<N_CONTROLS; g++)
            fill_rect(fb, g*GROUP_W, DISP_H+4, 1, PANEL_H-8, COL_SEPARATOR);

        for (int g=0; g<N_CONTROLS; g++) {
            int gcx = g*GROUP_W + GROUP_W/2;
            draw_str_cx(fb, gcx, LABEL_Y, labels[g], COL_LABEL);
            char val[16];
            switch(g) {
            case 0: snprintf(val,sizeof(val),"%03.0f",heading); break;
            case 1: snprintf(val,sizeof(val),"%03.0f",course); break;
            case 2: snprintf(val,sizeof(val),"%+.1f",cdi_dev/25.0f); break;
            case 3: snprintf(val,sizeof(val),"%03.0f",hdg_bug_deg); break;
            }
            draw_str_cx(fb, gcx, VALUE_Y, val, COL_VALUE);
            draw_button(fb, BTN_L_X(g), BTN_Y, -1,
                        hovered_btn==g*2, pressed_btn==g*2);
            draw_button(fb, BTN_R_X(g), BTN_Y,  1,
                        hovered_btn==g*2+1, pressed_btn==g*2+1);
        }

        /* ---- Present ---- */
        SDL_UpdateTexture(tex, NULL, fb, WIN_W*4);
        SDL_RenderCopy(ren, tex, NULL, NULL);
        SDL_RenderPresent(ren);
    }

    closesocket(sock);
    WSACleanup();
    SDL_DestroyTexture(tex); SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win); SDL_Quit();
    free(fb); free(bg_l8); free(compass_al44); free(cdi_dots_al44);
    free(course_ptr.px); free(cdi_bar.px); free(hdg_bug.px); free(lubber.px);
    return 0;
}
