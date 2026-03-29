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

/* ---- UDP packet ---- */
typedef struct {
    uint32_t instrument_id;     /* 4 = ADI */
    float    pitch;             /* degrees, positive = nose up */
    float    roll;              /* degrees, positive = right wing down */
} AdiPacket;
#define INSTRUMENT_ADI  4
#define PI_PORT         5555
#define SEND_INTERVAL   33

/* ---- Display geometry ---- */
#define DISP_W    480
#define DISP_H    480
#define PANEL_H   110
#define WIN_W     DISP_W
#define WIN_H     (DISP_H + PANEL_H)

/* ---- Panel layout (2 groups) ---- */
#define N_CONTROLS  2
#define GROUP_W     (WIN_W / N_CONTROLS)
#define BTN_W       32
#define BTN_H       28
#define BTN_Y       (DISP_H + 58)
#define LABEL_Y     (DISP_H + 14)
#define VALUE_Y     (DISP_H + 38)
#define BTN_L_X(g)  ((g) * GROUP_W + 30)
#define BTN_R_X(g)  ((g) * GROUP_W + GROUP_W - 30 - BTN_W)

/* ---- Panel colors ---- */
#define COL_PANEL_BG    0xFF1A1A1Au
#define COL_SEPARATOR   0xFF333333u
#define COL_BTN_NORMAL  0xFF3A3A3Au
#define COL_BTN_HOVER   0xFF505050u
#define COL_BTN_PRESS   0xFF686868u
#define COL_ARROW_C     0xFFDDDDDDu
#define COL_LABEL       0xFF888888u
#define COL_VALUE       0xFF44DD44u

/* ---- ADI rendering constants ---- */
#define ADI_R       190         /* main circle radius */
#define ADI_PPD     7.0f        /* pixels per degree of pitch */
#define ADI_CX      240
#define ADI_CY      240

/* ADI colors */
#define COL_SKY     0xFF1874CDu
#define COL_GND     0xFF8B5A2Bu
#define COL_WHITE   0xFFFFFFFFu
#define COL_BLACK   0xFF000000u
#define COL_GOLD    0xFFFFD700u
#define COL_BEZEL   0xFF222222u

/* ================================================================== */
/*  5x7 bitmap font                                                   */
/* ================================================================== */
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
                        int px = x+col*FONT_SCALE+sx, py = y+row*FONT_SCALE+sy;
                        if ((unsigned)px<WIN_W && (unsigned)py<WIN_H)
                            fb[py*WIN_W+px] = color;
                    }
}

static void draw_str(uint32_t *fb, int x, int y, const char *s, uint32_t color)
{ while (*s) { draw_char(fb,x,y,*s++,color); x+=CHAR_W; } }

static void draw_str_cx(uint32_t *fb, int cx, int y, const char *s, uint32_t color)
{
    int total_w = (int)strlen(s)*CHAR_W - FONT_SCALE;
    draw_str(fb, cx-total_w/2, y, s, color);
}

/* ================================================================== */
/*  GUI helpers                                                       */
/* ================================================================== */

static void fill_rect(uint32_t *fb, int rx, int ry, int rw, int rh, uint32_t c)
{
    for (int y=ry; y<ry+rh; y++) { if((unsigned)y>=WIN_H) continue;
        for (int x=rx; x<rx+rw; x++) if((unsigned)x<WIN_W) fb[y*WIN_W+x]=c; }
}

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

static void draw_button(uint32_t *fb, int bx, int by, int dir, int hover, int press)
{
    uint32_t bg=press?COL_BTN_PRESS:hover?COL_BTN_HOVER:COL_BTN_NORMAL;
    fill_rect(fb,bx,by,BTN_W,BTN_H,bg);
    draw_arrow_btn(fb,bx,by,BTN_W,BTN_H,dir,COL_ARROW_C);
}

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

/* ================================================================== */
/*  ADI procedural rendering                                          */
/* ================================================================== */

/* Set a pixel with bounds checking (stride = WIN_W for host) */
static inline void px(uint32_t *fb, int x, int y, uint32_t c)
{
    if ((unsigned)x < DISP_W && (unsigned)y < DISP_H)
        fb[y * WIN_W + x] = c;
}

/* Draw a thick line between two points */
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

static void render_adi(uint32_t *fb, float pitch, float roll, int stride)
{
    float roll_rad = roll * (float)M_PI / 180.0f;
    float pitch_rad = pitch * (float)M_PI / 180.0f;
    float cr = cosf(roll_rad), sr = sinf(roll_rad);
    float cp = cosf(pitch_rad), sp = sinf(pitch_rad);

    /* Sky/ground base colors as floats for shading */
    const float sky_r=0.094f, sky_g=0.455f, sky_b=0.804f;
    const float gnd_r=0.545f, gnd_g=0.353f, gnd_b=0.169f;

    /* Precompute pitch line sin targets */
    float pitch_sin[10];
    for (int i = 0; i < 10; i++)
        pitch_sin[i] = sinf((float)((i/2+1) * 5 * ((i%2)?-1:1)) * (float)M_PI / 180.0f);

    /* Pass 1: 3D sphere rendering */
    float inv_r = 1.0f / (float)ADI_R;
    for (int py = 0; py < DISP_H; py++) {
        float dy = (float)(py - ADI_CY);
        for (int pxx = 0; pxx < DISP_W; pxx++) {
            float dx = (float)(pxx - ADI_CX);
            float dist_sq = dx*dx + dy*dy;

            /* Outer bezel ring */
            if (dist_sq > (ADI_R+18)*(ADI_R+18)) {
                fb[py*stride+pxx] = COL_BLACK; continue;
            }
            if (dist_sq > ADI_R*ADI_R) {
                fb[py*stride+pxx] = COL_BEZEL; continue;
            }

            /* Map pixel to unit sphere: (u, v, z) */
            float u = dx * inv_r;
            float v = -dy * inv_r;       /* flip y to point up */
            float z = sqrtf(1.0f - u*u - v*v);

            /* Inverse roll: rotate (u,v) around z-axis */
            float u1 =  u*cr + v*sr;
            float v1 = -u*sr + v*cr;

            /* Inverse pitch: rotate (v1,z) around x-axis */
            float vy = v1*cp + z*sp;     /* world "up" component */
            float vx = u1;               /* world "right" component */
            float vz = -v1*sp + z*cp;    /* world "forward" component */

            /* Shading: edge darkening from sphere curvature */
            float shade = 0.35f + 0.65f * z;

            /* Sky or ground */
            float r, g, b;
            if (vy > 0) { r=sky_r; g=sky_g; b=sky_b; }
            else         { r=gnd_r; g=gnd_g; b=gnd_b; }

            /* Horizon line (great circle at equator) */
            if (fabsf(vy) < 0.012f) {
                r=1; g=1; b=1;
                shade = (shade < 0.7f) ? 0.7f : shade;
            }

            /* Pitch lines: check latitude against each line */
            float lon = fabsf(atan2f(vx, vz));
            for (int i = 0; i < 10; i++) {
                float target = pitch_sin[i];
                int deg5 = (i/2 + 1) * 5;
                float hw_rad = (deg5 % 10 == 0) ? 0.35f : 0.20f;
                float thick  = (deg5 % 10 == 0) ? 0.008f : 0.006f;

                if (fabsf(vy - target) < thick && lon < hw_rad) {
                    r=1; g=1; b=1;
                    shade = (shade < 0.7f) ? 0.7f : shade;
                }

                /* Vertical endcaps on 10° lines */
                if (deg5 % 10 == 0) {
                    float dvy = fabsf(vy - target);
                    if (dvy < 0.04f && dvy >= thick &&
                        lon > hw_rad - 0.03f && lon < hw_rad + 0.01f) {
                        int above = (target > 0) ? (vy < target) : (vy > target);
                        if (above) { r=1; g=1; b=1; shade=(shade<0.7f)?0.7f:shade; }
                    }
                }
            }

            /* Apply shading */
            uint8_t rb = (uint8_t)(r * shade * 255.0f);
            uint8_t gb2 = (uint8_t)(g * shade * 255.0f);
            uint8_t bb = (uint8_t)(b * shade * 255.0f);
            fb[py*stride+pxx] = 0xFF000000u | ((uint32_t)rb<<16) | ((uint32_t)gb2<<8) | bb;
        }
    }

    /* Pass 2: Roll scale ticks on bezel */
    static const float tick_angles[] = {0,10,20,30,45,60,-10,-20,-30,-45,-60};
    for (int i = 0; i < 11; i++) {
        float a_rad = (-90.0f + tick_angles[i]) * (float)M_PI / 180.0f;
        float ca = cosf(a_rad), sa = sinf(a_rad);
        int r0 = ADI_R + 2;
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

    /* Pass 3: Roll pointer (triangle at current roll angle) */
    {
        float a_rad = (-90.0f + roll) * (float)M_PI / 180.0f;
        float ca = cosf(a_rad), sa = sinf(a_rad);
        /* Perpendicular direction */
        float px_d = -sa, py_d = ca;

        float tip_r = ADI_R + 3;
        float base_r = ADI_R + 16;
        float tip_x = ADI_CX + tip_r*ca;
        float tip_y = ADI_CY + tip_r*sa;
        float base_x = ADI_CX + base_r*ca;
        float base_y = ADI_CY + base_r*sa;
        float hw = 6.0f;   /* half-width of triangle base */

        /* Three vertices */
        float v0x=tip_x, v0y=tip_y;
        float v1x=base_x+hw*px_d, v1y=base_y+hw*py_d;
        float v2x=base_x-hw*px_d, v2y=base_y-hw*py_d;

        /* Bounding box */
        int mn_x = (int)fminf(fminf(v0x,v1x),v2x) - 1;
        int mx_x = (int)fmaxf(fmaxf(v0x,v1x),v2x) + 1;
        int mn_y = (int)fminf(fminf(v0y,v1y),v2y) - 1;
        int mx_y = (int)fmaxf(fmaxf(v0y,v1y),v2y) + 1;

        for (int ty=mn_y; ty<=mx_y; ty++) {
            if ((unsigned)ty >= DISP_H) continue;
            for (int tx=mn_x; tx<=mx_x; tx++) {
                if ((unsigned)tx >= DISP_W) continue;
                /* Edge function test */
                float e0 = (v1x-v0x)*((float)ty-v0y) - (v1y-v0y)*((float)tx-v0x);
                float e1 = (v2x-v1x)*((float)ty-v1y) - (v2y-v1y)*((float)tx-v1x);
                float e2 = (v0x-v2x)*((float)ty-v2y) - (v0y-v2y)*((float)tx-v2x);
                if ((e0>=0&&e1>=0&&e2>=0) || (e0<=0&&e1<=0&&e2<=0))
                    fb[ty*stride+tx] = COL_WHITE;
            }
        }
    }

    /* Pass 4: Aircraft symbol (fixed at center) */
    /* Center dot */
    for (int dy=-4; dy<=4; dy++)
        for (int dx=-4; dx<=4; dx++)
            if (dx*dx+dy*dy <= 16)
                fb[(ADI_CY+dy)*stride+ADI_CX+dx] = COL_GOLD;

    /* Wings */
    for (int x=-48; x<=48; x++) {
        if (abs(x) < 7) continue;   /* gap around dot */
        for (int t=-1; t<=1; t++)    /* 3px thick */
            fb[(ADI_CY+t)*stride+ADI_CX+x] = COL_GOLD;
    }

    /* Wing tips (vertical drops) */
    for (int y=0; y<=12; y++) {
        fb[(ADI_CY+y)*stride+ADI_CX-48] = COL_GOLD;
        fb[(ADI_CY+y)*stride+ADI_CX-49] = COL_GOLD;
        fb[(ADI_CY+y)*stride+ADI_CX+48] = COL_GOLD;
        fb[(ADI_CY+y)*stride+ADI_CX+49] = COL_GOLD;
    }

    /* Fixed top index triangle (small white triangle at 12 o'clock) */
    for (int row=0; row<8; row++) {
        int half_w = row;
        for (int dx=-half_w; dx<=half_w; dx++)
            fb[(ADI_CY-ADI_R+8-row)*stride+ADI_CX+dx] = COL_WHITE;
    }
}

/* ================================================================== */
/*  Main                                                              */
/* ================================================================== */
int main(int argc, char *argv[])
{
    const char *pi_ip = "192.168.1.129";
    if (argc > 1) pi_ip = argv[1];

    WSADATA wsa; WSAStartup(MAKEWORD(2,2), &wsa);
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in pi_addr = {0};
    pi_addr.sin_family = AF_INET;
    pi_addr.sin_port = htons(PI_PORT);
    pi_addr.sin_addr.s_addr = inet_addr(pi_ip);
    printf("=== ADI Host → %s:%d ===\n", pi_ip, PI_PORT);

    uint32_t *fb = calloc(WIN_W*WIN_H, 4);

    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window *win = SDL_CreateWindow("ADI Host",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIN_W, WIN_H, 0);
    SDL_Renderer *ren = SDL_CreateRenderer(win, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    SDL_RenderSetLogicalSize(ren, WIN_W, WIN_H);
    SDL_Texture *tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING, WIN_W, WIN_H);

    float pitch = 0, roll_deg = 0;
    int auto_anim = 1;
    int pressed_btn = -1;
    Uint32 press_start = 0;
    int initial_step_done = 0;
    Uint32 last_tick = SDL_GetTicks(), last_send = 0;

    static const char *labels[N_CONTROLS] = {"PITCH", "ROLL"};

    int running = 1;
    while (running) {
        Uint32 now = SDL_GetTicks();
        float dt = (now-last_tick)/1000.0f;
        if (dt > 0.1f) dt = 0.1f;
        last_tick = now;

        int mx, my;
        SDL_GetMouseState(&mx, &my);
        int hovered_btn = hit_test(mx, my);

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

        /* Button actions */
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

        /* Arrow keys */
        const Uint8 *keys = SDL_GetKeyboardState(NULL);
        if(keys[SDL_SCANCODE_UP])    { pitch+=15*dt; auto_anim=0; }
        if(keys[SDL_SCANCODE_DOWN])  { pitch-=15*dt; auto_anim=0; }
        if(keys[SDL_SCANCODE_LEFT])  { roll_deg-=30*dt; auto_anim=0; }
        if(keys[SDL_SCANCODE_RIGHT]) { roll_deg+=30*dt; auto_anim=0; }

        /* Auto animation: gentle flying */
        if (auto_anim) {
            float t = now / 1000.0f;
            pitch = 8.0f * sinf(t * 0.4f) + 3.0f * sinf(t * 1.1f);
            roll_deg = 15.0f * sinf(t * 0.25f) + 8.0f * sinf(t * 0.7f);
        }

        /* Clamp */
        if(pitch>30) pitch=30; if(pitch<-30) pitch=-30;
        if(roll_deg>60) roll_deg=60; if(roll_deg<-60) roll_deg=-60;

        /* Send UDP */
        if (now-last_send >= SEND_INTERVAL) {
            AdiPacket pkt = {INSTRUMENT_ADI, pitch, roll_deg};
            sendto(sock,(char*)&pkt,sizeof(pkt),0,
                   (struct sockaddr*)&pi_addr,sizeof(pi_addr));
            last_send = now;
        }

        /* ---- Render ADI ---- */
        render_adi(fb, pitch, roll_deg, WIN_W);

        /* ---- Render control panel ---- */
        fill_rect(fb, 0, DISP_H, WIN_W, PANEL_H, COL_PANEL_BG);
        fill_rect(fb, 0, DISP_H, WIN_W, 1, COL_SEPARATOR);
        fill_rect(fb, GROUP_W, DISP_H+4, 1, PANEL_H-8, COL_SEPARATOR);

        for (int g=0; g<N_CONTROLS; g++) {
            int gcx = g*GROUP_W + GROUP_W/2;
            draw_str_cx(fb, gcx, LABEL_Y, labels[g], COL_LABEL);
            char val[16];
            switch(g) {
            case 0: snprintf(val,sizeof(val),"%+.0f",pitch); break;
            case 1: snprintf(val,sizeof(val),"%+.0f",roll_deg); break;
            }
            draw_str_cx(fb, gcx, VALUE_Y, val, COL_VALUE);
            draw_button(fb,BTN_L_X(g),BTN_Y,-1,hovered_btn==g*2,pressed_btn==g*2);
            draw_button(fb,BTN_R_X(g),BTN_Y, 1,hovered_btn==g*2+1,pressed_btn==g*2+1);
        }

        SDL_UpdateTexture(tex,NULL,fb,WIN_W*4);
        SDL_RenderCopy(ren,tex,NULL,NULL);
        SDL_RenderPresent(ren);
    }

    closesocket(sock); WSACleanup();
    free(fb);
    SDL_DestroyTexture(tex); SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win); SDL_Quit();
    return 0;
}
