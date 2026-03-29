/*
 * Altimeter Host — runs on PC, sends UDP to Raspberry Pi receiver
 * 480x480 AAU-19/A altimeter with GUI control panel
 *
 * Usage: altimeter_host.exe [pi_ip]
 *        Default IP: 192.168.1.129, port 5555
 *
 * Controls:
 *   Up/Down arrows — baro pressure
 *   Mouse buttons  — ALT and BARO in panel
 *   SPACE          — toggle auto altitude animation
 *   R              — reset
 *   ESC            — quit
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
    uint32_t instrument_id;
    float    altitude;
    float    baro_inhg;
} AltPacket;
#define INSTRUMENT_ALT  1
#define PI_PORT         5555
#define SEND_INTERVAL   33

/* ---- Display geometry ---- */
#define DISP_W    480
#define DISP_H    480
#define PANEL_H   110
#define WIN_W     DISP_W
#define WIN_H     (DISP_H + PANEL_H)
#define CX        (DISP_W / 2)
#define CY        (DISP_H / 2)

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---- Panel layout ---- */
#define N_CONTROLS  2
#define GROUP_W     (WIN_W / N_CONTROLS)    /* 240 px per group */
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
#define COL_ARROW       0xFFDDDDDDu
#define COL_LABEL       0xFF888888u
#define COL_VALUE       0xFF44DD44u

/* ---- Drum constants ---- */
#define STRIP_WIDTH   28
#define STRIP_HEIGHT  567
#define STRIP_DIGITS  10
#define DIGIT_HEIGHT  ((float)STRIP_HEIGHT / STRIP_DIGITS)
#define DRUM_X        164
#define DRUM_Y        174
#define DRUM_VIS_H    104
#define STRIP1K_WIDTH   45
#define STRIP1K_FULL_H  583
#define STRIP1K_HEIGHT  530
#define STRIP1K_HATCH   53
#define DIGIT1K_HEIGHT  53.0f
#define DRUM1K_X        (DRUM_X - 10 - STRIP1K_WIDTH)
#define DRUM10K_X       (DRUM1K_X - STRIP1K_WIDTH)
#define DRUM1K_Y        (DRUM_Y + (DRUM_VIS_H - DRUM1K_VIS_H) / 2)
#define DRUM1K_VIS_H    51
#define BARO_GAP 0
#define BARO_Y   280

/* ---- Pointer ---- */
#define PTR_WIDTH   63
#define PTR_HEIGHT  240
#define PIVOT_X     32
#define PIVOT_Y     50

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
    {'L', {0x10,0x10,0x10,0x10,0x10,0x10,0x1F}},
    {'O', {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}},
    {'R', {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}},
    {'T', {0x1F,0x04,0x04,0x04,0x04,0x04,0x04}},
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
                        int px = x + col*FONT_SCALE + sx;
                        int py = y + row*FONT_SCALE + sy;
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
    draw_str(fb, cx - total_w/2, y, s, color);
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
    int pad = 9, th = bh-2*pad, tw = bw-2*pad;
    if (th < 2 || tw < 2) return;
    int half = th/2;
    for (int row = 0; row < th; row++) {
        int w = tw * (half - abs(row-half)) / half;
        if (w < 1) w = 1;
        int y = by + pad + row;
        int x0 = (dir < 0) ? bx+bw-pad-w : bx+pad;
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
/*  Altimeter rendering                                               */
/* ================================================================== */

static void rgb565_to_argb8888(const uint16_t *src, uint32_t *dst, int count)
{
    for (int i = 0; i < count; i++) {
        uint16_t p = src[i];
        uint8_t r = ((p>>11)&0x1F)<<3; r|=r>>5;
        uint8_t g = ((p>>5)&0x3F)<<2;  g|=g>>6;
        uint8_t b = (p&0x1F)<<3;       b|=b>>5;
        dst[i] = 0xFF000000|(r<<16)|(g<<8)|b;
    }
}

static void argb4444_to_argb8888(const uint16_t *src, uint32_t *dst, int count)
{
    for (int i = 0; i < count; i++) {
        uint16_t p = src[i];
        dst[i] = (((p>>12)&0xF)*17<<24)|(((p>>8)&0xF)*17<<16)|
                 (((p>>4)&0xF)*17<<8)|((p&0xF)*17);
    }
}

static void blit_drum(uint32_t *fb, const uint16_t *strip,
    int strip_w, int strip_h, float digit_h,
    int x, int y, int vis_h, float value)
{
    int y_offset = (int)(value * digit_h);
    int strip_y_start = y_offset - vis_h/2 + (int)(digit_h/2);
    for (int row = 0; row < vis_h; row++) {
        int sy = strip_y_start + row;
        while (sy < 0) sy += strip_h;
        while (sy >= strip_h) sy -= strip_h;
        int bg_y = y + row;
        if (bg_y < 0 || bg_y >= DISP_H) continue;
        for (int col = 0; col < strip_w; col++) {
            int bg_x = x + col;
            if (bg_x < 0 || bg_x >= DISP_W) continue;
            uint16_t p = strip[sy*strip_w+col];
            uint8_t r = ((p>>11)&0x1F)<<3; r|=r>>5;
            uint8_t g = ((p>>5)&0x3F)<<2;  g|=g>>6;
            uint8_t b = (p&0x1F)<<3;       b|=b>>5;
            fb[bg_y*WIN_W+bg_x] = 0xFF000000|(r<<16)|(g<<8)|b;
        }
    }
}

static void update_baro_drums(uint32_t *fb, const uint16_t *strip,
    int strip_w, int strip_h, float digit_h, int vis_h, float baro_inhg)
{
    float bv = baro_inhg*100;
    float d3=fmodf(bv,10), d2r=fmodf(bv/10,10), d1r=fmodf(bv/100,10), d0r=fmodf(bv/1000,10);
    float c3=d3>=9?d3-9:0; float d2=floorf(d2r)+c3;
    float c2=(d2>=9&&d2<10)?d2-9:0; float d1=floorf(d1r)+c2;
    float c1=(d1>=9&&d1<10)?d1-9:0; float d0=floorf(d0r)+c1;
    d0=fmodf(d0+10,10); d1=fmodf(d1+10,10); d2=fmodf(d2+10,10); d3=fmodf(d3+10,10);
    int x0=321-4*strip_w/2;
    blit_drum(fb,strip,strip_w,strip_h,digit_h,x0,BARO_Y,vis_h,d0);
    blit_drum(fb,strip,strip_w,strip_h,digit_h,x0+strip_w,BARO_Y,vis_h,d1);
    blit_drum(fb,strip,strip_w,strip_h,digit_h,x0+2*strip_w,BARO_Y,vis_h,d2);
    blit_drum(fb,strip,strip_w,strip_h,digit_h,x0+3*strip_w,BARO_Y,vis_h,d3);
}

static void rotate_pointer(uint32_t *fb, const uint32_t *ptr, float angle_rad)
{
    float cosA=cosf(angle_rad), sinA=sinf(angle_rad);
    int csx[4]={0,PTR_WIDTH-1,0,PTR_WIDTH-1}, csy[4]={0,0,PTR_HEIGHT-1,PTR_HEIGHT-1};
    int nx0=DISP_W,ny0=DISP_H,nx1=0,ny1=0;
    for(int c=0;c<4;c++){
        float fx=(float)(csx[c]-PIVOT_X), fy=(float)(csy[c]-PIVOT_Y);
        int dx=CX+(int)(fx*cosA-fy*sinA), dy=CY+(int)(fx*sinA+fy*cosA);
        if(dx<nx0)nx0=dx; if(dx>nx1)nx1=dx;
        if(dy<ny0)ny0=dy; if(dy>ny1)ny1=dy;
    }
    if(nx0<0)nx0=0; if(ny0<0)ny0=0;
    if(nx1>=DISP_W)nx1=DISP_W-1; if(ny1>=DISP_H)ny1=DISP_H-1;
    for(int dy=ny0;dy<=ny1;dy++) for(int dx=nx0;dx<=nx1;dx++){
        float fx=(float)(dx-CX), fy=(float)(dy-CY);
        int sx=PIVOT_X+(int)(fx*cosA+fy*sinA+0.5f);
        int sy=PIVOT_Y+(int)(-fx*sinA+fy*cosA+0.5f);
        if(sx<0||sx>=PTR_WIDTH||sy<0||sy>=PTR_HEIGHT) continue;
        uint32_t sp=ptr[sy*PTR_WIDTH+sx]; uint8_t sa=(sp>>24)&0xFF;
        if(sa==0) continue;
        if(sa==255){fb[dy*WIN_W+dx]=sp; continue;}
        uint32_t dp=fb[dy*WIN_W+dx]; uint8_t ia=255-sa;
        fb[dy*WIN_W+dx]=0xFF000000|
            ((((sp>>16&0xFF)*sa+(dp>>16&0xFF)*ia)/255)<<16)|
            ((((sp>>8&0xFF)*sa+(dp>>8&0xFF)*ia)/255)<<8)|
            (((sp&0xFF)*sa+(dp&0xFF)*ia)/255);
    }
}

static void *load_bin(const char *path, size_t expected)
{
    FILE *f=fopen(path,"rb"); if(!f){fprintf(stderr,"Cannot open %s\n",path);return NULL;}
    void *buf=malloc(expected); if(!buf){fclose(f);return NULL;}
    size_t got=fread(buf,1,expected,f); fclose(f);
    if(got!=expected){fprintf(stderr,"%s: expected %zu got %zu\n",path,expected,got);free(buf);return NULL;}
    return buf;
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
    struct sockaddr_in pi_addr;
    memset(&pi_addr,0,sizeof(pi_addr));
    pi_addr.sin_family = AF_INET;
    pi_addr.sin_port = htons(PI_PORT);
    pi_addr.sin_addr.s_addr = inet_addr(pi_ip);
    printf("=== Altimeter Host → %s:%d ===\n", pi_ip, PI_PORT);

    /* ---- Load assets ---- */
    uint16_t *bg565=load_bin("../image.bin",DISP_W*DISP_H*2); if(!bg565)return 1;
    uint16_t *strip=load_bin("../100sWheel.bin",STRIP_WIDTH*STRIP_HEIGHT*2); if(!strip)return 1;
    uint16_t *ptr4444=load_bin("../pointerargb4444.bin",PTR_WIDTH*PTR_HEIGHT*2); if(!ptr4444)return 1;
    uint16_t *strip1k_full=load_bin("../1000sWheel.bin",STRIP1K_WIDTH*STRIP1K_FULL_H*2); if(!strip1k_full)return 1;
    uint16_t *strip1k = strip1k_full + STRIP1K_WIDTH*STRIP1K_HATCH;
    uint16_t *strip10k=load_bin("../10000sWheel.bin",STRIP1K_WIDTH*STRIP1K_HEIGHT*2); if(!strip10k)return 1;

    /* Build baro strip */
    int orig_cell=(int)(DIGIT_HEIGHT+0.5f), crop_left=2, crop_right=25;
    int baro_strip_w=crop_right-crop_left+1;
    int glyph_top=orig_cell, glyph_bot=0;
    for(int d=0;d<STRIP_DIGITS;d++){
        int cell_y=(int)(d*DIGIT_HEIGHT);
        for(int row=0;row<orig_cell&&(cell_y+row)<STRIP_HEIGHT;row++){
            int has=0;
            for(int col=0;col<STRIP_WIDTH;col++){
                uint16_t p=strip[(cell_y+row)*STRIP_WIDTH+col];
                int lum=((((p>>11)&0x1F)<<3)*77+(((p>>5)&0x3F)<<2)*150+((p&0x1F)<<3)*29)>>8;
                if(lum<128){has=1;break;}
            }
            if(has){if(row<glyph_top)glyph_top=row;if(row>glyph_bot)glyph_bot=row;}
        }
    }
    int glyph_h=glyph_bot-glyph_top+1, baro_cell=glyph_h+10;
    int baro_strip_h=baro_cell*STRIP_DIGITS;
    float baro_digit_h=(float)baro_cell;
    uint16_t *baro_strip=calloc(baro_strip_w*baro_strip_h,2);
    for(int d=0;d<STRIP_DIGITS;d++){
        int src_y=(int)(d*DIGIT_HEIGHT)+glyph_top, dst_y=d*baro_cell+5;
        for(int row=0;row<glyph_h;row++){
            int sy=src_y+row; if(sy>=STRIP_HEIGHT)break;
            for(int col=0;col<baro_strip_w;col++){
                uint16_t p=strip[sy*STRIP_WIDTH+crop_left+col];
                int lum=((((p>>11)&0x1F)<<3)*77+(((p>>5)&0x3F)<<2)*150+((p&0x1F)<<3)*29)>>8;
                baro_strip[(dst_y+row)*baro_strip_w+col]=(lum<128)?0xFFFF:0x0000;
            }
        }
    }

    uint32_t *bg_clean=malloc(DISP_W*DISP_H*4);
    uint32_t *fb=calloc(WIN_W*WIN_H,4);
    uint32_t *ptr8888=malloc(PTR_WIDTH*PTR_HEIGHT*4);
    rgb565_to_argb8888(bg565,bg_clean,DISP_W*DISP_H); free(bg565);
    argb4444_to_argb8888(ptr4444,ptr8888,PTR_WIDTH*PTR_HEIGHT); free(ptr4444);

    /* ---- SDL ---- */
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window *win=SDL_CreateWindow("Altimeter Host",
        SDL_WINDOWPOS_CENTERED,SDL_WINDOWPOS_CENTERED,WIN_W,WIN_H,0);
    SDL_Renderer *ren=SDL_CreateRenderer(win,-1,
        SDL_RENDERER_ACCELERATED|SDL_RENDERER_PRESENTVSYNC);
    SDL_RenderSetLogicalSize(ren,WIN_W,WIN_H);
    SDL_Texture *tex=SDL_CreateTexture(ren,SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING,WIN_W,WIN_H);

    /* ---- State ---- */
    float altitude=8000, baro_inhg=29.92f, alt_dir=1;
    int auto_alt = 1;
    int pressed_btn = -1;
    Uint32 press_start = 0;
    int initial_step_done = 0;
    Uint32 last_tick=SDL_GetTicks(), last_send=0;

    static const char *labels[N_CONTROLS] = { "ALT", "BARO" };

    int running=1;
    while(running){
        Uint32 now=SDL_GetTicks();
        float dt=(now-last_tick)/1000.0f;
        if(dt>0.1f)dt=0.1f;
        last_tick=now;

        int mx, my;
        SDL_GetMouseState(&mx, &my);
        int hovered_btn = hit_test(mx, my);

        SDL_Event ev;
        while(SDL_PollEvent(&ev)){
            switch (ev.type) {
            case SDL_QUIT: running=0; break;
            case SDL_KEYDOWN:
                switch(ev.key.keysym.sym){
                case SDLK_ESCAPE: running=0; break;
                case SDLK_SPACE: auto_alt=!auto_alt; break;
                case SDLK_r: altitude=8000; baro_inhg=29.92f; auto_alt=1; alt_dir=1; break;
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

        /* ---- Button actions ---- */
        if (pressed_btn >= 0) {
            int dir = (pressed_btn & 1) ? 1 : -1;
            int group = pressed_btn / 2;
            float *target = NULL;
            float step = 0, cont = 0;

            switch (group) {
            case 0: target=&altitude;  step=100; cont=1000*dt; auto_alt=0; break;
            case 1: target=&baro_inhg; step=0.01f; cont=0.10f*dt; break;
            }

            if (target) {
                if (!initial_step_done) { *target+=dir*step; initial_step_done=1; }
                else if (now-press_start>400) *target+=dir*cont;
            }
        }

        /* ---- Arrow keys for baro (continuous) ---- */
        const Uint8 *keys = SDL_GetKeyboardState(NULL);
        if (keys[SDL_SCANCODE_UP])   baro_inhg += 0.05f * dt;
        if (keys[SDL_SCANCODE_DOWN]) baro_inhg -= 0.05f * dt;

        /* ---- Auto altitude animation ---- */
        if (auto_alt) {
            altitude += dt*200*alt_dir;
            if(altitude>=12000){altitude=12000;alt_dir=-1;}
            if(altitude<=8000){altitude=8000;alt_dir=1;}
        }

        /* ---- Clamp ---- */
        if(altitude>99999) altitude=99999;
        if(altitude<0) altitude=0;
        if(baro_inhg>31) baro_inhg=31;
        if(baro_inhg<28.1f) baro_inhg=28.1f;

        /* ---- Send UDP ---- */
        if(now-last_send>=SEND_INTERVAL){
            AltPacket pkt={INSTRUMENT_ALT, altitude, baro_inhg};
            sendto(sock,(char*)&pkt,sizeof(pkt),0,
                   (struct sockaddr*)&pi_addr,sizeof(pi_addr));
            last_send=now;
        }

        /* ---- Compute drum positions ---- */
        float ptr_angle=(altitude/1000)*2*(float)M_PI;
        float drum_val=fmodf(altitude/100+5,10);
        float d1kr=fmodf(altitude/1000,10), w1k=fmodf(altitude,1000);
        float d1kv=(w1k<100&&altitude>=100)?fmodf(floorf(d1kr)-1+w1k/100+10,10):floorf(d1kr);
        float d10kr=fmodf(altitude/10000,10), w10k=fmodf(altitude,10000);
        float d10kv=(w10k<100&&altitude>=100)?fmodf(floorf(d10kr)-1+w10k/100+10,10):floorf(d10kr);

        /* ---- Render instrument ---- */
        /* Copy clean background into instrument area only */
        for (int y=0; y<DISP_H; y++)
            memcpy(&fb[y*WIN_W], &bg_clean[y*DISP_W], DISP_W*4);

        blit_drum(fb,strip,STRIP_WIDTH,STRIP_HEIGHT,DIGIT_HEIGHT,DRUM_X,DRUM_Y,DRUM_VIS_H,drum_val);
        blit_drum(fb,strip1k,STRIP1K_WIDTH,STRIP1K_HEIGHT,DIGIT1K_HEIGHT,DRUM1K_X,DRUM1K_Y,DRUM1K_VIS_H,d1kv);
        blit_drum(fb,strip10k,STRIP1K_WIDTH,STRIP1K_HEIGHT,DIGIT1K_HEIGHT,DRUM10K_X,DRUM1K_Y,DRUM1K_VIS_H,d10kv);
        int baro_vis=(int)(baro_digit_h*1.1f);
        update_baro_drums(fb,baro_strip,baro_strip_w,baro_strip_h,baro_digit_h,baro_vis,baro_inhg);
        rotate_pointer(fb,ptr8888,ptr_angle);

        /* ---- Render control panel ---- */
        fill_rect(fb, 0, DISP_H, WIN_W, PANEL_H, COL_PANEL_BG);
        fill_rect(fb, 0, DISP_H, WIN_W, 1, COL_SEPARATOR);
        fill_rect(fb, GROUP_W, DISP_H+4, 1, PANEL_H-8, COL_SEPARATOR);

        for (int g=0; g<N_CONTROLS; g++) {
            int gcx = g*GROUP_W + GROUP_W/2;
            draw_str_cx(fb, gcx, LABEL_Y, labels[g], COL_LABEL);

            char val[16];
            switch(g) {
            case 0: snprintf(val,sizeof(val),"%05.0f",altitude); break;
            case 1: snprintf(val,sizeof(val),"%.2f",baro_inhg); break;
            }
            draw_str_cx(fb, gcx, VALUE_Y, val, COL_VALUE);

            draw_button(fb, BTN_L_X(g), BTN_Y, -1,
                        hovered_btn==g*2, pressed_btn==g*2);
            draw_button(fb, BTN_R_X(g), BTN_Y,  1,
                        hovered_btn==g*2+1, pressed_btn==g*2+1);
        }

        /* ---- Present ---- */
        SDL_UpdateTexture(tex,NULL,fb,WIN_W*4);
        SDL_RenderCopy(ren,tex,NULL,NULL);
        SDL_RenderPresent(ren);
    }

    closesocket(sock); WSACleanup();
    free(bg_clean);free(fb);free(ptr8888);free(strip);free(strip1k_full);free(strip10k);free(baro_strip);
    SDL_DestroyTexture(tex);SDL_DestroyRenderer(ren);SDL_DestroyWindow(win);SDL_Quit();
    return 0;
}
