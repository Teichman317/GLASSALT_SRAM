/*
 * ASI (Mach Airspeed Indicator) Host — Gulfstream-style
 * Procedural rendering — no bitmap assets
 * Non-linear airspeed dial, barber-pole VMO pointer,
 * Mach drum display, OFF/VMO flag
 *
 * Usage: asi_host.exe [pi_ip]
 *
 * Controls:
 *   Up/Down      — IAS
 *   Left/Right   — VMO pointer
 *   Q/W          — Mach (manual adjust)
 *   SPACE        — toggle auto animation
 *   R            — reset
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

/* ---- UDP ---- */
typedef struct {
    uint32_t instrument_id;     /* 5 = ASI */
    float    ias;               /* knots */
    float    mach;              /* 0.10-0.99 */
    float    vmo;               /* knots */
} AsiPacket;
#define INSTRUMENT_ASI  5
#define PI_PORT         5555
#define SEND_INTERVAL   33

/* ---- Display ---- */
#define DISP_W    480
#define DISP_H    480
#define PANEL_H   110
#define WIN_W     DISP_W
#define WIN_H     (DISP_H + PANEL_H)
#define CX        240
#define CY        240

/* ---- Panel layout (3 groups) ---- */
#define N_CONTROLS  3
#define GROUP_W     (WIN_W / N_CONTROLS)
#define BTN_W       32
#define BTN_H       28
#define BTN_Y       (DISP_H + 58)
#define LABEL_Y     (DISP_H + 14)
#define VALUE_Y     (DISP_H + 38)
#define BTN_L_X(g)  ((g) * GROUP_W + 6)
#define BTN_R_X(g)  ((g) * GROUP_W + GROUP_W - 6 - BTN_W)

#define COL_PANEL_BG    0xFF1A1A1Au
#define COL_SEPARATOR   0xFF333333u
#define COL_BTN_NORMAL  0xFF3A3A3Au
#define COL_BTN_HOVER   0xFF505050u
#define COL_BTN_PRESS   0xFF686868u
#define COL_ARROW_C     0xFFDDDDDDu
#define COL_LABEL       0xFF888888u
#define COL_VALUE_G     0xFF44DD44u

/* ---- Dial constants ---- */
#define DIAL_R      195         /* outer edge of dial */
#define TICK_MIN    12          /* minor tick length */
#define TICK_MAJ    (TICK_MIN*2) /* major = 2x minor length */
/* Both share same inner radius; major extends outward beyond minor */
#define TICK_INNER  (DIAL_R - TICK_MAJ)  /* inner radius for major ticks */
#define TICK_MINOR_INNER (DIAL_R - TICK_MIN)  /* inner radius for minor ticks */
#define NUM_R       155         /* number center radius */
#define PTR_LEN     170         /* pointer length from center */
#define HUB_R       18          /* center hub radius */

/* ================================================================== */
/*  5x7 bitmap font                                                   */
/* ================================================================== */
#define FONT_W 5
#define FONT_H 7

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
    {'C', {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}},
    {'E', {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}},
    {'F', {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10}},
    {'H', {0x11,0x11,0x11,0x1F,0x11,0x11,0x11}},
    {'I', {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}},
    {'K', {0x11,0x12,0x14,0x18,0x14,0x12,0x11}},
    {'L', {0x10,0x10,0x10,0x10,0x10,0x10,0x1F}},
    {'M', {0x11,0x1B,0x15,0x15,0x11,0x11,0x11}},
    {'O', {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}},
    {'S', {0x0E,0x11,0x10,0x0E,0x01,0x11,0x0E}},
    {'T', {0x1F,0x04,0x04,0x04,0x04,0x04,0x04}},
    {'V', {0x11,0x11,0x11,0x11,0x0A,0x0A,0x04}},
    {'.', {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C}},
    {' ', {0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    {'+', {0x00,0x04,0x04,0x1F,0x04,0x04,0x00}},
    {'-', {0x00,0x00,0x00,0x1F,0x00,0x00,0x00}},
};
#define N_GLYPHS ((int)(sizeof(glyphs)/sizeof(glyphs[0])))

static const uint8_t *font_lookup(char c)
{
    for (int i = 0; i < N_GLYPHS; i++)
        if (glyphs[i].ch == c) return glyphs[i].r;
    return NULL;
}

/* Draw a character at (cx,cy) rotated by angle_rad, scale factor */
static void draw_rotated_char(uint32_t *fb, int stride, int fcx, int fcy,
                               char c, float angle_rad, uint32_t color, int scale)
{
    const uint8_t *bits = font_lookup(c);
    if (!bits) return;
    float cs = cosf(angle_rad), sn = sinf(angle_rad);
    float hw = (FONT_W * scale) / 2.0f;
    float hh = (FONT_H * scale) / 2.0f;

    for (int row = 0; row < FONT_H; row++)
        for (int col = 0; col < FONT_W; col++) {
            if (!(bits[row] & (0x10 >> col))) continue;
            for (int sy = 0; sy < scale; sy++)
                for (int sx = 0; sx < scale; sx++) {
                    float lx = col*scale + sx - hw;
                    float ly = row*scale + sy - hh;
                    int px = (int)(fcx + lx*cs - ly*sn + 0.5f);
                    int py = (int)(fcy + lx*sn + ly*cs + 0.5f);
                    if ((unsigned)px < DISP_W && (unsigned)py < DISP_H)
                        fb[py*stride+px] = color;
                }
        }
}

/* Draw a string along the tangent at a dial position */
static void draw_dial_text(uint32_t *fb, int stride, float angle_deg,
                            float radius, const char *text, uint32_t color, int scale)
{
    float angle_rad = angle_deg * (float)M_PI / 180.0f;
    int tcx = CX + (int)(radius * sinf(angle_rad));
    int tcy = CY - (int)(radius * cosf(angle_rad));

    /* Text rotation: tangent to the dial, readable from outside */
    float text_rot = angle_rad;
    float tang_x = cosf(angle_rad);
    float tang_y = sinf(angle_rad);

    if (angle_deg < 0) { angle_deg += 360; }
    if (angle_deg > 90 && angle_deg < 270) {
        text_rot += (float)M_PI;
        tang_x = -tang_x;
        tang_y = -tang_y;
    }

    int len = (int)strlen(text);
    float spacing = (FONT_W + 1) * scale;

    for (int i = 0; i < len; i++) {
        float offset = (i - (len - 1) / 2.0f) * spacing;
        int ccx = tcx + (int)(offset * tang_x);
        int ccy = tcy + (int)(offset * tang_y);
        draw_rotated_char(fb, stride, ccx, ccy, text[i], text_rot, color, scale);
    }
}

/* Non-rotated text for panel (reuse from other hosts) */
static void draw_char_flat(uint32_t *fb, int x, int y, char c, uint32_t color)
{
    const uint8_t *bits = font_lookup(c);
    if (!bits) return;
    for (int row=0;row<FONT_H;row++)
        for (int col=0;col<FONT_W;col++)
            if (bits[row] & (0x10>>col))
                for (int sy=0;sy<2;sy++) for (int sx=0;sx<2;sx++){
                    int px=x+col*2+sx, py=y+row*2+sy;
                    if((unsigned)px<WIN_W&&(unsigned)py<WIN_H)
                        fb[py*WIN_W+px]=color;
                }
}
static void draw_str_flat(uint32_t *fb, int x, int y, const char *s, uint32_t color)
{ while(*s){draw_char_flat(fb,x,y,*s++,color);x+=12;} }
static void draw_str_cx_flat(uint32_t *fb, int cx, int y, const char *s, uint32_t color)
{ int tw=(int)strlen(s)*12-2; draw_str_flat(fb,cx-tw/2,y,s,color); }

/* ================================================================== */
/*  GUI helpers                                                       */
/* ================================================================== */

static void fill_rect(uint32_t *fb, int rx, int ry, int rw, int rh, uint32_t c)
{
    for(int y=ry;y<ry+rh;y++){if((unsigned)y>=WIN_H) continue;
    for(int x=rx;x<rx+rw;x++) if((unsigned)x<WIN_W) fb[y*WIN_W+x]=c;}
}

static void draw_arrow_btn(uint32_t *fb, int bx, int by, int bw, int bh,
                            int dir, uint32_t color)
{
    int pad=9,th=bh-2*pad,tw=bw-2*pad,half=th/2;
    if(th<2||tw<2)return;
    for(int row=0;row<th;row++){
        int w=tw*(half-abs(row-half))/half; if(w<1)w=1;
        int y=by+pad+row,x0=(dir<0)?bx+bw-pad-w:bx+pad;
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
        int g=i/2,is_r=i&1;
        int bx=is_r?BTN_R_X(g):BTN_L_X(g);
        if(mx>=bx&&mx<bx+BTN_W&&my>=BTN_Y&&my<BTN_Y+BTN_H) return i;
    }
    return -1;
}

/* ================================================================== */
/*  Non-linear airspeed scale                                         */
/* ================================================================== */

/* Angles in degrees CW from 12 o'clock.
 * Calibrated: 60kt=3°(12 o'clock), 200kt=180°(6 o'clock), 400kt=300°(10 o'clock) */
typedef struct { float kts; float deg; } AsiScale;
static const AsiScale asi_scale[] = {
    {  60,   3 }, {  80,  28 }, { 100,  55 },
    { 120,  80 }, { 140, 105 }, { 160, 130 },
    { 180, 155 }, { 200, 180 }, { 230, 213 },
    { 250, 235 }, { 300, 262 }, { 350, 282 },
    { 400, 300 }, { 450, 316 }, { 500, 330 },
};
#define ASI_N (sizeof(asi_scale)/sizeof(asi_scale[0]))

static float kts_to_deg(float kts)
{
    if (kts <= asi_scale[0].kts) return asi_scale[0].deg;
    if (kts >= asi_scale[ASI_N-1].kts) return asi_scale[ASI_N-1].deg;
    for (int i = 0; i < (int)ASI_N-1; i++) {
        if (kts >= asi_scale[i].kts && kts <= asi_scale[i+1].kts) {
            float t = (kts - asi_scale[i].kts) / (asi_scale[i+1].kts - asi_scale[i].kts);
            return asi_scale[i].deg + t * (asi_scale[i+1].deg - asi_scale[i].deg);
        }
    }
    return 0;
}

/* ================================================================== */
/*  Dial face rendering (pre-rendered once)                           */
/* ================================================================== */

static void draw_tick(uint32_t *fb, int stride, float angle_deg,
                       int r_inner, int r_outer, int width, uint32_t color)
{
    float rad = angle_deg * (float)M_PI / 180.0f;
    float sx = sinf(rad), cx = cosf(rad);
    for (int r = r_inner; r <= r_outer; r++) {
        int px_c = CX + (int)(r * sx + 0.5f);
        int py_c = CY - (int)(r * cx + 0.5f);
        for (int w = -width/2; w <= width/2; w++) {
            int px = px_c + (int)(w * cx + 0.5f);
            int py = py_c + (int)(w * sx + 0.5f);
            if ((unsigned)px < DISP_W && (unsigned)py < DISP_H)
                fb[py*stride+px] = color;
        }
    }
}

static void render_dial_face(uint32_t *fb, int stride)
{
    uint32_t white = 0xFFDDDDDDu;
    uint32_t gray  = 0xFFAAAAAAu;

    /* Black background circle */
    for (int y = 0; y < DISP_H; y++)
        for (int x = 0; x < DISP_W; x++) {
            float dx = x - CX, dy = y - CY;
            float dsq = dx*dx + dy*dy;
            if (dsq > (DIAL_R+20)*(DIAL_R+20))
                fb[y*stride+x] = 0xFF000000u;
            else if (dsq > (DIAL_R+5)*(DIAL_R+5))
                fb[y*stride+x] = 0xFF222222u;  /* bezel */
            else
                fb[y*stride+x] = 0xFF0A0A0Au;  /* face */
        }

    /* Draw tick marks.
     * Major and minor share same inner radius.
     * Major = 2x length, 2x width of minor.
     *
     * Graduation scheme (from the real instrument):
     *   60-80:  major at 60,80; minor every 5 kts (4 minors between)
     *   80-200: major every 10 kts; minor every 2 kts (4 minors between)
     *   200+:   major at labeled values; minor every 5 or 10 kts */

    /* Labeled major tick positions */
    static const int labels[] = {60,80,100,120,140,160,180,200,230,250,300,350,400};
    int n_labels = 13;

    /* Pass 1: Major ticks with numbers */
    for (int i = 0; i < n_labels; i++) {
        float deg = kts_to_deg((float)labels[i]);
        draw_tick(fb, stride, deg, TICK_INNER, TICK_INNER + TICK_MAJ, 2, white);
        char num[5];
        snprintf(num, sizeof(num), "%d", labels[i]);
        draw_dial_text(fb, stride, deg, NUM_R, num, white, 2);
    }

    /* Pass 2: Minor ticks */
    /* 60-80: every 5 kts (4 minors between majors) */
    for (int kts = 65; kts < 80; kts += 5) {
        float deg = kts_to_deg((float)kts);
        draw_tick(fb, stride, deg, TICK_INNER, TICK_INNER + TICK_MIN, 1, gray);
    }

    /* 80-230: unlabeled major ticks at every 10 kts (90,110,...,190,210,220) */
    for (int kts = 90; kts <= 220; kts += 10) {
        /* Skip if already labeled */
        int skip = 0;
        for (int i = 0; i < n_labels; i++)
            if (kts == labels[i]) { skip = 1; break; }
        if (skip) continue;
        float deg = kts_to_deg((float)kts);
        draw_tick(fb, stride, deg, TICK_INNER, TICK_INNER + TICK_MAJ, 2, white);
    }

    /* 80-230: minor ticks every 2 kts (4 minors between each 10-knot major) */
    for (int kts = 82; kts < 230; kts += 2) {
        if (kts % 10 == 0) continue;   /* skip major positions */
        float deg = kts_to_deg((float)kts);
        draw_tick(fb, stride, deg, TICK_INNER, TICK_INNER + TICK_MIN, 1, gray);
    }

    /* 230-250: major tick at 240, minors every 2 kts */
    {
        float deg = kts_to_deg(240.0f);
        draw_tick(fb, stride, deg, TICK_INNER, TICK_INNER + TICK_MAJ, 2, white);
    }
    for (int kts = 232; kts < 250; kts += 2) {
        if (kts % 10 == 0) continue;
        float deg = kts_to_deg((float)kts);
        draw_tick(fb, stride, deg, TICK_INNER, TICK_INNER + TICK_MIN, 1, gray);
    }

    /* 250-500: unlabeled major ticks at 450 and 500 */
    for (int kts = 450; kts <= 500; kts += 50) {
        float deg = kts_to_deg((float)kts);
        draw_tick(fb, stride, deg, TICK_INNER, TICK_INNER + TICK_MAJ, 2, white);
    }

    /* 250-500: minor ticks every 10 kts (4 minors between each 50-knot major) */
    for (int kts = 260; kts <= 490; kts += 10) {
        if (kts % 50 == 0) continue;   /* skip major positions */
        float deg = kts_to_deg((float)kts);
        draw_tick(fb, stride, deg, TICK_INNER, TICK_INNER + TICK_MIN, 1, gray);
    }

    /* "IAS" and "KT" labels */
    draw_dial_text(fb, stride, 160, 90, "IAS", gray, 2);
    draw_dial_text(fb, stride, 172, 90, "KT", gray, 2);
}

/* ================================================================== */
/*  Dynamic rendering (per frame)                                     */
/* ================================================================== */

/* Draw IAS pointer (white, thin) */
static void draw_ias_pointer(uint32_t *fb, int stride, float angle_deg)
{
    float rad = angle_deg * (float)M_PI / 180.0f;
    float sx = sinf(rad), cx = cosf(rad);
    uint32_t white = 0xFFEEEEEEu;

    /* Main shaft */
    for (int r = 30; r <= PTR_LEN; r++) {
        int px_c = CX + (int)(r * sx);
        int py_c = CY - (int)(r * cx);
        for (int w = -1; w <= 1; w++) {
            int px = px_c + (int)(w * cx);
            int py = py_c + (int)(w * sx);
            if ((unsigned)px < DISP_W && (unsigned)py < DISP_H)
                fb[py*stride+px] = white;
        }
    }

    /* Arrow tip */
    for (int r = PTR_LEN - 15; r <= PTR_LEN; r++) {
        float frac = (float)(r - (PTR_LEN-15)) / 15.0f;
        int half_w = (int)(6.0f * (1.0f - frac));
        int px_c = CX + (int)(r * sx);
        int py_c = CY - (int)(r * cx);
        for (int w = -half_w; w <= half_w; w++) {
            int px = px_c + (int)(w * cx);
            int py = py_c + (int)(w * sx);
            if ((unsigned)px < DISP_W && (unsigned)py < DISP_H)
                fb[py*stride+px] = white;
        }
    }
}

/* Draw VMO barber-pole pointer (red/white diagonal stripes) */
static void draw_vmo_pointer(uint32_t *fb, int stride, float angle_deg)
{
    float rad = angle_deg * (float)M_PI / 180.0f;
    float sx = sinf(rad), cx = cosf(rad);

    for (int r = 40; r <= PTR_LEN - 5; r++) {
        int px_c = CX + (int)(r * sx);
        int py_c = CY - (int)(r * cx);
        int half_w = 3;

        /* Taper at tip */
        if (r > PTR_LEN - 20) {
            float frac = (float)(PTR_LEN - 5 - r) / 15.0f;
            half_w = (int)(3.0f * frac) + 1;
        }

        for (int w = -half_w; w <= half_w; w++) {
            int px = px_c + (int)(w * cx);
            int py = py_c + (int)(w * sx);
            if ((unsigned)px >= DISP_W || (unsigned)py >= DISP_H) continue;

            /* Diagonal stripe pattern */
            int stripe = ((r + w + 1000) / 5) % 2;
            fb[py*stride+px] = stripe ? 0xFFDD2222u : 0xFFEEEEEEu;
        }
    }
}

/* Draw Mach drum display */
static void draw_mach_display(uint32_t *fb, int stride, float mach)
{
    /* Window position: top center of dial */
    int wx = CX - 30, wy = CY - 140;
    int ww = 60, wh = 32;

    /* Dark background */
    for (int y = wy; y < wy+wh; y++)
        for (int x = wx; x < wx+ww; x++)
            if ((unsigned)x < DISP_W && (unsigned)y < DISP_H)
                fb[y*stride+x] = 0xFF111111u;

    /* Border */
    for (int x = wx; x < wx+ww; x++) {
        if ((unsigned)x < DISP_W) {
            if ((unsigned)wy < DISP_H) fb[wy*stride+x] = 0xFF444444u;
            if ((unsigned)(wy+wh-1) < DISP_H) fb[(wy+wh-1)*stride+x] = 0xFF444444u;
        }
    }
    for (int y = wy; y < wy+wh; y++) {
        if ((unsigned)y < DISP_H) {
            if ((unsigned)wx < DISP_W) fb[y*stride+wx] = 0xFF444444u;
            if ((unsigned)(wx+ww-1) < DISP_W) fb[y*stride+wx+ww-1] = 0xFF444444u;
        }
    }

    /* Decimal point and digits */
    int m100 = (int)(mach * 100 + 0.5f);
    if (m100 < 10) m100 = 10;
    if (m100 > 99) m100 = 99;
    int d_tens = m100 / 10;
    int d_ones = m100 % 10;

    char txt[8];
    snprintf(txt, sizeof(txt), ".%d%d", d_tens, d_ones);
    /* Render centered in window */
    int tx = wx + 8;
    int ty = wy + 5;
    for (int i = 0; txt[i]; i++) {
        draw_rotated_char(fb, stride, tx + i*14, ty + 10, txt[i], 0, 0xFFDDDDDDu, 3);
    }

    /* "MACH" label below */
    draw_dial_text(fb, stride, 0, 105, "MACH", 0xFF999999u, 1);
}

/* Draw OFF / VMO flag */
static void draw_flag(uint32_t *fb, int stride, int off_flag, int vmo_warning)
{
    int fx = CX + 40, fy = CY + 50;
    int fw = 38, fh = 18;

    uint32_t bg, fg;
    const char *txt;

    if (vmo_warning) {
        bg = 0xFFDD2222u; txt = "VMO"; fg = 0xFFFFFFFFu;
    } else if (off_flag) {
        bg = 0xFFDD3300u; txt = "OFF"; fg = 0xFFFFFFFFu;
    } else {
        bg = 0xFF111111u; txt = ""; fg = 0xFF111111u;
    }

    for (int y = fy; y < fy+fh; y++)
        for (int x = fx; x < fx+fw; x++)
            if ((unsigned)x < DISP_W && (unsigned)y < DISP_H)
                fb[y*stride+x] = bg;

    if (txt[0]) {
        int len = (int)strlen(txt);
        int tx = fx + fw/2;
        int ty2 = fy + fh/2;
        float spacing = 8;
        for (int i = 0; i < len; i++) {
            int cx2 = tx + (int)((i - (len-1)/2.0f) * spacing);
            draw_rotated_char(fb, stride, cx2, ty2, txt[i], 0, fg, 1);
        }
    }
}

/* Draw center hub */
static void draw_hub(uint32_t *fb, int stride)
{
    for (int dy = -HUB_R; dy <= HUB_R; dy++)
        for (int dx = -HUB_R; dx <= HUB_R; dx++)
            if (dx*dx + dy*dy <= HUB_R*HUB_R) {
                int px = CX+dx, py = CY+dy;
                if ((unsigned)px < DISP_W && (unsigned)py < DISP_H) {
                    float shade = 0.08f + 0.07f * (1.0f - sqrtf(dx*dx+dy*dy)/(float)HUB_R);
                    uint8_t v = (uint8_t)(shade * 255);
                    fb[py*stride+px] = 0xFF000000u | (v<<16) | (v<<8) | v;
                }
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
    printf("=== ASI Host → %s:%d ===\n", pi_ip, PI_PORT);

    /* ---- Pre-render dial face ---- */
    uint32_t *dial_bg = calloc(DISP_W * DISP_H, 4);
    render_dial_face(dial_bg, DISP_W);
    printf("Dial face rendered\n");

    uint32_t *fb = calloc(WIN_W * WIN_H, 4);

    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window *win = SDL_CreateWindow("ASI Host",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIN_W, WIN_H, 0);
    SDL_Renderer *ren = SDL_CreateRenderer(win, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    SDL_RenderSetLogicalSize(ren, WIN_W, WIN_H);
    SDL_Texture *tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING, WIN_W, WIN_H);

    /* State */
    float ias = 140, mach = 0.15f, vmo = 340;
    int auto_anim = 1, off_flag = 0;
    int pressed_btn = -1;
    Uint32 press_start = 0;
    int initial_step_done = 0;
    Uint32 last_tick = SDL_GetTicks(), last_send = 0;

    static const char *labels[] = {"IAS", "MACH", "VMO"};

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
                case SDLK_r: ias=140; mach=0.15f; vmo=340; auto_anim=1; break;
                default: break;
                } break;
            case SDL_MOUSEBUTTONDOWN:
                if(ev.button.button==SDL_BUTTON_LEFT){
                    pressed_btn=hit_test(ev.button.x,ev.button.y);
                    press_start=now; initial_step_done=0;
                } break;
            case SDL_MOUSEBUTTONUP:
                if(ev.button.button==SDL_BUTTON_LEFT) pressed_btn=-1; break;
            }
        }

        /* Button actions */
        if (pressed_btn >= 0) {
            int dir=(pressed_btn&1)?1:-1, group=pressed_btn/2;
            float *target=NULL, step=0, cont=0;
            switch(group){
            case 0: target=&ias;  step=5; cont=60*dt; auto_anim=0; break;
            case 1: target=&mach; step=0.01f; cont=0.05f*dt; auto_anim=0; break;
            case 2: target=&vmo;  step=5; cont=60*dt; break;
            }
            if(target){
                if(!initial_step_done){*target+=dir*step;initial_step_done=1;}
                else if(now-press_start>400)*target+=dir*cont;
            }
        }

        /* Arrow keys */
        const Uint8 *keys = SDL_GetKeyboardState(NULL);
        if(keys[SDL_SCANCODE_UP])    { ias+=60*dt; auto_anim=0; }
        if(keys[SDL_SCANCODE_DOWN])  { ias-=60*dt; auto_anim=0; }
        if(keys[SDL_SCANCODE_LEFT])  vmo-=60*dt;
        if(keys[SDL_SCANCODE_RIGHT]) vmo+=60*dt;
        if(keys[SDL_SCANCODE_Q])     { mach-=0.05f*dt; auto_anim=0; }
        if(keys[SDL_SCANCODE_W])     { mach+=0.05f*dt; auto_anim=0; }

        /* Auto animation: takeoff to cruise */
        if (auto_anim) {
            float t = fmodf(now / 1000.0f, 30.0f);
            if (t < 15) {
                ias = 60 + t * 22;       /* 60→390 over 15s */
                mach = 0.10f + t * 0.056f;  /* .10→.94 */
            } else {
                ias = 390 - (t-15) * 22;
                mach = 0.94f - (t-15) * 0.056f;
            }
        }

        /* Clamp */
        if(ias<60) ias=60; if(ias>500) ias=500;
        if(mach<0.10f) mach=0.10f; if(mach>0.99f) mach=0.99f;
        if(vmo<100) vmo=100; if(vmo>400) vmo=400;

        /* VMO warning */
        int vmo_warn = (ias > vmo);

        /* Send UDP */
        if (now-last_send >= SEND_INTERVAL) {
            AsiPacket pkt = {INSTRUMENT_ASI, ias, mach, vmo};
            sendto(sock,(char*)&pkt,sizeof(pkt),0,
                   (struct sockaddr*)&pi_addr,sizeof(pi_addr));
            last_send = now;
        }

        /* ---- Render ---- */
        /* Copy pre-rendered dial to instrument area */
        for (int y = 0; y < DISP_H; y++)
            memcpy(&fb[y*WIN_W], &dial_bg[y*DISP_W], DISP_W*4);

        /* Dynamic elements */
        draw_mach_display(fb, WIN_W, mach);
        draw_flag(fb, WIN_W, off_flag, vmo_warn);
        draw_vmo_pointer(fb, WIN_W, kts_to_deg(vmo));
        draw_ias_pointer(fb, WIN_W, kts_to_deg(ias));
        draw_hub(fb, WIN_W);

        /* ---- Control panel ---- */
        fill_rect(fb, 0, DISP_H, WIN_W, PANEL_H, COL_PANEL_BG);
        fill_rect(fb, 0, DISP_H, WIN_W, 1, COL_SEPARATOR);
        for (int g=1; g<N_CONTROLS; g++)
            fill_rect(fb, g*GROUP_W, DISP_H+4, 1, PANEL_H-8, COL_SEPARATOR);

        for (int g=0; g<N_CONTROLS; g++) {
            int gcx = g*GROUP_W + GROUP_W/2;
            draw_str_cx_flat(fb, gcx, LABEL_Y, labels[g], COL_LABEL);
            char val[16];
            switch(g){
            case 0: snprintf(val,sizeof(val),"%03.0f",ias); break;
            case 1: snprintf(val,sizeof(val),".%02d",(int)(mach*100+0.5f)); break;
            case 2: snprintf(val,sizeof(val),"%03.0f",vmo); break;
            }
            draw_str_cx_flat(fb, gcx, VALUE_Y, val, COL_VALUE_G);
            draw_button(fb,BTN_L_X(g),BTN_Y,-1,hovered_btn==g*2,pressed_btn==g*2);
            draw_button(fb,BTN_R_X(g),BTN_Y, 1,hovered_btn==g*2+1,pressed_btn==g*2+1);
        }

        SDL_UpdateTexture(tex,NULL,fb,WIN_W*4);
        SDL_RenderCopy(ren,tex,NULL,NULL);
        SDL_RenderPresent(ren);
    }

    closesocket(sock); WSACleanup();
    free(dial_bg); free(fb);
    SDL_DestroyTexture(tex); SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win); SDL_Quit();
    return 0;
}
