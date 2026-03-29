/*
 * Combined Instrument Panel — Raspberry Pi
 * Renders four 480x480 instruments in a 2x2 grid:
 *   [Altimeter] [HSI]
 *   [ADI      ] [VSI]
 *
 * Single UDP socket on port 5555 receives all instrument data.
 * Each instrument runs a demo animation until UDP data arrives.
 *
 * Usage: ./panel_pi [port]
 *        ESC to quit (requires keyboard) or kill from SSH
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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---- Layout ---- */
#define IW  480             /* instrument width/height */
#define IH  480
#define PW  (IW * 2)        /* panel: 960 x 960 (2x2 grid) */
#define PH  (IH * 2)

/* ---- UDP packets ---- */
#define ID_ALT  1
#define ID_VSI  2
#define ID_HSI  3
#define ID_ADI  4

/* ================================================================== */
/*  Shared helpers                                                    */
/* ================================================================== */

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
        dst[i] = (((p>>12)&0xF)*17u<<24) | (((p>>8)&0xF)*17u<<16) |
                 (((p>>4)&0xF)*17u<<8) | ((p&0xF)*17u);
    }
}

/* ================================================================== */
/*  ALTIMETER rendering                                               */
/* ================================================================== */

#define ALT_STRIP_W   28
#define ALT_STRIP_H   567
#define ALT_STRIP_DIG 10
#define ALT_DIGIT_H   ((float)ALT_STRIP_H / ALT_STRIP_DIG)
#define ALT_DRUM_X    164
#define ALT_DRUM_Y    174
#define ALT_DRUM_VH   104
#define ALT_1K_W      45
#define ALT_1K_FULL_H 583
#define ALT_1K_H      530
#define ALT_1K_HATCH  53
#define ALT_1K_DIGH   53.0f
#define ALT_1K_X      (ALT_DRUM_X - 10 - ALT_1K_W)
#define ALT_10K_X     (ALT_1K_X - ALT_1K_W)
#define ALT_1K_Y      (ALT_DRUM_Y + (ALT_DRUM_VH - 51) / 2)
#define ALT_1K_VH     51
#define ALT_BARO_Y    280
#define ALT_PTR_W     63
#define ALT_PTR_H     240
#define ALT_PIV_X     32
#define ALT_PIV_Y     50

static void alt_blit_drum(uint32_t *fb, const uint16_t *strip,
    int sw, int sh, float dh, int x, int y, int vh, float val)
{
    int ys = (int)(val*dh) - vh/2 + (int)(dh/2);
    for (int r=0; r<vh; r++) {
        int sy=ys+r; while(sy<0)sy+=sh; while(sy>=sh)sy-=sh;
        int by=y+r; if((unsigned)by>=IH) continue;
        for (int c=0; c<sw; c++) {
            int bx=x+c; if((unsigned)bx>=IW) continue;
            uint16_t p=strip[sy*sw+c];
            uint8_t cr=((p>>11)&0x1F)<<3; cr|=cr>>5;
            uint8_t cg=((p>>5)&0x3F)<<2;  cg|=cg>>6;
            uint8_t cb=(p&0x1F)<<3;        cb|=cb>>5;
            fb[by*IW+bx]=0xFF000000|(cr<<16)|(cg<<8)|cb;
        }
    }
}

static void alt_baro_drums(uint32_t *fb, const uint16_t *strip,
    int sw, int sh, float dh, int vh, float baro)
{
    float bv=baro*100;
    float d3=fmodf(bv,10), d2r=fmodf(bv/10,10), d1r=fmodf(bv/100,10), d0r=fmodf(bv/1000,10);
    float c3=d3>=9?d3-9:0; float d2=floorf(d2r)+c3;
    float c2=(d2>=9&&d2<10)?d2-9:0; float d1=floorf(d1r)+c2;
    float c1=(d1>=9&&d1<10)?d1-9:0; float d0=floorf(d0r)+c1;
    d0=fmodf(d0+10,10); d1=fmodf(d1+10,10); d2=fmodf(d2+10,10); d3=fmodf(d3+10,10);
    int x0=321-4*sw/2;
    alt_blit_drum(fb,strip,sw,sh,dh,x0,ALT_BARO_Y,vh,d0);
    alt_blit_drum(fb,strip,sw,sh,dh,x0+sw,ALT_BARO_Y,vh,d1);
    alt_blit_drum(fb,strip,sw,sh,dh,x0+2*sw,ALT_BARO_Y,vh,d2);
    alt_blit_drum(fb,strip,sw,sh,dh,x0+3*sw,ALT_BARO_Y,vh,d3);
}

static void alt_rotate_ptr(uint32_t *fb, const uint32_t *ptr, float rad)
{
    float cs=cosf(rad), sn=sinf(rad);
    int csx[4]={0,ALT_PTR_W-1,0,ALT_PTR_W-1}, csy[4]={0,0,ALT_PTR_H-1,ALT_PTR_H-1};
    int n0=IW,n1=0,m0=IH,m1=0;
    for(int i=0;i<4;i++){
        float fx=(float)(csx[i]-ALT_PIV_X), fy=(float)(csy[i]-ALT_PIV_Y);
        int dx=240+(int)(fx*cs-fy*sn), dy=240+(int)(fx*sn+fy*cs);
        if(dx<n0)n0=dx; if(dx>n1)n1=dx; if(dy<m0)m0=dy; if(dy>m1)m1=dy;
    }
    if(n0<0)n0=0; if(m0<0)m0=0; if(n1>=IW)n1=IW-1; if(m1>=IH)m1=IH-1;
    for(int dy=m0;dy<=m1;dy++) for(int dx=n0;dx<=n1;dx++){
        float fx=(float)(dx-240), fy=(float)(dy-240);
        int sx=ALT_PIV_X+(int)(fx*cs+fy*sn+0.5f);
        int sy=ALT_PIV_Y+(int)(-fx*sn+fy*cs+0.5f);
        if(sx<0||sx>=ALT_PTR_W||sy<0||sy>=ALT_PTR_H) continue;
        uint32_t sp=ptr[sy*ALT_PTR_W+sx]; uint8_t sa=(sp>>24)&0xFF;
        if(sa==0) continue;
        if(sa==255){fb[dy*IW+dx]=sp;continue;}
        uint32_t dp=fb[dy*IW+dx]; uint8_t ia=255-sa;
        fb[dy*IW+dx]=0xFF000000|
            ((((sp>>16&0xFF)*sa+(dp>>16&0xFF)*ia)/255)<<16)|
            ((((sp>>8&0xFF)*sa+(dp>>8&0xFF)*ia)/255)<<8)|
            (((sp&0xFF)*sa+(dp&0xFF)*ia)/255);
    }
}

/* ================================================================== */
/*  HSI rendering                                                     */
/* ================================================================== */

typedef struct { int ox,oy,w,h; uint16_t *px; } Sprite;

static int load_sprite(const char *path, Sprite *s)
{
    FILE *f=fopen(path,"rb"); if(!f){fprintf(stderr,"Cannot open %s\n",path);return -1;}
    uint16_t hdr[4]; if(fread(hdr,2,4,f)!=4){fclose(f);return -1;}
    s->ox=hdr[0]; s->oy=hdr[1]; s->w=hdr[2]; s->h=hdr[3];
    size_t n=(size_t)s->w*s->h; s->px=malloc(n*2);
    if(!s->px){fclose(f);return -1;}
    if(fread(s->px,2,n,f)!=n){free(s->px);fclose(f);return -1;}
    fclose(f); return 0;
}

static inline void hsi_blend(uint32_t *dst, uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    if(a==0) return;
    if(a==255){*dst=0xFF000000u|((uint32_t)r<<16)|((uint32_t)g<<8)|b;return;}
    uint32_t d=*dst;
    *dst=0xFF000000u|
        ((uint32_t)((r*a+((d>>16)&0xFF)*(255-a))/255)<<16)|
        ((uint32_t)((g*a+((d>>8)&0xFF)*(255-a))/255)<<8)|
        (uint32_t)((b*a+(d&0xFF)*(255-a))/255);
}

static void hsi_blit_bg(uint32_t *fb, const uint8_t *bg)
{
    for(int i=0;i<IW*IH;i++){
        uint8_t l=bg[i];
        fb[i]=0xFF000000u|((uint32_t)l<<16)|((uint32_t)l<<8)|l;
    }
}

static void hsi_rot_al44(uint32_t *fb, const uint8_t *src, float deg)
{
    float rad=deg*(float)M_PI/180.0f, cs=cosf(rad), sn=sinf(rad);
    for(int dy=0;dy<IH;dy++){
        float fy=(float)(dy-240);
        for(int dx=0;dx<IW;dx++){
            float fx=(float)(dx-240);
            int sx=(int)(240+fx*cs+fy*sn+0.5f);
            int sy=(int)(240-fx*sn+fy*cs+0.5f);
            if((unsigned)sx>=IW||(unsigned)sy>=IH) continue;
            uint8_t p=src[sy*IW+sx]; uint8_t a4=p>>4;
            if(a4==0) continue;
            hsi_blend(&fb[dy*IW+dx],(p&0xF)*17,(p&0xF)*17,(p&0xF)*17,a4*17);
        }
    }
}

static void hsi_rot_sprite(uint32_t *fb, const Sprite *s, float deg, float tx, float ty)
{
    float rad=deg*(float)M_PI/180.0f, cs=cosf(rad), sn=sinf(rad);
    float corners[4][2]={
        {s->ox+tx,s->oy+ty},{s->ox+s->w+tx,s->oy+ty},
        {s->ox+tx,s->oy+s->h+ty},{s->ox+s->w+tx,s->oy+s->h+ty}
    };
    int mn_x=IW,mn_y=IH,mx_x=0,mx_y=0;
    for(int i=0;i<4;i++){
        float fx=corners[i][0]-240, fy=corners[i][1]-240;
        int rx=(int)(240+fx*cs-fy*sn), ry=(int)(240+fx*sn+fy*cs);
        if(rx<mn_x)mn_x=rx; if(rx>mx_x)mx_x=rx;
        if(ry<mn_y)mn_y=ry; if(ry>mx_y)mx_y=ry;
    }
    if(mn_x<0)mn_x=0; else if(mn_x>2)mn_x-=2;
    if(mn_y<0)mn_y=0; else if(mn_y>2)mn_y-=2;
    if(mx_x>=IW)mx_x=IW-1; else mx_x+=2;
    if(mx_y>=IH)mx_y=IH-1; else mx_y+=2;
    if(mx_x>=IW)mx_x=IW-1; if(mx_y>=IH)mx_y=IH-1;

    for(int dy=mn_y;dy<=mx_y;dy++){
        float fy=(float)(dy-240);
        for(int dx=mn_x;dx<=mx_x;dx++){
            float fx=(float)(dx-240);
            float sxf=240+fx*cs+fy*sn-tx, syf=240-fx*sn+fy*cs-ty;
            int lx=(int)(sxf-s->ox+0.5f), ly=(int)(syf-s->oy+0.5f);
            if((unsigned)lx>=(unsigned)s->w||(unsigned)ly>=(unsigned)s->h) continue;
            uint16_t p=s->px[ly*s->w+lx]; if(!(p&0x8000)) continue;
            fb[dy*IW+dx]=0xFF000000u|
                ((uint32_t)(((p>>10)&0x1F)<<3)<<16)|
                ((uint32_t)(((p>>5)&0x1F)<<3)<<8)|
                (uint32_t)((p&0x1F)<<3);
        }
    }
}

static void hsi_blit_sprite(uint32_t *fb, const Sprite *s)
{
    for(int y=0;y<s->h;y++){
        int dy=s->oy+y; if((unsigned)dy>=IH) continue;
        for(int x=0;x<s->w;x++){
            int dx=s->ox+x; if((unsigned)dx>=IW) continue;
            uint16_t p=s->px[y*s->w+x]; if(!(p&0x8000)) continue;
            fb[dy*IW+dx]=0xFF000000u|
                ((uint32_t)(((p>>10)&0x1F)<<3)<<16)|
                ((uint32_t)(((p>>5)&0x1F)<<3)<<8)|
                (uint32_t)((p&0x1F)<<3);
        }
    }
}

/* ================================================================== */
/*  VSI rendering                                                     */
/* ================================================================== */

#define VSI_PTR_W   363
#define VSI_PTR_H   60
#define VSI_PIV_X   202
#define VSI_PIV_Y   30

typedef struct { float fpm; float deg; } VsiScale;
static const VsiScale vsi_scale[] = {
    {-2000,-151},{-1500,-127},{-1000,-90},{-500,-25},{0,0},
    {500,24},{1000,90},{1500,129},{2000,153},{2500,174},{3000,188},
};
#define VSI_SC_N (sizeof(vsi_scale)/sizeof(vsi_scale[0]))

static float vsi_fpm_to_deg(float fpm)
{
    if(fpm<=vsi_scale[0].fpm) return vsi_scale[0].deg;
    if(fpm>=vsi_scale[VSI_SC_N-1].fpm) return vsi_scale[VSI_SC_N-1].deg;
    for(int i=0;i<(int)VSI_SC_N-1;i++){
        if(fpm>=vsi_scale[i].fpm && fpm<=vsi_scale[i+1].fpm){
            float t=(fpm-vsi_scale[i].fpm)/(vsi_scale[i+1].fpm-vsi_scale[i].fpm);
            return vsi_scale[i].deg+t*(vsi_scale[i+1].deg-vsi_scale[i].deg);
        }
    }
    return 0;
}

static void vsi_rotate_ptr(uint32_t *fb, const uint32_t *ptr, float rad)
{
    float cs=cosf(rad), sn=sinf(rad);
    int csx[4]={0,VSI_PTR_W-1,0,VSI_PTR_W-1}, csy[4]={0,0,VSI_PTR_H-1,VSI_PTR_H-1};
    int n0=IW,n1=0,m0=IH,m1=0;
    for(int i=0;i<4;i++){
        float fx=(float)(csx[i]-VSI_PIV_X), fy=(float)(csy[i]-VSI_PIV_Y);
        int dx=240+(int)(fx*cs-fy*sn), dy=240+(int)(fx*sn+fy*cs);
        if(dx<n0)n0=dx; if(dx>n1)n1=dx; if(dy<m0)m0=dy; if(dy>m1)m1=dy;
    }
    if(n0<0)n0=0; if(m0<0)m0=0; if(n1>=IW)n1=IW-1; if(m1>=IH)m1=IH-1;
    for(int dy=m0;dy<=m1;dy++) for(int dx=n0;dx<=n1;dx++){
        float fx=(float)(dx-240), fy=(float)(dy-240);
        int sx=VSI_PIV_X+(int)(fx*cs+fy*sn+0.5f);
        int sy=VSI_PIV_Y+(int)(-fx*sn+fy*cs+0.5f);
        if(sx<0||sx>=VSI_PTR_W||sy<0||sy>=VSI_PTR_H) continue;
        uint32_t sp=ptr[sy*VSI_PTR_W+sx]; uint8_t sa=(sp>>24)&0xFF;
        if(sa==0) continue;
        if(sa==255){fb[dy*IW+dx]=sp;continue;}
        uint32_t dp=fb[dy*IW+dx]; uint8_t ia=255-sa;
        fb[dy*IW+dx]=0xFF000000|
            ((((sp>>16&0xFF)*sa+(dp>>16&0xFF)*ia)/255)<<16)|
            ((((sp>>8&0xFF)*sa+(dp>>8&0xFF)*ia)/255)<<8)|
            (((sp&0xFF)*sa+(dp&0xFF)*ia)/255);
    }
}

/* ================================================================== */
/*  ADI procedural rendering                                          */
/* ================================================================== */

#define ADI_R       190
#define ADI_PPD     7.0f
#define COL_SKY     0xFF1874CDu
#define COL_GND     0xFF8B5A2Bu
#define COL_WHITE   0xFFFFFFFFu
#define COL_BEZEL   0xFF222222u
#define COL_GOLD    0xFFFFD700u

static void adi_render(uint32_t *fb, float pitch, float roll)
{
    float roll_rad = roll * (float)M_PI / 180.0f;
    float pitch_rad = pitch * (float)M_PI / 180.0f;
    float cr = cosf(roll_rad), sr = sinf(roll_rad);
    float cp = cosf(pitch_rad), sp = sinf(pitch_rad);

    const float sky_r=0.094f, sky_g=0.455f, sky_b=0.804f;
    const float gnd_r=0.545f, gnd_g=0.353f, gnd_b=0.169f;

    float pitch_sin[10];
    for (int i=0; i<10; i++)
        pitch_sin[i] = sinf((float)((i/2+1)*5*((i%2)?-1:1)) * (float)M_PI/180.0f);

    float inv_r = 1.0f / (float)ADI_R;
    for (int py=0; py<IH; py++) {
        float dy=(float)(py-240);
        for (int px2=0; px2<IW; px2++) {
            float dx=(float)(px2-240);
            float dsq=dx*dx+dy*dy;
            if (dsq > (ADI_R+18)*(ADI_R+18)) { fb[py*IW+px2]=0xFF000000u; continue; }
            if (dsq > ADI_R*ADI_R) { fb[py*IW+px2]=COL_BEZEL; continue; }

            float u=dx*inv_r, v=-dy*inv_r;
            float z=sqrtf(1.0f-u*u-v*v);

            float u1= u*cr+v*sr;
            float v1=-u*sr+v*cr;
            float vy=v1*cp+z*sp;
            float vx=u1;
            float vz=-v1*sp+z*cp;

            float shade=0.35f+0.65f*z;
            float r,g,b;
            if (vy>0) {r=sky_r;g=sky_g;b=sky_b;} else {r=gnd_r;g=gnd_g;b=gnd_b;}

            if (fabsf(vy)<0.012f) { r=1;g=1;b=1; shade=(shade<0.7f)?0.7f:shade; }

            float lon=fabsf(atan2f(vx,vz));
            for (int i=0;i<10;i++) {
                float target=pitch_sin[i];
                int d5=(i/2+1)*5;
                float hw_r=(d5%10==0)?0.35f:0.20f;
                float th2=(d5%10==0)?0.008f:0.006f;
                if (fabsf(vy-target)<th2 && lon<hw_r) {
                    r=1;g=1;b=1; shade=(shade<0.7f)?0.7f:shade;
                }
                if (d5%10==0) {
                    float dvy=fabsf(vy-target);
                    if (dvy<0.04f&&dvy>=th2&&lon>hw_r-0.03f&&lon<hw_r+0.01f) {
                        int above=(target>0)?(vy<target):(vy>target);
                        if(above){r=1;g=1;b=1;shade=(shade<0.7f)?0.7f:shade;}
                    }
                }
            }

            uint8_t rb=(uint8_t)(r*shade*255);
            uint8_t gb2=(uint8_t)(g*shade*255);
            uint8_t bb=(uint8_t)(b*shade*255);
            fb[py*IW+px2]=0xFF000000u|((uint32_t)rb<<16)|((uint32_t)gb2<<8)|bb;
        }
    }

    /* Roll scale ticks */
    static const float ticks[]={0,10,20,30,45,60,-10,-20,-30,-45,-60};
    for (int i=0;i<11;i++) {
        float a_rad=(-90.0f+ticks[i])*(float)M_PI/180.0f;
        float ca=cosf(a_rad), sa=sinf(a_rad);
        int r0=ADI_R+2, r1=ADI_R+((ticks[i]==0)?18:(fabsf(ticks[i])<=30)?12:10);
        for(int r=r0;r<=r1;r++){
            int tx=(int)(240+r*ca+0.5f), ty=(int)(240+r*sa+0.5f);
            if((unsigned)tx<IW&&(unsigned)ty<IH) fb[ty*IW+tx]=COL_WHITE;
            if((unsigned)(tx+1)<IW&&(unsigned)ty<IH) fb[ty*IW+tx+1]=COL_WHITE;
        }
    }

    /* Roll pointer triangle */
    {
        float a_rad=(-90.0f+roll)*(float)M_PI/180.0f;
        float ca=cosf(a_rad), sa=sinf(a_rad);
        float pd=-sa, qd=ca;
        float tx2=240+(ADI_R+3)*ca, ty2=240+(ADI_R+3)*sa;
        float bx=240+(ADI_R+16)*ca, by=240+(ADI_R+16)*sa;
        float v[3][2]={{tx2,ty2},{bx+6*pd,by+6*qd},{bx-6*pd,by-6*qd}};
        int mn_x=(int)fminf(fminf(v[0][0],v[1][0]),v[2][0])-1;
        int mx_x=(int)fmaxf(fmaxf(v[0][0],v[1][0]),v[2][0])+1;
        int mn_y=(int)fminf(fminf(v[0][1],v[1][1]),v[2][1])-1;
        int mx_y=(int)fmaxf(fmaxf(v[0][1],v[1][1]),v[2][1])+1;
        for(int ty3=mn_y;ty3<=mx_y;ty3++){
            if((unsigned)ty3>=IH) continue;
            for(int tx3=mn_x;tx3<=mx_x;tx3++){
                if((unsigned)tx3>=IW) continue;
                float e0=(v[1][0]-v[0][0])*((float)ty3-v[0][1])-(v[1][1]-v[0][1])*((float)tx3-v[0][0]);
                float e1=(v[2][0]-v[1][0])*((float)ty3-v[1][1])-(v[2][1]-v[1][1])*((float)tx3-v[1][0]);
                float e2=(v[0][0]-v[2][0])*((float)ty3-v[2][1])-(v[0][1]-v[2][1])*((float)tx3-v[2][0]);
                if((e0>=0&&e1>=0&&e2>=0)||(e0<=0&&e1<=0&&e2<=0))
                    fb[ty3*IW+tx3]=COL_WHITE;
            }
        }
    }

    /* Aircraft symbol: dot + wings + tips */
    for(int dy=-4;dy<=4;dy++) for(int dx=-4;dx<=4;dx++)
        if(dx*dx+dy*dy<=16) fb[(240+dy)*IW+240+dx]=COL_GOLD;
    for(int x=-48;x<=48;x++){ if(abs(x)<7) continue;
        for(int t=-1;t<=1;t++) fb[(240+t)*IW+240+x]=COL_GOLD; }
    for(int y=0;y<=12;y++){
        fb[(240+y)*IW+240-48]=COL_GOLD; fb[(240+y)*IW+240-49]=COL_GOLD;
        fb[(240+y)*IW+240+48]=COL_GOLD; fb[(240+y)*IW+240+49]=COL_GOLD;
    }

    /* Fixed top index triangle */
    for(int row=0;row<8;row++){
        int hw=row;
        for(int dx=-hw;dx<=hw;dx++) fb[(240-ADI_R+8-row)*IW+240+dx]=COL_WHITE;
    }
}

/* ================================================================== */
/*  Main                                                              */
/* ================================================================== */
int main(int argc, char *argv[])
{
    int port = 5555;
    if (argc > 1) port = atoi(argv[1]);

    /* ---- UDP socket ---- */
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); return 1; }
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind"); close(sock); return 1;
    }
    fcntl(sock, F_SETFL, O_NONBLOCK);
    printf("=== Instrument Panel on port %d ===\n", port);

    /* ================================================================ */
    /*  Load all assets                                                 */
    /* ================================================================ */

    /* Altimeter */
    printf("Loading altimeter assets...\n");
    uint16_t *alt_bg565 = load_bin("../image.bin", IW*IH*2);
    uint16_t *alt_strip = load_bin("../100sWheel.bin", ALT_STRIP_W*ALT_STRIP_H*2);
    uint16_t *alt_ptr4444 = load_bin("../pointerargb4444.bin", ALT_PTR_W*ALT_PTR_H*2);
    uint16_t *alt_1k_full = load_bin("../1000sWheel.bin", ALT_1K_W*ALT_1K_FULL_H*2);
    uint16_t *alt_10k = load_bin("../10000sWheel.bin", ALT_1K_W*ALT_1K_H*2);
    if (!alt_bg565||!alt_strip||!alt_ptr4444||!alt_1k_full||!alt_10k) return 1;
    uint16_t *alt_1k = alt_1k_full + ALT_1K_W * ALT_1K_HATCH;

    /* Build baro strip */
    int orig_cell=(int)(ALT_DIGIT_H+0.5f), crop_l=2, crop_r=25;
    int baro_sw=crop_r-crop_l+1;
    int gt=orig_cell, gb=0;
    for(int d=0;d<ALT_STRIP_DIG;d++){
        int cy=(int)(d*ALT_DIGIT_H);
        for(int r=0;r<orig_cell&&(cy+r)<ALT_STRIP_H;r++){
            int has=0;
            for(int c=0;c<ALT_STRIP_W;c++){
                uint16_t p=alt_strip[(cy+r)*ALT_STRIP_W+c];
                int lum=((((p>>11)&0x1F)<<3)*77+(((p>>5)&0x3F)<<2)*150+((p&0x1F)<<3)*29)>>8;
                if(lum<128){has=1;break;}
            }
            if(has){if(r<gt)gt=r;if(r>gb)gb=r;}
        }
    }
    int gh=gb-gt+1, bcell=gh+10, baro_sh=bcell*ALT_STRIP_DIG;
    float baro_dh=(float)bcell;
    uint16_t *baro_strip=calloc(baro_sw*baro_sh,2);
    for(int d=0;d<ALT_STRIP_DIG;d++){
        int sy=(int)(d*ALT_DIGIT_H)+gt, dy2=d*bcell+5;
        for(int r=0;r<gh;r++){
            if(sy+r>=ALT_STRIP_H)break;
            for(int c=0;c<baro_sw;c++){
                uint16_t p=alt_strip[(sy+r)*ALT_STRIP_W+crop_l+c];
                int lum=((((p>>11)&0x1F)<<3)*77+(((p>>5)&0x3F)<<2)*150+((p&0x1F)<<3)*29)>>8;
                baro_strip[(dy2+r)*baro_sw+c]=(lum<128)?0xFFFF:0x0000;
            }
        }
    }

    uint32_t *alt_bg = malloc(IW*IH*4);
    uint32_t *alt_ptr = malloc(ALT_PTR_W*ALT_PTR_H*4);
    rgb565_to_argb8888(alt_bg565, alt_bg, IW*IH); free(alt_bg565);
    argb4444_to_argb8888(alt_ptr4444, alt_ptr, ALT_PTR_W*ALT_PTR_H); free(alt_ptr4444);

    /* HSI */
    printf("Loading HSI assets...\n");
    uint8_t *hsi_bg = load_bin("../HSI/background.bin", IW*IH);
    uint8_t *hsi_compass = load_bin("../HSI/compass_card.bin", IW*IH);
    uint8_t *hsi_dots = load_bin("../HSI/cdi_dots.bin", IW*IH);
    if (!hsi_bg||!hsi_compass||!hsi_dots) return 1;
    Sprite hsi_cptr, hsi_cbar, hsi_hbug, hsi_lubber;
    if (load_sprite("../HSI/course_pointer.bin", &hsi_cptr)) return 1;
    if (load_sprite("../HSI/cdi_bar.bin",        &hsi_cbar)) return 1;
    if (load_sprite("../HSI/heading_bug.bin",    &hsi_hbug)) return 1;
    if (load_sprite("../HSI/lubber_aircraft.bin", &hsi_lubber)) return 1;

    /* VSI */
    printf("Loading VSI assets...\n");
    uint16_t *vsi_bg565 = load_bin("../Vertical speed/vsi_bg.bin", IW*IH*2);
    uint16_t *vsi_ptr4444 = load_bin("../Vertical speed/vsi_pointer.bin", VSI_PTR_W*VSI_PTR_H*2);
    if (!vsi_bg565||!vsi_ptr4444) return 1;
    uint32_t *vsi_bg = malloc(IW*IH*4);
    uint32_t *vsi_ptr = malloc(VSI_PTR_W*VSI_PTR_H*4);
    rgb565_to_argb8888(vsi_bg565, vsi_bg, IW*IH); free(vsi_bg565);
    argb4444_to_argb8888(vsi_ptr4444, vsi_ptr, VSI_PTR_W*VSI_PTR_H); free(vsi_ptr4444);

    printf("All assets loaded OK\n");

    /* ---- Framebuffers ---- */
    uint32_t *fb_alt = malloc(IW*IH*4);
    uint32_t *fb_hsi = malloc(IW*IH*4);
    uint32_t *fb_vsi = malloc(IW*IH*4);
    uint32_t *fb_adi = malloc(IW*IH*4);
    uint32_t *fb_panel = calloc(PW*PH, 4);
    if (!fb_alt||!fb_hsi||!fb_vsi||!fb_adi||!fb_panel) return 1;

    /* ---- SDL ---- */
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window *win = SDL_CreateWindow("Instruments",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, PW, PH, 0);
    SDL_Renderer *ren = SDL_CreateRenderer(win, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    SDL_RenderSetLogicalSize(ren, PW, PH);
    SDL_Texture *tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING, PW, PH);

    /* ---- State ---- */
    /* Altimeter */
    float alt_altitude = 8000, alt_baro = 29.92f, alt_dir = 1;
    int alt_live = 0;
    /* HSI */
    float hsi_hdg = 0, hsi_crs = 45, hsi_bug = 90, hsi_dev = 0;
    int hsi_live = 0;
    /* VSI */
    float vsi_fpm = 0, vsi_dir = 1;
    int vsi_live = 0;
    /* ADI */
    float adi_pitch = 0, adi_roll = 0;
    int adi_live = 0;

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
        uint8_t buf[32];
        ssize_t n;
        while ((n = recvfrom(sock, buf, sizeof(buf), 0, NULL, NULL)) > 0) {
            if (n < 8) continue;
            uint32_t id;
            memcpy(&id, buf, 4);

            if (id == ID_ALT && n >= 12) {
                float vals[2];
                memcpy(vals, buf+4, 8);
                alt_altitude = vals[0];
                alt_baro = vals[1];
                alt_live = 1;
            }
            else if (id == ID_VSI && n >= 8) {
                float val;
                memcpy(&val, buf+4, 4);
                vsi_fpm = val;
                vsi_live = 1;
            }
            else if (id == ID_HSI && n >= 20) {
                float vals[4];
                memcpy(vals, buf+4, 16);
                hsi_hdg = vals[0];
                hsi_crs = vals[1];
                hsi_bug = vals[2];
                hsi_dev = vals[3] * 25.0f;  /* dots → pixels */
                hsi_live = 1;
            }
            else if (id == ID_ADI && n >= 12) {
                float vals[2];
                memcpy(vals, buf+4, 8);
                adi_pitch = vals[0];
                adi_roll = vals[1];
                adi_live = 1;
            }
        }

        /* ---- Demo animations (when no UDP) ---- */
        if (!alt_live) {
            alt_altitude += dt*200*alt_dir;
            if(alt_altitude>=12000){alt_altitude=12000;alt_dir=-1;}
            if(alt_altitude<=8000){alt_altitude=8000;alt_dir=1;}
        }
        if (!hsi_live) {
            hsi_hdg = fmodf(hsi_hdg + 6*dt, 360);
            float t = fmodf(now/1000.0f, 8.0f);
            hsi_dev = (t<4) ? -50+25*t : 150-25*t;
        }
        if (!vsi_live) {
            vsi_fpm += dt*300*vsi_dir;
            if(vsi_fpm>=2000){vsi_fpm=2000;vsi_dir=-1;}
            if(vsi_fpm<=-1500){vsi_fpm=-1500;vsi_dir=1;}
        }
        if (!adi_live) {
            float t = now / 1000.0f;
            adi_pitch = 8.0f * sinf(t*0.4f) + 3.0f * sinf(t*1.1f);
            adi_roll = 15.0f * sinf(t*0.25f) + 8.0f * sinf(t*0.7f);
        }

        /* ============================================================ */
        /*  Render Altimeter → fb_alt                                   */
        /* ============================================================ */
        memcpy(fb_alt, alt_bg, IW*IH*4);
        {
            float ptr_ang = (alt_altitude/1000) * 2 * (float)M_PI;
            float dv = fmodf(alt_altitude/100+5, 10);
            float d1kr = fmodf(alt_altitude/1000, 10);
            float w1k = fmodf(alt_altitude, 1000);
            float d1kv = (w1k<100&&alt_altitude>=100) ?
                fmodf(floorf(d1kr)-1+w1k/100+10, 10) : floorf(d1kr);
            float d10kr = fmodf(alt_altitude/10000, 10);
            float w10k = fmodf(alt_altitude, 10000);
            float d10kv = (w10k<100&&alt_altitude>=100) ?
                fmodf(floorf(d10kr)-1+w10k/100+10, 10) : floorf(d10kr);

            alt_blit_drum(fb_alt, alt_strip, ALT_STRIP_W, ALT_STRIP_H,
                          ALT_DIGIT_H, ALT_DRUM_X, ALT_DRUM_Y, ALT_DRUM_VH, dv);
            alt_blit_drum(fb_alt, alt_1k, ALT_1K_W, ALT_1K_H,
                          ALT_1K_DIGH, ALT_1K_X, ALT_1K_Y, ALT_1K_VH, d1kv);
            alt_blit_drum(fb_alt, alt_10k, ALT_1K_W, ALT_1K_H,
                          ALT_1K_DIGH, ALT_10K_X, ALT_1K_Y, ALT_1K_VH, d10kv);
            int baro_vis = (int)(baro_dh * 1.1f);
            alt_baro_drums(fb_alt, baro_strip, baro_sw, baro_sh, baro_dh,
                           baro_vis, alt_baro);
            alt_rotate_ptr(fb_alt, alt_ptr, ptr_ang);
        }

        /* ============================================================ */
        /*  Render HSI → fb_hsi                                         */
        /* ============================================================ */
        {
            float card_rot = -hsi_hdg;
            float crs_rot = hsi_crs - hsi_hdg;
            float bug_rot = hsi_bug - hsi_hdg;

            hsi_blit_bg(fb_hsi, hsi_bg);
            hsi_rot_al44(fb_hsi, hsi_dots, crs_rot);
            hsi_rot_al44(fb_hsi, hsi_compass, card_rot);
            hsi_rot_sprite(fb_hsi, &hsi_hbug, bug_rot, 0, 0);
            hsi_rot_sprite(fb_hsi, &hsi_cptr, crs_rot, 0, 0);
            hsi_rot_sprite(fb_hsi, &hsi_cbar, crs_rot, hsi_dev, 0);
            hsi_blit_sprite(fb_hsi, &hsi_lubber);
        }

        /* ============================================================ */
        /*  Render VSI → fb_vsi                                         */
        /* ============================================================ */
        memcpy(fb_vsi, vsi_bg, IW*IH*4);
        {
            float deg = vsi_fpm_to_deg(vsi_fpm);
            float rad = deg * (float)M_PI / 180.0f;
            vsi_rotate_ptr(fb_vsi, vsi_ptr, rad);
        }

        /* ============================================================ */
        /*  Render ADI → fb_adi                                         */
        /* ============================================================ */
        adi_render(fb_adi, adi_pitch, adi_roll);

        /* ============================================================ */
        /*  Assemble 2x2 panel:  [ALT] [HSI]                           */
        /*                       [ADI] [VSI]                            */
        /* ============================================================ */
        for (int y = 0; y < IH; y++) {
            /* Top row */
            memcpy(&fb_panel[y*PW + 0],  &fb_alt[y*IW], IW*4);
            memcpy(&fb_panel[y*PW + IW], &fb_hsi[y*IW], IW*4);
            /* Bottom row */
            memcpy(&fb_panel[(y+IH)*PW + 0],  &fb_adi[y*IW], IW*4);
            memcpy(&fb_panel[(y+IH)*PW + IW], &fb_vsi[y*IW], IW*4);
        }

        /* ---- Present ---- */
        SDL_UpdateTexture(tex, NULL, fb_panel, PW*4);
        SDL_RenderCopy(ren, tex, NULL, NULL);
        SDL_RenderPresent(ren);
    }

    close(sock);
    free(fb_alt); free(fb_hsi); free(fb_vsi); free(fb_adi); free(fb_panel);
    free(alt_bg); free(alt_ptr); free(alt_strip); free(alt_1k_full);
    free(alt_10k); free(baro_strip);
    free(hsi_bg); free(hsi_compass); free(hsi_dots);
    free(hsi_cptr.px); free(hsi_cbar.px); free(hsi_hbug.px); free(hsi_lubber.px);
    free(vsi_bg); free(vsi_ptr);
    SDL_DestroyTexture(tex); SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win); SDL_Quit();
    return 0;
}
