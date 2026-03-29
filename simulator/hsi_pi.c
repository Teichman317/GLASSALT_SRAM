/*
 * HSI Receiver — runs on Raspberry Pi, receives UDP from host
 * 480x480 KI-525A HSI face, no control panel
 *
 * Usage: ./hsi_pi [port]
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

/* ---- UDP packet (must match hsi_host.c) ---- */
typedef struct {
    uint32_t instrument_id;     /* 3 = HSI */
    float    heading;           /* degrees, 0-360 */
    float    course;            /* degrees, 0-360 */
    float    heading_bug;       /* degrees, 0-360 */
    float    cdi_dev;           /* dots, -3 to +3 */
} HsiPacket;

#define INSTRUMENT_HSI  3

/* ---- Display geometry ---- */
#define W     480
#define H     480
#define CX    (W / 2)
#define CY    (H / 2)

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
    printf("  %s: offset(%d,%d) size(%dx%d)\n", path, s->ox, s->oy, s->w, s->h);
    return 0;
}

/* ================================================================== */
/*  Instrument compositing                                            */
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
    for (int i = 0; i < W * H; i++) {
        uint8_t l = bg[i];
        fb[i] = 0xFF000000u | ((uint32_t)l<<16) | ((uint32_t)l<<8) | l;
    }
}

static void rotate_blend_al44(uint32_t *fb, const uint8_t *src, float deg)
{
    float rad = deg * (float)M_PI / 180.0f;
    float cs = cosf(rad), sn = sinf(rad);
    for (int dy = 0; dy < H; dy++) {
        float fy = (float)(dy - CY);
        for (int dx = 0; dx < W; dx++) {
            float fx = (float)(dx - CX);
            int sx = (int)(CX + fx*cs + fy*sn + 0.5f);
            int sy = (int)(CY - fx*sn + fy*cs + 0.5f);
            if ((unsigned)sx >= W || (unsigned)sy >= H) continue;
            uint8_t p = src[sy * W + sx];
            uint8_t a4 = p >> 4;
            if (a4 == 0) continue;
            blend_pixel(&fb[dy*W+dx], (p&0xF)*17, (p&0xF)*17, (p&0xF)*17, a4*17);
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
    int mn_x=W, mn_y=H, mx_x=0, mx_y=0;
    for (int i=0; i<4; i++) {
        float fx=corners[i][0]-CX, fy=corners[i][1]-CY;
        int rx=(int)(CX+fx*cs-fy*sn), ry=(int)(CY+fx*sn+fy*cs);
        if(rx<mn_x)mn_x=rx; if(rx>mx_x)mx_x=rx;
        if(ry<mn_y)mn_y=ry; if(ry>mx_y)mx_y=ry;
    }
    if(mn_x<0)mn_x=0; else if(mn_x>2)mn_x-=2;
    if(mn_y<0)mn_y=0; else if(mn_y>2)mn_y-=2;
    if(mx_x>=W)mx_x=W-1; else mx_x+=2;
    if(mx_y>=H)mx_y=H-1; else mx_y+=2;
    if(mx_x>=W)mx_x=W-1;
    if(mx_y>=H)mx_y=H-1;

    for (int dy=mn_y; dy<=mx_y; dy++) {
        float fy=(float)(dy-CY);
        for (int dx=mn_x; dx<=mx_x; dx++) {
            float fx=(float)(dx-CX);
            float sx_f=CX+fx*cs+fy*sn-tx, sy_f=CY-fx*sn+fy*cs-ty;
            int lx=(int)(sx_f-s->ox+0.5f), ly=(int)(sy_f-s->oy+0.5f);
            if((unsigned)lx>=(unsigned)s->w||(unsigned)ly>=(unsigned)s->h) continue;
            uint16_t p=s->px[ly*s->w+lx];
            if(!(p&0x8000)) continue;
            fb[dy*W+dx] = 0xFF000000u |
                ((uint32_t)(((p>>10)&0x1F)<<3)<<16) |
                ((uint32_t)(((p>>5)&0x1F)<<3)<<8) |
                (uint32_t)((p&0x1F)<<3);
        }
    }
}

static void blit_sprite(uint32_t *fb, const Sprite *s)
{
    for (int y=0; y<s->h; y++) {
        int dy=s->oy+y; if((unsigned)dy>=H) continue;
        for (int x=0; x<s->w; x++) {
            int dx=s->ox+x; if((unsigned)dx>=W) continue;
            uint16_t p=s->px[y*s->w+x]; if(!(p&0x8000)) continue;
            fb[dy*W+dx] = 0xFF000000u |
                ((uint32_t)(((p>>10)&0x1F)<<3)<<16) |
                ((uint32_t)(((p>>5)&0x1F)<<3)<<8) |
                (uint32_t)((p&0x1F)<<3);
        }
    }
}

/* ================================================================== */
/*  Main                                                              */
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

    printf("=== HSI Receiver ===\n");
    printf("Listening on UDP port %d\n\n", port);

    /* ---- Load assets ---- */
    printf("Loading HSI layers from ../HSI/\n");

    uint8_t *bg_l8 = load_bin("../HSI/background.bin", W * H);
    if (!bg_l8) return 1;
    uint8_t *compass_al44 = load_bin("../HSI/compass_card.bin", W * H);
    if (!compass_al44) return 1;
    uint8_t *cdi_dots_al44 = load_bin("../HSI/cdi_dots.bin", W * H);
    if (!cdi_dots_al44) return 1;

    Sprite course_ptr, cdi_bar, hdg_bug, lubber;
    if (load_sprite("../HSI/course_pointer.bin", &course_ptr)) return 1;
    if (load_sprite("../HSI/cdi_bar.bin",        &cdi_bar))    return 1;
    if (load_sprite("../HSI/heading_bug.bin",    &hdg_bug))    return 1;
    if (load_sprite("../HSI/lubber_aircraft.bin", &lubber))     return 1;

    printf("All layers loaded OK\n");

    /* ---- Framebuffer ---- */
    uint32_t *fb = calloc(W * H, sizeof(uint32_t));
    if (!fb) return 1;

    /* ---- SDL ---- */
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "SDL_Init: %s\n", SDL_GetError()); return 1;
    }
    SDL_Window *win = SDL_CreateWindow("HSI",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, W, H, 0);
    SDL_Renderer *ren = SDL_CreateRenderer(win, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    SDL_RenderSetLogicalSize(ren, W, H);
    SDL_Texture *tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING, W, H);

    /* ---- Instrument state (updated by UDP) ---- */
    float heading     = 0.0f;
    float course      = 0.0f;
    float hdg_bug_deg = 0.0f;
    float cdi_dev     = 0.0f;     /* in pixels (dots * 25) */
    int   got_data    = 0;
    int   packets_rx  = 0;

    int running = 1;

    while (running) {
        /* ---- Events ---- */
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT) running = 0;
            if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_ESCAPE)
                running = 0;
        }

        /* ---- Receive UDP (drain all pending packets, keep latest) ---- */
        HsiPacket pkt;
        ssize_t n;
        while ((n = recvfrom(sock, &pkt, sizeof(pkt), 0, NULL, NULL)) > 0) {
            if (n == sizeof(pkt) && pkt.instrument_id == INSTRUMENT_HSI) {
                heading     = pkt.heading;
                course      = pkt.course;
                hdg_bug_deg = pkt.heading_bug;
                cdi_dev     = pkt.cdi_dev * 25.0f;   /* dots → pixels */
                got_data    = 1;
                packets_rx++;
            }
        }

        /* ---- Rotation angles ---- */
        float card_rot = -heading;
        float crs_rot  = course - heading;
        float bug_rot  = hdg_bug_deg - heading;

        /* ---- Composite 7 layers ---- */
        blit_background(fb, bg_l8);
        rotate_blend_al44(fb, cdi_dots_al44, crs_rot);
        rotate_blend_al44(fb, compass_al44, card_rot);
        rotate_blend_sprite(fb, &hdg_bug, bug_rot, 0, 0);
        rotate_blend_sprite(fb, &course_ptr, crs_rot, 0, 0);
        rotate_blend_sprite(fb, &cdi_bar, crs_rot, cdi_dev, 0);
        blit_sprite(fb, &lubber);

        /* ---- Present ---- */
        SDL_UpdateTexture(tex, NULL, fb, W * 4);
        SDL_RenderCopy(ren, tex, NULL, NULL);
        SDL_RenderPresent(ren);
    }

    close(sock);
    SDL_DestroyTexture(tex); SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win); SDL_Quit();
    free(fb); free(bg_l8); free(compass_al44); free(cdi_dots_al44);
    free(course_ptr.px); free(cdi_bar.px); free(hdg_bug.px); free(lubber.px);
    return 0;
}
