/*
 * GLASSALT VSI Receiver — Raspberry Pi graphical instrument
 * Listens for UDP packets from the host simulator and renders
 * the VSI dial + rotating pointer directly to the HDMI framebuffer
 * via Linux DRM/KMS (no X11 required).
 *
 * Build on Pi:
 *   gcc -o vsi_receiver vsi_receiver.c -lm -ldrm -I/usr/include/libdrm
 *
 * Usage: sudo ./vsi_receiver [path_to_assets]
 *   Assets needed: vsi_bg.bin (480x480 RGB565)
 *                  vsi_pointer.bin (363x60 ARGB4444)
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#define LISTEN_PORT  5555

#define IMG_WIDTH   480
#define IMG_HEIGHT  480

#define PTR_WIDTH   363
#define PTR_HEIGHT  60
#define PIVOT_X     202
#define PIVOT_Y     30
#define CX  240
#define CY  240

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* UDP packet — must match host */
typedef struct {
    uint32_t instrument_id;
    float    value;
} SimPacket;

#define INSTRUMENT_VSI  2

/* --- Calibrated VSI scale table --- */
typedef struct { float fpm; float deg; } ScalePoint;
static const ScalePoint vsi_scale[] = {
    { -2500, -151.0f }, { -2000, -127.0f }, { -1500, -90.0f },
    { -1000, -57.5f },  { -500, -25.0f },   { 0, 0.0f },
    { 500, 24.0f },     { 1000, 57.0f },    { 1500, 90.0f },
    { 2000, 129.0f },   { 2500, 153.0f },   { 3000, 174.0f },
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

/* --- Asset conversion --- */
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

/* --- Pointer rotation with alpha blending --- */
static void rotate_pointer(uint32_t *fb, int fb_stride,
                           const uint32_t *ptr, float angle_rad,
                           int ox, int oy)
{
    float cosA = cosf(angle_rad);
    float sinA = sinf(angle_rad);

    /* Bounding box from corners */
    int corners_sx[4] = {0, PTR_WIDTH-1, 0, PTR_WIDTH-1};
    int corners_sy[4] = {0, 0, PTR_HEIGHT-1, PTR_HEIGHT-1};
    int nx0 = fb_stride, ny0 = fb_stride, nx1 = 0, ny1 = 0;
    for (int c = 0; c < 4; c++) {
        float fx = (float)(corners_sx[c] - PIVOT_X);
        float fy = (float)(corners_sy[c] - PIVOT_Y);
        int dx = ox + (int)(fx * cosA - fy * sinA);
        int dy = oy + (int)(fx * sinA + fy * cosA);
        if (dx < nx0) nx0 = dx; if (dx > nx1) nx1 = dx;
        if (dy < ny0) ny0 = dy; if (dy > ny1) ny1 = dy;
    }
    if (nx0 < 0) nx0 = 0; if (ny0 < 0) ny0 = 0;
    if (nx1 >= fb_stride) nx1 = fb_stride - 1;
    if (ny1 >= fb_stride) ny1 = fb_stride - 1;

    for (int dy = ny0; dy <= ny1; dy++) {
        for (int dx = nx0; dx <= nx1; dx++) {
            float fx = (float)(dx - ox);
            float fy = (float)(dy - oy);
            int sx = PIVOT_X + (int)(fx * cosA + fy * sinA + 0.5f);
            int sy = PIVOT_Y + (int)(-fx * sinA + fy * cosA + 0.5f);
            if (sx < 0 || sx >= PTR_WIDTH || sy < 0 || sy >= PTR_HEIGHT) continue;
            uint32_t sp = ptr[sy * PTR_WIDTH + sx];
            uint8_t sa = (sp >> 24) & 0xFF;
            if (sa == 0) continue;
            if (sa == 255) {
                fb[dy * fb_stride + dx] = sp;
            } else {
                uint32_t dp = fb[dy * fb_stride + dx];
                uint8_t ia = 255 - sa;
                uint8_t or_ = ((sp>>16&0xFF)*sa + (dp>>16&0xFF)*ia) / 255;
                uint8_t og  = ((sp>>8&0xFF)*sa  + (dp>>8&0xFF)*ia)  / 255;
                uint8_t ob  = ((sp&0xFF)*sa     + (dp&0xFF)*ia)     / 255;
                fb[dy * fb_stride + dx] = 0xFF000000 | (or_<<16) | (og<<8) | ob;
            }
        }
    }
}

/* --- Tiny 5x7 font for FPM readout --- */
static const uint8_t font5x7[][7] = {
    /* space */ {0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    /* 0 */  {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E},
    /* 1 */  {0x04,0x06,0x04,0x04,0x04,0x04,0x0E},
    /* 2 */  {0x0E,0x11,0x10,0x08,0x04,0x02,0x1F},
    /* 3 */  {0x0E,0x11,0x10,0x0C,0x10,0x11,0x0E},
    /* 4 */  {0x08,0x0C,0x0A,0x09,0x1F,0x08,0x08},
    /* 5 */  {0x1F,0x01,0x0F,0x10,0x10,0x11,0x0E},
    /* 6 */  {0x0C,0x02,0x01,0x0F,0x11,0x11,0x0E},
    /* 7 */  {0x1F,0x10,0x08,0x04,0x02,0x02,0x02},
    /* 8 */  {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E},
    /* 9 */  {0x0E,0x11,0x11,0x1E,0x10,0x08,0x06},
    /* + */  {0x00,0x04,0x04,0x1F,0x04,0x04,0x00},
    /* - */  {0x00,0x00,0x00,0x1F,0x00,0x00,0x00},
    /* F */  {0x1F,0x01,0x01,0x0F,0x01,0x01,0x01},
    /* P */  {0x0F,0x11,0x11,0x0F,0x01,0x01,0x01},
    /* M */  {0x11,0x1B,0x15,0x15,0x11,0x11,0x11},
};

static int font_index(char c)
{
    if (c >= '0' && c <= '9') return 1 + (c - '0');
    if (c == '+') return 11;
    if (c == '-') return 12;
    if (c == 'F') return 13;
    if (c == 'P') return 14;
    if (c == 'M') return 15;
    if (c == ' ') return 0;
    return 0;
}

static void draw_text(uint32_t *fb, int stride, int fb_h,
                      int x, int y, const char *str, int scale, uint32_t color)
{
    for (int ci = 0; str[ci]; ci++) {
        int gi = font_index(str[ci]);
        for (int row = 0; row < 7; row++) {
            uint8_t bits = font5x7[gi][row];
            for (int col = 0; col < 5; col++) {
                if (bits & (0x10 >> col)) {
                    for (int sy = 0; sy < scale; sy++) {
                        for (int sx = 0; sx < scale; sx++) {
                            int px = x + ci * 6 * scale + col * scale + sx;
                            int py = y + row * scale + sy;
                            if (px >= 0 && px < stride && py >= 0 && py < fb_h)
                                fb[py * stride + px] = color;
                        }
                    }
                }
            }
        }
    }
}

/* --- DRM/KMS framebuffer setup --- */
struct drm_fb {
    int drm_fd;
    uint32_t fb_id;
    uint32_t handle;
    uint32_t *map;
    uint32_t width, height, stride, size;
    uint32_t conn_id;
    uint32_t crtc_id;
    drmModeModeInfo mode;
    drmModeCrtc *saved_crtc;
};

static volatile int running = 1;
static void sig_handler(int sig) { (void)sig; running = 0; }

static int drm_setup(struct drm_fb *dfb)
{
    dfb->drm_fd = open("/dev/dri/card1", O_RDWR);
    if (dfb->drm_fd < 0) {
        dfb->drm_fd = open("/dev/dri/card0", O_RDWR);
        if (dfb->drm_fd < 0) {
            perror("open /dev/dri/card*");
            return -1;
        }
    }

    drmModeRes *res = drmModeGetResources(dfb->drm_fd);
    if (!res) {
        /* Try the other card */
        close(dfb->drm_fd);
        dfb->drm_fd = open("/dev/dri/card0", O_RDWR);
        if (dfb->drm_fd < 0) { perror("open card0"); return -1; }
        res = drmModeGetResources(dfb->drm_fd);
        if (!res) { fprintf(stderr, "drmModeGetResources failed\n"); return -1; }
    }

    /* Find the first connected connector */
    drmModeConnector *conn = NULL;
    for (int i = 0; i < res->count_connectors; i++) {
        conn = drmModeGetConnector(dfb->drm_fd, res->connectors[i]);
        if (conn && conn->connection == DRM_MODE_CONNECTED && conn->count_modes > 0)
            break;
        drmModeFreeConnector(conn);
        conn = NULL;
    }
    if (!conn) {
        fprintf(stderr, "No connected display found\n");
        drmModeFreeResources(res);
        return -1;
    }

    dfb->conn_id = conn->connector_id;
    dfb->mode = conn->modes[0];  /* Use preferred (first) mode */
    dfb->width = dfb->mode.hdisplay;
    dfb->height = dfb->mode.vdisplay;

    printf("Display: %s %dx%d @ %dHz\n", conn->modes[0].name,
           dfb->width, dfb->height, dfb->mode.vrefresh);

    /* Find encoder and CRTC */
    drmModeEncoder *enc = NULL;
    if (conn->encoder_id) {
        enc = drmModeGetEncoder(dfb->drm_fd, conn->encoder_id);
    }
    if (enc) {
        dfb->crtc_id = enc->crtc_id;
        drmModeFreeEncoder(enc);
    } else {
        /* Pick first available CRTC */
        for (int i = 0; i < res->count_crtcs; i++) {
            dfb->crtc_id = res->crtcs[i];
            break;
        }
    }

    dfb->saved_crtc = drmModeGetCrtc(dfb->drm_fd, dfb->crtc_id);

    drmModeFreeConnector(conn);
    drmModeFreeResources(res);

    /* Create dumb buffer */
    struct drm_mode_create_dumb creq = {0};
    creq.width = dfb->width;
    creq.height = dfb->height;
    creq.bpp = 32;
    if (drmIoctl(dfb->drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq) < 0) {
        perror("DRM_IOCTL_MODE_CREATE_DUMB");
        return -1;
    }
    dfb->handle = creq.handle;
    dfb->stride = creq.pitch;
    dfb->size = creq.size;

    /* Create framebuffer object */
    if (drmModeAddFB(dfb->drm_fd, dfb->width, dfb->height, 24, 32,
                     dfb->stride, dfb->handle, &dfb->fb_id) < 0) {
        perror("drmModeAddFB");
        return -1;
    }

    /* Memory-map it */
    struct drm_mode_map_dumb mreq = {0};
    mreq.handle = dfb->handle;
    if (drmIoctl(dfb->drm_fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq) < 0) {
        perror("DRM_IOCTL_MODE_MAP_DUMB");
        return -1;
    }
    dfb->map = mmap(0, dfb->size, PROT_READ | PROT_WRITE, MAP_SHARED,
                    dfb->drm_fd, mreq.offset);
    if (dfb->map == MAP_FAILED) {
        perror("mmap");
        return -1;
    }

    /* Set the mode */
    if (drmModeSetCrtc(dfb->drm_fd, dfb->crtc_id, dfb->fb_id, 0, 0,
                       &dfb->conn_id, 1, &dfb->mode) < 0) {
        perror("drmModeSetCrtc");
        return -1;
    }

    return 0;
}

static void drm_cleanup(struct drm_fb *dfb)
{
    if (dfb->saved_crtc) {
        drmModeSetCrtc(dfb->drm_fd, dfb->saved_crtc->crtc_id,
                       dfb->saved_crtc->buffer_id,
                       dfb->saved_crtc->x, dfb->saved_crtc->y,
                       &dfb->conn_id, 1, &dfb->saved_crtc->mode);
        drmModeFreeCrtc(dfb->saved_crtc);
    }
    munmap(dfb->map, dfb->size);
    struct drm_mode_destroy_dumb dreq = {0};
    dreq.handle = dfb->handle;
    drmIoctl(dfb->drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
    close(dfb->drm_fd);
}

/* --- File loader --- */
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

int main(int argc, char *argv[])
{
    const char *asset_dir = ".";
    if (argc > 1) asset_dir = argv[1];

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    /* --- Load assets --- */
    char path[256];

    snprintf(path, sizeof(path), "%s/vsi_bg.bin", asset_dir);
    uint16_t *bg565 = load_bin(path, IMG_WIDTH * IMG_HEIGHT * 2);
    if (!bg565) return 1;

    snprintf(path, sizeof(path), "%s/vsi_pointer.bin", asset_dir);
    uint16_t *ptr4444 = load_bin(path, PTR_WIDTH * PTR_HEIGHT * 2);
    if (!ptr4444) return 1;

    /* Convert to ARGB8888 */
    uint32_t *bg_clean = malloc(IMG_WIDTH * IMG_HEIGHT * 4);
    uint32_t *ptr8888 = malloc(PTR_WIDTH * PTR_HEIGHT * 4);
    if (!bg_clean || !ptr8888) { fprintf(stderr, "malloc failed\n"); return 1; }

    rgb565_to_argb8888(bg565, bg_clean, IMG_WIDTH * IMG_HEIGHT);
    free(bg565);
    argb4444_to_argb8888(ptr4444, ptr8888, PTR_WIDTH * PTR_HEIGHT);
    free(ptr4444);

    /* Working framebuffer for compositing (480x480) */
    uint32_t *render_fb = malloc(IMG_WIDTH * IMG_HEIGHT * 4);
    if (!render_fb) { fprintf(stderr, "malloc failed\n"); return 1; }

    /* --- Setup DRM display --- */
    struct drm_fb dfb = {0};
    if (drm_setup(&dfb) < 0) {
        fprintf(stderr, "Failed to setup DRM display\n");
        return 1;
    }

    printf("Framebuffer: %dx%d stride=%d\n", dfb.width, dfb.height, dfb.stride);

    /* Center the 480x480 instrument on screen */
    int offset_x = ((int)dfb.width - IMG_WIDTH) / 2;
    int offset_y = ((int)dfb.height - IMG_HEIGHT) / 2;
    if (offset_x < 0) offset_x = 0;
    if (offset_y < 0) offset_y = 0;

    /* Clear the full screen to black */
    memset(dfb.map, 0, dfb.size);

    /* --- Setup UDP socket --- */
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); drm_cleanup(&dfb); return 1; }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(LISTEN_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(sock);
        drm_cleanup(&dfb);
        return 1;
    }

    /* Non-blocking so we can render continuously */
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    printf("=== GLASSALT VSI Receiver (graphical) ===\n");
    printf("Listening on UDP port %d\n", LISTEN_PORT);
    printf("Display: %dx%d, instrument at (%d,%d)\n",
           dfb.width, dfb.height, offset_x, offset_y);
    printf("Press Ctrl+C to quit\n\n");

    float vsi_fpm = 0.0f;
    float display_fpm = 0.0f;  /* smoothed for display */
    int connected = 0;
    int fb_stride_px = dfb.stride / 4;  /* stride in pixels */

    while (running) {
        /* Drain all pending UDP packets, keep latest */
        SimPacket pkt;
        struct sockaddr_in from_addr;
        socklen_t from_len = sizeof(from_addr);
        int got_packet = 0;

        while (1) {
            ssize_t n = recvfrom(sock, &pkt, sizeof(pkt), 0,
                                 (struct sockaddr *)&from_addr, &from_len);
            if (n != sizeof(pkt)) break;
            if (pkt.instrument_id == INSTRUMENT_VSI) {
                vsi_fpm = pkt.value;
                got_packet = 1;
                if (!connected) {
                    char sender_ip[INET_ADDRSTRLEN];
                    inet_ntop(AF_INET, &from_addr.sin_addr, sender_ip, sizeof(sender_ip));
                    printf("Connected! Receiving from %s\n", sender_ip);
                    connected = 1;
                }
            }
        }

        /* Smooth the pointer movement */
        float diff = vsi_fpm - display_fpm;
        display_fpm += diff * 0.15f;
        if (fabsf(diff) < 0.5f) display_fpm = vsi_fpm;

        /* --- Render instrument into working buffer --- */
        memcpy(render_fb, bg_clean, IMG_WIDTH * IMG_HEIGHT * 4);

        float angle_deg = vsi_fpm_to_deg(display_fpm);
        float angle_rad = angle_deg * (float)M_PI / 180.0f;
        rotate_pointer(render_fb, IMG_WIDTH, ptr8888, angle_rad, CX, CY);

        /* Draw digital FPM readout */
        char fpm_str[16];
        snprintf(fpm_str, sizeof(fpm_str), "%+.0f FPM", display_fpm);
        draw_text(render_fb, IMG_WIDTH, IMG_HEIGHT,
                  IMG_WIDTH/2 - 60, IMG_HEIGHT - 30, fpm_str, 2, 0xFF00FF00);

        /* --- Blit to DRM framebuffer --- */
        for (int y = 0; y < IMG_HEIGHT && (y + offset_y) < (int)dfb.height; y++) {
            uint32_t *dst = dfb.map + (y + offset_y) * fb_stride_px + offset_x;
            uint32_t *src = render_fb + y * IMG_WIDTH;
            int copy_w = IMG_WIDTH;
            if (offset_x + copy_w > (int)dfb.width)
                copy_w = (int)dfb.width - offset_x;
            memcpy(dst, src, copy_w * 4);
        }

        /* ~30 fps */
        usleep(33333);
    }

    printf("\nShutting down...\n");
    close(sock);
    drm_cleanup(&dfb);
    free(bg_clean);
    free(ptr8888);
    free(render_fb);
    return 0;
}
