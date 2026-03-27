/*
 * GLASSALT Altimeter Receiver — Raspberry Pi graphical instrument
 * Listens for UDP altitude packets from the host simulator and renders
 * the AAU-19/A altimeter directly to the HDMI framebuffer via DRM/KMS.
 *
 * Build on Pi:
 *   gcc -o alt_receiver alt_receiver.c -lm -ldrm -I/usr/include/libdrm
 *
 * Usage: sudo ./alt_receiver [asset_dir]
 *   Assets needed in asset_dir:
 *     image.bin           (480x480 RGB565)  — altimeter background
 *     pointerargb4444.bin (63x240 ARGB4444) — pointer
 *     100sWheel.bin       (28x567 RGB565)   — 100s drum strip
 *     1000sWheel.bin      (45x583 RGB565)   — 1000s drum strip (with hatch)
 *     10000sWheel.bin     (45x530 RGB565)   — 10000s drum strip
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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* UDP packet — must match host */
typedef struct {
    uint32_t instrument_id;
    float    value;
} SimPacket;

#define INSTRUMENT_ALT  1
#define INSTRUMENT_VSI  2

/* --- Altimeter pointer --- */
#define PTR_WIDTH   63
#define PTR_HEIGHT  240
#define PIVOT_X     32
#define PIVOT_Y     50
#define CX  240
#define CY  240

/* --- 100s drum --- */
#define STRIP_WIDTH   28
#define STRIP_HEIGHT  567
#define STRIP_DIGITS  10
#define DIGIT_HEIGHT  ((float)STRIP_HEIGHT / STRIP_DIGITS)  /* 56.7 */
#define DRUM_X        164
#define DRUM_Y        174
#define DRUM_VIS_H    104

/* --- 1000s drum --- */
#define STRIP1K_WIDTH   45
#define STRIP1K_FULL_H  583
#define STRIP1K_HEIGHT  530
#define STRIP1K_HATCH   53
#define STRIP1K_DIGITS  10
#define DIGIT1K_HEIGHT  53.0f
#define DRUM1K_VIS_H    51
#define DRUM1K_X        (DRUM_X - 10 - STRIP1K_WIDTH)
#define DRUM10K_X       (DRUM1K_X - STRIP1K_WIDTH)
#define DRUM1K_Y        (DRUM_Y + (DRUM_VIS_H - DRUM1K_VIS_H) / 2)

/* --- Baro drums --- */
#define BARO_GAP      0
#define BARO_Y        280
#define BARO_VIS_H    104

/* --- Format conversion --- */
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

/* --- Drum blit (RGB565 strip onto ARGB8888 framebuffer) --- */
static void blit_drum(uint32_t *fb, const uint16_t *strip,
                      int strip_w, int strip_h, float digit_h,
                      int x, int y, int vis_h, float value)
{
    float y_offset_f = value * digit_h;
    int y_offset = (int)y_offset_f;
    int strip_y_start = y_offset - vis_h / 2 + (int)(digit_h / 2);

    for (int row = 0; row < vis_h; row++) {
        int sy = strip_y_start + row;
        while (sy < 0)        sy += strip_h;
        while (sy >= strip_h) sy -= strip_h;

        int bg_y = y + row;
        if (bg_y < 0 || bg_y >= IMG_HEIGHT) continue;

        for (int col = 0; col < strip_w; col++) {
            int bg_x = x + col;
            if (bg_x < 0 || bg_x >= IMG_WIDTH) continue;

            uint16_t p = strip[sy * strip_w + col];
            uint8_t r = ((p >> 11) & 0x1F) << 3; r |= r >> 5;
            uint8_t g = ((p >>  5) & 0x3F) << 2; g |= g >> 6;
            uint8_t b = ( p        & 0x1F) << 3; b |= b >> 5;
            fb[bg_y * IMG_WIDTH + bg_x] = 0xFF000000 | (r << 16) | (g << 8) | b;
        }
    }
}

/* --- Baro drums with Geneva drive carry --- */
static void update_baro_drums(uint32_t *fb, const uint16_t *strip,
                              int strip_w, int strip_h, float digit_h, int vis_h,
                              float baro_inhg)
{
    float baro_val = baro_inhg * 100.0f;

    float drum3_pos = fmodf(baro_val, 10.0f);
    float drum2_raw = fmodf(baro_val / 10.0f, 10.0f);
    float drum1_raw = fmodf(baro_val / 100.0f, 10.0f);
    float drum0_raw = fmodf(baro_val / 1000.0f, 10.0f);

    float carry3 = (drum3_pos >= 9.0f) ? (drum3_pos - 9.0f) : 0.0f;
    float drum2_pos = floorf(drum2_raw) + carry3;
    float carry2 = (drum2_pos >= 9.0f && drum2_pos < 10.0f) ? (drum2_pos - 9.0f) : 0.0f;
    float drum1_pos = floorf(drum1_raw) + carry2;
    float carry1 = (drum1_pos >= 9.0f && drum1_pos < 10.0f) ? (drum1_pos - 9.0f) : 0.0f;
    float drum0_pos = floorf(drum0_raw) + carry1;

    while (drum0_pos < 0) drum0_pos += 10.0f;
    while (drum1_pos < 0) drum1_pos += 10.0f;
    while (drum2_pos < 0) drum2_pos += 10.0f;
    while (drum3_pos < 0) drum3_pos += 10.0f;
    drum0_pos = fmodf(drum0_pos, 10.0f);
    drum1_pos = fmodf(drum1_pos, 10.0f);
    drum2_pos = fmodf(drum2_pos, 10.0f);
    drum3_pos = fmodf(drum3_pos, 10.0f);

    int total_w = 4 * strip_w + 3 * BARO_GAP;
    int baro_x0 = 321 - total_w / 2;
    int x0 = baro_x0;
    int x1 = x0 + strip_w + BARO_GAP;
    int x2 = x1 + strip_w + BARO_GAP;
    int x3 = x2 + strip_w + BARO_GAP;

    blit_drum(fb, strip, strip_w, strip_h, digit_h, x0, BARO_Y, vis_h, drum0_pos);
    blit_drum(fb, strip, strip_w, strip_h, digit_h, x1, BARO_Y, vis_h, drum1_pos);
    blit_drum(fb, strip, strip_w, strip_h, digit_h, x2, BARO_Y, vis_h, drum2_pos);
    blit_drum(fb, strip, strip_w, strip_h, digit_h, x3, BARO_Y, vis_h, drum3_pos);
}

/* --- Pointer rotation with alpha blending --- */
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
    if (nx1 >= IMG_WIDTH)  nx1 = IMG_WIDTH - 1;
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

/* --- Tiny 5x7 font for readout --- */
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
    /* T */  {0x1F,0x04,0x04,0x04,0x04,0x04,0x04},
};

static int font_index(char c)
{
    if (c >= '0' && c <= '9') return 1 + (c - '0');
    if (c == '+') return 11;
    if (c == '-') return 12;
    if (c == 'F') return 13;
    if (c == 'T') return 14;
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

/* --- DRM/KMS --- */
struct drm_fb {
    int drm_fd;
    uint32_t fb_id, handle;
    uint32_t *map;
    uint32_t width, height, stride, size;
    uint32_t conn_id, crtc_id;
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
        if (dfb->drm_fd < 0) { perror("open /dev/dri/card*"); return -1; }
    }

    drmModeRes *res = drmModeGetResources(dfb->drm_fd);
    if (!res) {
        close(dfb->drm_fd);
        dfb->drm_fd = open("/dev/dri/card0", O_RDWR);
        if (dfb->drm_fd < 0) { perror("open card0"); return -1; }
        res = drmModeGetResources(dfb->drm_fd);
        if (!res) { fprintf(stderr, "drmModeGetResources failed\n"); return -1; }
    }

    drmModeConnector *conn = NULL;
    for (int i = 0; i < res->count_connectors; i++) {
        conn = drmModeGetConnector(dfb->drm_fd, res->connectors[i]);
        if (conn && conn->connection == DRM_MODE_CONNECTED && conn->count_modes > 0)
            break;
        drmModeFreeConnector(conn);
        conn = NULL;
    }
    if (!conn) { fprintf(stderr, "No connected display\n"); drmModeFreeResources(res); return -1; }

    dfb->conn_id = conn->connector_id;
    dfb->mode = conn->modes[0];
    dfb->width = dfb->mode.hdisplay;
    dfb->height = dfb->mode.vdisplay;
    printf("Display: %s %dx%d @ %dHz\n", conn->modes[0].name,
           dfb->width, dfb->height, dfb->mode.vrefresh);

    drmModeEncoder *enc = conn->encoder_id ? drmModeGetEncoder(dfb->drm_fd, conn->encoder_id) : NULL;
    if (enc) { dfb->crtc_id = enc->crtc_id; drmModeFreeEncoder(enc); }
    else { dfb->crtc_id = res->crtcs[0]; }

    dfb->saved_crtc = drmModeGetCrtc(dfb->drm_fd, dfb->crtc_id);
    drmModeFreeConnector(conn);
    drmModeFreeResources(res);

    struct drm_mode_create_dumb creq = {0};
    creq.width = dfb->width; creq.height = dfb->height; creq.bpp = 32;
    if (drmIoctl(dfb->drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq) < 0) { perror("CREATE_DUMB"); return -1; }
    dfb->handle = creq.handle; dfb->stride = creq.pitch; dfb->size = creq.size;

    if (drmModeAddFB(dfb->drm_fd, dfb->width, dfb->height, 24, 32,
                     dfb->stride, dfb->handle, &dfb->fb_id) < 0) { perror("AddFB"); return -1; }

    struct drm_mode_map_dumb mreq = {0};
    mreq.handle = dfb->handle;
    if (drmIoctl(dfb->drm_fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq) < 0) { perror("MAP_DUMB"); return -1; }
    dfb->map = mmap(0, dfb->size, PROT_READ|PROT_WRITE, MAP_SHARED, dfb->drm_fd, mreq.offset);
    if (dfb->map == MAP_FAILED) { perror("mmap"); return -1; }

    if (drmModeSetCrtc(dfb->drm_fd, dfb->crtc_id, dfb->fb_id, 0, 0,
                       &dfb->conn_id, 1, &dfb->mode) < 0) { perror("SetCrtc"); return -1; }
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

    snprintf(path, sizeof(path), "%s/image.bin", asset_dir);
    uint16_t *bg565 = load_bin(path, IMG_WIDTH * IMG_HEIGHT * 2);
    if (!bg565) return 1;

    snprintf(path, sizeof(path), "%s/pointerargb4444.bin", asset_dir);
    uint16_t *ptr4444 = load_bin(path, PTR_WIDTH * PTR_HEIGHT * 2);
    if (!ptr4444) return 1;

    snprintf(path, sizeof(path), "%s/100sWheel.bin", asset_dir);
    uint16_t *strip = load_bin(path, STRIP_WIDTH * STRIP_HEIGHT * 2);
    if (!strip) return 1;

    snprintf(path, sizeof(path), "%s/1000sWheel.bin", asset_dir);
    uint16_t *strip1k_full = load_bin(path, STRIP1K_WIDTH * STRIP1K_FULL_H * 2);
    if (!strip1k_full) return 1;
    uint16_t *strip1k = strip1k_full + STRIP1K_WIDTH * STRIP1K_HATCH;

    snprintf(path, sizeof(path), "%s/10000sWheel.bin", asset_dir);
    uint16_t *strip10k = load_bin(path, STRIP1K_WIDTH * STRIP1K_HEIGHT * 2);
    if (!strip10k) return 1;

    /* --- Build baro strip (white-on-black, cropped) --- */
    int orig_cell = (int)(DIGIT_HEIGHT + 0.5f);
    int crop_left = 2, crop_right = 25;
    int baro_strip_w = crop_right - crop_left + 1;

    int glyph_top = orig_cell, glyph_bot = 0;
    for (int d = 0; d < STRIP_DIGITS; d++) {
        int cell_y = (int)(d * DIGIT_HEIGHT);
        for (int row = 0; row < orig_cell && (cell_y + row) < STRIP_HEIGHT; row++) {
            int has_pixel = 0;
            for (int col = 0; col < STRIP_WIDTH; col++) {
                uint16_t p = strip[(cell_y + row) * STRIP_WIDTH + col];
                uint8_t r = ((p >> 11) & 0x1F) << 3;
                uint8_t g = ((p >>  5) & 0x3F) << 2;
                uint8_t b = ( p        & 0x1F) << 3;
                int lum = (r * 77 + g * 150 + b * 29) >> 8;
                if (lum < 128) { has_pixel = 1; break; }
            }
            if (has_pixel) {
                if (row < glyph_top) glyph_top = row;
                if (row > glyph_bot) glyph_bot = row;
            }
        }
    }
    int glyph_h = glyph_bot - glyph_top + 1;
    int baro_gap_v = 10;
    int baro_cell = glyph_h + baro_gap_v;
    int baro_strip_h = baro_cell * STRIP_DIGITS;

    uint16_t *baro_strip = calloc(baro_strip_w * baro_strip_h, 2);
    if (!baro_strip) return 1;

    for (int d = 0; d < STRIP_DIGITS; d++) {
        int src_y = (int)(d * DIGIT_HEIGHT) + glyph_top;
        int dst_y = d * baro_cell + baro_gap_v / 2;
        for (int row = 0; row < glyph_h; row++) {
            int sy = src_y + row;
            if (sy >= STRIP_HEIGHT) break;
            for (int col = 0; col < baro_strip_w; col++) {
                int src_col = crop_left + col;
                uint16_t p = strip[sy * STRIP_WIDTH + src_col];
                uint8_t r = ((p >> 11) & 0x1F) << 3;
                uint8_t g = ((p >>  5) & 0x3F) << 2;
                uint8_t b = ( p        & 0x1F) << 3;
                int lum = (r * 77 + g * 150 + b * 29) >> 8;
                baro_strip[(dst_y + row) * baro_strip_w + col] = (lum < 128) ? 0xFFFF : 0x0000;
            }
        }
    }

    float baro_digit_height = (float)baro_cell;

    /* --- Convert to ARGB8888 --- */
    uint32_t *bg_clean = malloc(IMG_WIDTH * IMG_HEIGHT * 4);
    uint32_t *render_fb = malloc(IMG_WIDTH * IMG_HEIGHT * 4);
    if (!bg_clean || !render_fb) return 1;
    rgb565_to_argb8888(bg565, bg_clean, IMG_WIDTH * IMG_HEIGHT);
    free(bg565);

    uint32_t *ptr8888 = malloc(PTR_WIDTH * PTR_HEIGHT * 4);
    if (!ptr8888) return 1;
    argb4444_to_argb8888(ptr4444, ptr8888, PTR_WIDTH * PTR_HEIGHT);
    free(ptr4444);

    /* --- Setup DRM --- */
    struct drm_fb dfb = {0};
    if (drm_setup(&dfb) < 0) { fprintf(stderr, "DRM setup failed\n"); return 1; }
    printf("Framebuffer: %dx%d stride=%d\n", dfb.width, dfb.height, dfb.stride);

    int offset_x = ((int)dfb.width - IMG_WIDTH) / 2;
    int offset_y = ((int)dfb.height - IMG_HEIGHT) / 2;
    if (offset_x < 0) offset_x = 0;
    if (offset_y < 0) offset_y = 0;

    memset(dfb.map, 0, dfb.size);

    /* --- Setup UDP --- */
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); drm_cleanup(&dfb); return 1; }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(LISTEN_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind"); close(sock); drm_cleanup(&dfb); return 1;
    }

    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    printf("=== GLASSALT Altimeter Receiver ===\n");
    printf("Listening on UDP port %d\n", LISTEN_PORT);
    printf("Press Ctrl+C to quit\n\n");

    float altitude = 0.0f;
    float display_alt = 0.0f;
    float baro_inhg = 29.92f;
    int connected = 0;
    int fb_stride_px = dfb.stride / 4;

    while (running) {
        /* Drain UDP packets */
        SimPacket pkt;
        struct sockaddr_in from_addr;
        socklen_t from_len = sizeof(from_addr);

        while (1) {
            ssize_t n = recvfrom(sock, &pkt, sizeof(pkt), 0,
                                 (struct sockaddr *)&from_addr, &from_len);
            if (n != sizeof(pkt)) break;
            if (pkt.instrument_id == INSTRUMENT_ALT) {
                altitude = pkt.value;
                if (!connected) {
                    char ip[INET_ADDRSTRLEN];
                    inet_ntop(AF_INET, &from_addr.sin_addr, ip, sizeof(ip));
                    printf("Connected! Receiving from %s\n", ip);
                    connected = 1;
                }
            }
        }

        /* Smooth pointer movement */
        float diff = altitude - display_alt;
        display_alt += diff * 0.15f;
        if (fabsf(diff) < 0.5f) display_alt = altitude;

        /* --- Compute altimeter elements --- */
        float alt = display_alt;

        /* Pointer: one revolution per 1000 ft */
        float pointer_angle = (alt / 1000.0f) * 2.0f * (float)M_PI;

        /* 100s drum */
        float drum_val = fmodf(alt / 100.0f + 5.0f, 10.0f);

        /* 1000s drum with transition */
        float drum1k_raw = fmodf(alt / 1000.0f, 10.0f);
        float within_1k = fmodf(alt, 1000.0f);
        float drum1k_val;
        if (within_1k < 100.0f && alt >= 100.0f) {
            float t = within_1k / 100.0f;
            drum1k_val = fmodf(floorf(drum1k_raw) - 1.0f + t + 10.0f, 10.0f);
        } else {
            drum1k_val = floorf(drum1k_raw);
        }

        /* 10000s drum with transition */
        float drum10k_raw = fmodf(alt / 10000.0f, 10.0f);
        float within_10k = fmodf(alt, 10000.0f);
        float drum10k_val;
        if (within_10k < 100.0f && alt >= 100.0f) {
            float t = within_10k / 100.0f;
            drum10k_val = fmodf(floorf(drum10k_raw) - 1.0f + t + 10.0f, 10.0f);
        } else {
            drum10k_val = floorf(drum10k_raw);
        }

        /* --- Render --- */
        memcpy(render_fb, bg_clean, IMG_WIDTH * IMG_HEIGHT * 4);

        /* Drums */
        blit_drum(render_fb, strip, STRIP_WIDTH, STRIP_HEIGHT,
                  DIGIT_HEIGHT, DRUM_X, DRUM_Y, DRUM_VIS_H, drum_val);
        blit_drum(render_fb, strip1k, STRIP1K_WIDTH, STRIP1K_HEIGHT,
                  DIGIT1K_HEIGHT, DRUM1K_X, DRUM1K_Y, DRUM1K_VIS_H, drum1k_val);
        blit_drum(render_fb, strip10k, STRIP1K_WIDTH, STRIP1K_HEIGHT,
                  DIGIT1K_HEIGHT, DRUM10K_X, DRUM1K_Y, DRUM1K_VIS_H, drum10k_val);

        /* Baro drums */
        int baro_vis = (int)(baro_digit_height * 1.1f);
        update_baro_drums(render_fb, baro_strip, baro_strip_w, baro_strip_h,
                          baro_digit_height, baro_vis, baro_inhg);

        /* Pointer */
        rotate_pointer(render_fb, ptr8888, pointer_angle);

        /* Digital altitude readout */
        char alt_str[16];
        snprintf(alt_str, sizeof(alt_str), "%d FT", (int)display_alt);
        draw_text(render_fb, IMG_WIDTH, IMG_HEIGHT,
                  IMG_WIDTH/2 - 50, IMG_HEIGHT - 30, alt_str, 2, 0xFF00FF00);

        /* --- Blit to DRM --- */
        for (int y = 0; y < IMG_HEIGHT && (y + offset_y) < (int)dfb.height; y++) {
            uint32_t *dst = dfb.map + (y + offset_y) * fb_stride_px + offset_x;
            uint32_t *src = render_fb + y * IMG_WIDTH;
            int copy_w = IMG_WIDTH;
            if (offset_x + copy_w > (int)dfb.width)
                copy_w = (int)dfb.width - offset_x;
            memcpy(dst, src, copy_w * 4);
        }

        usleep(33333);  /* ~30 fps */
    }

    printf("\nShutting down...\n");
    close(sock);
    drm_cleanup(&dfb);
    free(bg_clean); free(render_fb); free(ptr8888);
    free(strip); free(strip1k_full); free(strip10k); free(baro_strip);
    return 0;
}
