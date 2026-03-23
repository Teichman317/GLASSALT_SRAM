/*
 * Generate baro drum strip .bin from 100sWheel.bin
 * Output: ../baroWheel.bin (RGB565, white-on-black, cropped & spaced)
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#define STRIP_WIDTH   28
#define STRIP_HEIGHT  567
#define STRIP_DIGITS  10
#define DIGIT_HEIGHT  ((float)STRIP_HEIGHT / STRIP_DIGITS)  /* 56.7 px */

int main(void)
{
    FILE *f = fopen("../100sWheel.bin", "rb");
    if (!f) { fprintf(stderr, "Cannot open ../100sWheel.bin\n"); return 1; }
    uint16_t *strip = malloc(STRIP_WIDTH * STRIP_HEIGHT * 2);
    if (!strip) return 1;
    fread(strip, 2, STRIP_WIDTH * STRIP_HEIGHT, f);
    fclose(f);

    int orig_cell = (int)(DIGIT_HEIGHT + 0.5f);
    int crop_left = 2;
    int crop_right = 25;
    int baro_strip_w = crop_right - crop_left + 1;  /* 24px wide */

    /* Find glyph vertical bounds */
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

    /* Write output */
    f = fopen("../baroWheel.bin", "wb");
    if (!f) { fprintf(stderr, "Cannot create ../baroWheel.bin\n"); return 1; }
    fwrite(baro_strip, 2, baro_strip_w * baro_strip_h, f);
    fclose(f);

    printf("Generated baroWheel.bin: %d x %d pixels, %d bytes (RGB565)\n",
           baro_strip_w, baro_strip_h, baro_strip_w * baro_strip_h * 2);
    printf("  glyph height: %d px, cell height: %d px, gap: %d px\n",
           glyph_h, baro_cell, baro_gap_v);

    free(strip);
    free(baro_strip);
    return 0;
}
