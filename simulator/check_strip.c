/* Quick tool to find the leftmost and rightmost non-background columns in the strip */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

int main(void)
{
    FILE *f = fopen("../100sWheel.bin", "rb");
    if (!f) { printf("Cannot open\n"); return 1; }
    int W = 28, H = 567;
    uint16_t *strip = malloc(W * H * 2);
    fread(strip, 2, W * H, f);
    fclose(f);

    /* Find leftmost and rightmost columns that have any dark (digit) pixel */
    int left = W, right = 0;
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            uint16_t p = strip[y * W + x];
            uint8_t r = ((p >> 11) & 0x1F) << 3;
            uint8_t g = ((p >>  5) & 0x3F) << 2;
            uint8_t b = ( p        & 0x1F) << 3;
            int lum = (r * 77 + g * 150 + b * 29) >> 8;
            if (lum < 128) {
                if (x < left) left = x;
                if (x > right) right = x;
            }
        }
    }
    printf("Digit pixels span columns %d to %d (width %d of %d)\n", left, right, right - left + 1, W);
    printf("Left padding: %d px, Right padding: %d px\n", left, W - 1 - right);
    free(strip);
    return 0;
}
