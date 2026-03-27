/*
 * GLASSALT Altimeter — Headless Pi version
 * Runs the same altitude math as the SDL2 simulator,
 * prints values to the terminal over SSH.
 *
 * No graphics — just the core logic proving the Pi
 * can run our altimeter engine.
 */
#include <stdio.h>
#include <math.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main(void)
{
    float altitude = 8000.0f;
    float alt_dir  = 1.0f;       /* 1 = climbing, -1 = descending */
    float baro_inhg = 29.92f;    /* barometric pressure in inHg */

    /* We use clock() to measure elapsed time between frames,
     * just like SDL_GetTicks() does in the graphical version. */
    struct timespec ts_last, ts_now;
    clock_gettime(CLOCK_MONOTONIC, &ts_last);

    printf("=== GLASSALT Altimeter Engine on Raspberry Pi ===\n");
    printf("Altitude sweeps 8,000 - 12,000 ft at 200 ft/sec\n");
    printf("Press Ctrl+C to stop\n\n");

    int frame = 0;

    while (1) {
        /* Measure elapsed time (dt) in seconds */
        clock_gettime(CLOCK_MONOTONIC, &ts_now);
        float dt = (ts_now.tv_sec  - ts_last.tv_sec)
                 + (ts_now.tv_nsec - ts_last.tv_nsec) / 1e9f;
        ts_last = ts_now;

        /* --- Same triangle wave as the SDL2 simulator --- */
        altitude += dt * 200.0f * alt_dir;
        if (altitude >= 12000.0f) { altitude = 12000.0f; alt_dir = -1.0f; }
        if (altitude <=  8000.0f) { altitude =  8000.0f; alt_dir =  1.0f; }

        /* --- Pointer angle (one full rotation per 1000 ft) --- */
        float pointer_deg = fmodf(altitude / 1000.0f, 1.0f) * 360.0f;

        /* --- 100s drum position (same math as simulator) --- */
        float drum100 = fmodf(altitude / 100.0f + 5.0f, 10.0f);

        /* --- 1000s drum with transition near boundaries --- */
        float drum1k_raw = fmodf(altitude / 1000.0f, 10.0f);
        float within_1k  = fmodf(altitude, 1000.0f);
        float drum1k;
        if (within_1k < 100.0f && altitude >= 100.0f) {
            float t = within_1k / 100.0f;
            drum1k = fmodf(floorf(drum1k_raw) - 1.0f + t + 10.0f, 10.0f);
        } else {
            drum1k = floorf(drum1k_raw);
        }

        /* --- 10,000s drum with transition near boundaries --- */
        float drum10k_raw = fmodf(altitude / 10000.0f, 10.0f);
        float within_10k  = fmodf(altitude, 10000.0f);
        float drum10k;
        if (within_10k < 100.0f && altitude >= 100.0f) {
            float t = within_10k / 100.0f;
            drum10k = fmodf(floorf(drum10k_raw) - 1.0f + t + 10.0f, 10.0f);
        } else {
            drum10k = floorf(drum10k_raw);
        }

        /* --- Baro drums (Geneva drive carry) --- */
        float baro_val = baro_inhg * 100.0f;
        float baro_d3 = fmodf(baro_val, 10.0f);
        float baro_d2_raw = fmodf(baro_val / 10.0f, 10.0f);
        float baro_d1_raw = fmodf(baro_val / 100.0f, 10.0f);
        float baro_d0_raw = fmodf(baro_val / 1000.0f, 10.0f);

        float carry3 = (baro_d3 >= 9.0f) ? (baro_d3 - 9.0f) : 0.0f;
        float baro_d2 = floorf(baro_d2_raw) + carry3;
        float carry2 = (baro_d2 >= 9.0f && baro_d2 < 10.0f) ? (baro_d2 - 9.0f) : 0.0f;
        float baro_d1 = floorf(baro_d1_raw) + carry2;
        float carry1 = (baro_d1 >= 9.0f && baro_d1 < 10.0f) ? (baro_d1 - 9.0f) : 0.0f;
        float baro_d0 = floorf(baro_d0_raw) + carry1;

        /* Print every ~0.5 seconds (every 50th frame at ~100 Hz) */
        frame++;
        if (frame % 50 == 0) {
            printf("\033[2J\033[H");  /* clear terminal, cursor to top */
            printf("╔══════════════════════════════════════╗\n");
            printf("║   GLASSALT AAU-19/A  Altimeter       ║\n");
            printf("╠══════════════════════════════════════╣\n");
            printf("║                                      ║\n");
            printf("║   Altitude:  %7.0f ft              ║\n", altitude);
            printf("║   Pointer:   %5.1f°                 ║\n", pointer_deg);
            printf("║                                      ║\n");
            printf("║   Drums:  [%1.0f] [%1.0f] [%1.0f] x100 ft    ║\n",
                   floorf(drum10k), floorf(drum1k), drum100);
            printf("║                                      ║\n");
            printf("║   Baro:   %1.0f%1.0f.%1.0f%1.0f inHg            ║\n",
                   floorf(fmodf(baro_d0, 10.0f)),
                   floorf(fmodf(baro_d1, 10.0f)),
                   floorf(fmodf(baro_d2, 10.0f)),
                   floorf(fmodf(baro_d3, 10.0f)));
            printf("║                                      ║\n");
            printf("║   %s                          ║\n",
                   alt_dir > 0 ? "▲ CLIMBING " : "▼ DESCENDING");
            printf("╚══════════════════════════════════════╝\n");
        }

        /* Sleep ~10ms to avoid hammering the CPU */
        struct timespec sleep_ts = {0, 10000000};  /* 10ms */
        nanosleep(&sleep_ts, NULL);
    }

    return 0;
}
