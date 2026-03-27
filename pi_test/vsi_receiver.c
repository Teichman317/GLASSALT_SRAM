/*
 * GLASSALT VSI Receiver — Raspberry Pi headless instrument
 * Listens for UDP packets from the host simulator and displays
 * the vertical speed as a text gauge over SSH.
 *
 * Usage: ./vsi_receiver
 * Listens on port 5555 for SimPackets from the host PC.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/time.h>

#define LISTEN_PORT  5555

/* Must match the host's packet format exactly */
typedef struct {
    uint32_t instrument_id;   /* 1=altimeter, 2=VSI */
    float    value;           /* vertical speed in fpm */
} SimPacket;

#define INSTRUMENT_VSI  2

/* Simple ASCII bar graph */
static void draw_bar(float fpm, int width)
{
    int half = width / 2;
    float norm = fpm / 2000.0f;  /* normalize to ±1 */
    if (norm > 1.0f) norm = 1.0f;
    if (norm < -1.0f) norm = -1.0f;
    int bar = (int)(norm * half);

    printf("  DN [");
    for (int i = -half; i <= half; i++) {
        if (i == 0) {
            printf("|");
        } else if (bar >= 0 && i > 0 && i <= bar) {
            printf("#");
        } else if (bar < 0 && i < 0 && i >= bar) {
            printf("#");
        } else {
            printf(" ");
        }
    }
    printf("] UP\n");
}

int main(void)
{
    /* Create UDP socket */
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket");
        return 1;
    }

    /* Bind to port */
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(LISTEN_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(sock);
        return 1;
    }

    printf("=== GLASSALT VSI Receiver ===\n");
    printf("Listening on UDP port %d...\n", LISTEN_PORT);
    printf("Waiting for packets from host...\n\n");

    /* Set a timeout so we can detect when packets stop */
    struct timeval tv;
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    float last_fpm = 0.0f;
    int packets_received = 0;
    int display_counter = 0;
    int connected = 0;
    char sender_ip[INET_ADDRSTRLEN] = "";

    while (1) {
        SimPacket pkt;
        struct sockaddr_in from_addr;
        socklen_t from_len = sizeof(from_addr);

        ssize_t n = recvfrom(sock, &pkt, sizeof(pkt), 0,
                             (struct sockaddr *)&from_addr, &from_len);

        if (n == sizeof(pkt) && pkt.instrument_id == INSTRUMENT_VSI) {
            if (!connected) {
                inet_ntop(AF_INET, &from_addr.sin_addr, sender_ip, sizeof(sender_ip));
                printf("Connected! Receiving from %s\n\n", sender_ip);
                connected = 1;
            }

            last_fpm = pkt.value;
            packets_received++;
            display_counter++;

            /* Update display every 5th packet (~6 Hz, smooth for terminal) */
            if (display_counter >= 5) {
                display_counter = 0;

                printf("\033[2J\033[H");  /* clear screen */
                printf("╔════════════════════════════════════════╗\n");
                printf("║    GLASSALT VSI — Pi Receiver          ║\n");
                printf("╠════════════════════════════════════════╣\n");
                printf("║                                        ║\n");

                /* Direction indicator */
                const char *dir_str;
                if (last_fpm > 50.0f)       dir_str = "▲ CLIMBING";
                else if (last_fpm < -50.0f)  dir_str = "▼ DESCENDING";
                else                         dir_str = "— LEVEL";

                printf("║   Vertical Speed: %+7.0f fpm          ║\n", last_fpm);
                printf("║   Status:  %s                   ║\n", dir_str);
                printf("║                                        ║\n");

                draw_bar(last_fpm, 36);

                printf("║                                        ║\n");
                printf("║   Packets: %d  From: %s      \n",
                       packets_received, sender_ip);
                printf("╚════════════════════════════════════════╝\n");
            }
        } else if (n < 0) {
            /* Timeout — no packets received */
            if (connected) {
                printf("\033[2J\033[H");
                printf("╔════════════════════════════════════════╗\n");
                printf("║    GLASSALT VSI — Pi Receiver          ║\n");
                printf("╠════════════════════════════════════════╣\n");
                printf("║                                        ║\n");
                printf("║   -- NO SIGNAL --                      ║\n");
                printf("║   Waiting for host...                  ║\n");
                printf("║   Total packets received: %-6d        ║\n", packets_received);
                printf("║                                        ║\n");
                printf("╚════════════════════════════════════════╝\n");
            }
        }
    }

    close(sock);
    return 0;
}
