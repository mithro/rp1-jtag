/*
 * test_gpio_loopback.c - Simple wire loopback test
 *
 * Tests JTAG shift through real GPIO with a single jumper wire
 * connecting TDI output to TDO input. Uses the real jtag_shift
 * PIO program. Tests at multiple frequencies (1, 10, 33, 50 MHz).
 *
 * Requires: RPi 5 + 1 jumper wire (TDI GPIO -> TDO GPIO)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rp1_jtag.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* Default pins (NeTV2 wiring) */
#define DEFAULT_TCK 4
#define DEFAULT_TMS 17
#define DEFAULT_TDI 27
#define DEFAULT_TDO 22

/*
 * Run loopback tests at a given frequency.
 * Returns the number of failures.
 */
static int run_tests_at_freq(rp1_jtag_t *jtag, uint32_t freq_hz,
                             int *pass_count)
{
    int fail = 0;
    const char *freq_label;
    char freq_buf[32];

    if (freq_hz >= 1000000) {
        snprintf(freq_buf, sizeof(freq_buf), "%u MHz", freq_hz / 1000000);
    } else {
        snprintf(freq_buf, sizeof(freq_buf), "%u kHz", freq_hz / 1000);
    }
    freq_label = freq_buf;

    rp1_jtag_set_freq(jtag, freq_hz);
    uint32_t actual = rp1_jtag_get_freq(jtag);
    printf("\n--- %s (requested %u Hz, actual %u Hz) ---\n",
           freq_label, freq_hz, actual);

    /* Test 1: 8-bit transfer */
    {
        uint8_t tms[] = {0x00};
        uint8_t tdi[] = {0xA5};
        uint8_t tdo[1] = {0};

        int rc = rp1_jtag_shift(jtag, 8, tms, tdi, tdo);
        if (rc == 0 && tdo[0] == tdi[0]) {
            printf("  PASS: 8-bit loopback (0x%02X) @ %s\n",
                   tdo[0], freq_label);
            (*pass_count)++;
        } else {
            printf("  FAIL: 8-bit loopback @ %s (got 0x%02X, "
                   "expected 0x%02X, rc=%d)\n",
                   freq_label, tdo[0], tdi[0], rc);
            fail++;
        }
    }

    /* Test 2: 256-bit transfer (32 bytes) */
    {
        int bytes = 32;
        uint8_t tms[32] = {0};
        uint8_t tdi[32], tdo[32] = {0};
        for (int i = 0; i < bytes; i++)
            tdi[i] = (uint8_t)i;

        int rc = rp1_jtag_shift(jtag, bytes * 8, tms, tdi, tdo);
        int match = (rc == 0 && memcmp(tdi, tdo, bytes) == 0);
        if (match) {
            printf("  PASS: 256-bit loopback @ %s\n", freq_label);
            (*pass_count)++;
        } else {
            printf("  FAIL: 256-bit loopback @ %s (rc=%d)\n",
                   freq_label, rc);
            fail++;
        }
    }

    /* Test 3: 4 KB transfer with throughput measurement */
    {
        int bytes = 4096;
        uint8_t *tms = calloc(bytes, 1);
        uint8_t *tdi = malloc(bytes);
        uint8_t *tdo = calloc(bytes, 1);

        if (!tms || !tdi || !tdo) {
            printf("  FAIL: allocation failed for 4 KB test @ %s\n",
                   freq_label);
            free(tms);
            free(tdi);
            free(tdo);
            fail++;
        } else {
            for (int i = 0; i < bytes; i++)
                tdi[i] = (uint8_t)((i * 7 + 0x5A) & 0xFF);

            struct timespec t_start, t_end;
            clock_gettime(CLOCK_MONOTONIC, &t_start);

            int rc = rp1_jtag_shift(jtag, bytes * 8, tms, tdi, tdo);

            clock_gettime(CLOCK_MONOTONIC, &t_end);

            double elapsed_s = (t_end.tv_sec - t_start.tv_sec)
                             + (t_end.tv_nsec - t_start.tv_nsec) / 1e9;

            if (rc == 0 && memcmp(tdi, tdo, bytes) == 0) {
                double mbps = (2.0 * bytes / 1e6) / elapsed_s;
                printf("  PASS: 4 KB loopback @ %s "
                       "(%.1f ms, %.1f MB/s aggregate)\n",
                       freq_label, elapsed_s * 1000.0, mbps);
                (*pass_count)++;
            } else {
                int mismatches = 0;
                for (int i = 0; i < bytes; i++) {
                    if (tdo[i] != tdi[i])
                        mismatches++;
                }
                printf("  FAIL: 4 KB loopback @ %s (rc=%d, %d mismatches)\n",
                       freq_label, rc, mismatches);
                fail++;
            }

            free(tms);
            free(tdi);
            free(tdo);
        }
    }

    return fail;
}

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    printf("GPIO loopback test (requires RPi 5 + 1 jumper wire)\n");
    printf("===================================================\n\n");

    rp1_jtag_pins_t pins = {
        .tck = DEFAULT_TCK, .tms = DEFAULT_TMS,
        .tdi = DEFAULT_TDI, .tdo = DEFAULT_TDO,
        .srst = -1, .trst = -1
    };

    /* TODO: Parse command-line args for custom pin numbers */

    printf("Pins: TCK=%d TMS=%d TDI=%d TDO=%d\n",
           pins.tck, pins.tms, pins.tdi, pins.tdo);
    printf("Connect jumper wire: GPIO %d (TDI) -> GPIO %d (TDO)\n",
           pins.tdi, pins.tdo);

    rp1_jtag_t *jtag = rp1_jtag_init(&pins);
    if (!jtag) {
        fprintf(stderr, "Failed to initialize JTAG (need sudo?)\n");
        return 1;
    }

    int pass = 0, fail = 0;

    /* Test frequencies: 1, 10, 33, 50 MHz */
    static const uint32_t frequencies[] = {
         1000000,   /* 1 MHz - conservative */
        10000000,   /* 10 MHz */
        33000000,   /* 33 MHz */
        50000000,   /* 50 MHz - aggressive */
    };
    static const int num_freqs = sizeof(frequencies) / sizeof(frequencies[0]);

    for (int f = 0; f < num_freqs; f++) {
        fail += run_tests_at_freq(jtag, frequencies[f], &pass);
    }

    printf("\n%d passed, %d failed\n", pass, fail);
    rp1_jtag_close(jtag);
    return fail > 0 ? 1 : 0;
}
