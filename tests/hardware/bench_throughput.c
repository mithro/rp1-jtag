/*
 * bench_throughput.c - Throughput benchmark for all loopback modes
 *
 * Measures actual data throughput for:
 *   --pio-loopback:   PIO internal loopback (no wiring)
 *   --gpio-loopback:  TDI->TDO wire loopback (1 jumper)
 *   --target-loopback: PIO target SM loopback (4 jumpers)
 *
 * Reports throughput in kB/s and compares to sysfsgpio baseline.
 *
 * Requires: RPi 5 + optional wiring depending on mode
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rp1_jtag.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define TRANSFER_SIZE (32 * 1024)  /* 32 KB per run */
#define NUM_RUNS 5

static double time_diff_ms(struct timespec *start, struct timespec *end)
{
    double s = (end->tv_sec - start->tv_sec) * 1000.0;
    double ns = (end->tv_nsec - start->tv_nsec) / 1000000.0;
    return s + ns;
}

int main(int argc, char *argv[])
{
    printf("Throughput benchmark\n");
    printf("====================\n\n");

    /* TODO: Parse command-line arguments for mode and pins.
     * For now, default to PIO loopback mode. */

    printf("Mode: PIO loopback (default)\n");
    printf("Transfer size: %d bytes per run\n", TRANSFER_SIZE);
    printf("Runs: %d\n\n", NUM_RUNS);

    rp1_jtag_t *jtag = rp1_jtag_init_loopback();
    if (!jtag) {
        fprintf(stderr, "Failed to initialize JTAG loopback\n");
        return 1;
    }

    /* Set frequency */
    rp1_jtag_set_freq(jtag, 6000000);

    uint8_t *tms = calloc(TRANSFER_SIZE, 1);
    uint8_t *tdi = malloc(TRANSFER_SIZE);
    uint8_t *tdo = malloc(TRANSFER_SIZE);

    for (int i = 0; i < TRANSFER_SIZE; i++)
        tdi[i] = (uint8_t)(i & 0xFF);

    double total_ms = 0;
    int errors = 0;

    for (int run = 0; run < NUM_RUNS; run++) {
        memset(tdo, 0, TRANSFER_SIZE);

        struct timespec start, end;
        clock_gettime(CLOCK_MONOTONIC, &start);

        int rc = rp1_jtag_shift(jtag, TRANSFER_SIZE * 8, tms, tdi, tdo);

        clock_gettime(CLOCK_MONOTONIC, &end);

        if (rc < 0) {
            printf("Run %d: FAILED (rc=%d)\n", run, rc);
            errors++;
            continue;
        }

        double ms = time_diff_ms(&start, &end);
        double kbps = (TRANSFER_SIZE / 1024.0) / (ms / 1000.0);
        total_ms += ms;

        /* Verify data */
        int mismatches = 0;
        for (int i = 0; i < TRANSFER_SIZE; i++) {
            if (tdo[i] != tdi[i])
                mismatches++;
        }

        printf("Run %d: %.1f ms, %.1f kB/s, %d mismatches\n",
               run, ms, kbps, mismatches);
        if (mismatches > 0)
            errors++;
    }

    if (errors == 0) {
        double avg_ms = total_ms / NUM_RUNS;
        double avg_kbps = (TRANSFER_SIZE / 1024.0) / (avg_ms / 1000.0);
        printf("\nAverage: %.1f ms, %.1f kB/s\n", avg_ms, avg_kbps);
        printf("Baseline: sysfsgpio ~5 kB/s\n");
        printf("Speedup: %.0fx\n", avg_kbps / 5.0);
    }

    free(tms);
    free(tdi);
    free(tdo);
    rp1_jtag_close(jtag);
    return errors > 0 ? 1 : 0;
}
