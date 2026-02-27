/*
 * bench_dma_loopback.c - DMA throughput benchmark via PIO loopback
 *
 * Measures sustained DMA throughput using the jtag_loopback PIO program.
 * No external wiring needed -- uses PIO internal loopback.
 *
 * Reports min, max, mean, and median throughput in MB/s (decimal,
 * 1 MB = 1,000,000 bytes). Includes data verification on each iteration.
 *
 * Expected throughput: ~42 MB/s aggregate on RPi 5.
 *
 * Usage:
 *   sudo ./bench_dma_loopback [--size BYTES] [--iterations N]
 *
 * Requires: RPi 5, sudo for /dev/pio0
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rp1_jtag.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define DEFAULT_SIZE       (256 * 1024)  /* 256 KB */
#define DEFAULT_ITERATIONS 10
#define WARMUP_ITERATIONS  2

static int compare_double(const void *a, const void *b)
{
    double da = *(const double *)a;
    double db = *(const double *)b;
    if (da < db) return -1;
    if (da > db) return 1;
    return 0;
}

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [--size BYTES] [--iterations N]\n"
        "\n"
        "Options:\n"
        "  --size BYTES      Transfer size per iteration (default: %d)\n"
        "  --iterations N    Number of measured iterations (default: %d)\n"
        "\n"
        "Additionally runs %d warmup iterations (not counted in stats).\n"
        "Reports aggregate (TX+RX) throughput in MB/s (1 MB = 1,000,000 bytes).\n",
        prog, DEFAULT_SIZE, DEFAULT_ITERATIONS, WARMUP_ITERATIONS);
}

int main(int argc, char *argv[])
{
    int transfer_size = DEFAULT_SIZE;
    int iterations = DEFAULT_ITERATIONS;

    /* Parse arguments */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--size") == 0 && i + 1 < argc) {
            transfer_size = atoi(argv[++i]);
            if (transfer_size <= 0) {
                fprintf(stderr, "Error: invalid size '%s'\n", argv[i]);
                return 1;
            }
        } else if (strcmp(argv[i], "--iterations") == 0 && i + 1 < argc) {
            iterations = atoi(argv[++i]);
            if (iterations <= 0) {
                fprintf(stderr, "Error: invalid iterations '%s'\n", argv[i]);
                return 1;
            }
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            usage(argv[0]);
            return 1;
        }
    }

    printf("DMA loopback throughput benchmark\n");
    printf("=================================\n\n");
    printf("Transfer size:  %d bytes (%d KB)\n", transfer_size, transfer_size / 1024);
    printf("Iterations:     %d (+ %d warmup)\n", iterations, WARMUP_ITERATIONS);
    printf("\n");

    /* Initialize loopback mode (uses DMA internally) */
    rp1_jtag_t *jtag = rp1_jtag_init_loopback();
    if (!jtag) {
        fprintf(stderr, "Failed to initialize JTAG loopback (need sudo?)\n");
        return 1;
    }

    /* Allocate buffers */
    uint8_t *tms = calloc(transfer_size, 1);
    uint8_t *tdi = malloc(transfer_size);
    uint8_t *tdo = malloc(transfer_size);

    if (!tms || !tdi || !tdo) {
        fprintf(stderr, "Failed to allocate buffers (%d bytes each)\n",
                transfer_size);
        free(tms);
        free(tdi);
        free(tdo);
        rp1_jtag_close(jtag);
        return 1;
    }

    /* Fill TDI with repeating pattern */
    for (int i = 0; i < transfer_size; i++)
        tdi[i] = (uint8_t)(i & 0xFF);

    int total_iters = WARMUP_ITERATIONS + iterations;
    double *throughputs = malloc(iterations * sizeof(double));
    if (!throughputs) {
        fprintf(stderr, "Failed to allocate throughput array\n");
        free(tms);
        free(tdi);
        free(tdo);
        rp1_jtag_close(jtag);
        return 1;
    }

    int errors = 0;
    int measured = 0;

    for (int iter = 0; iter < total_iters; iter++) {
        bool is_warmup = (iter < WARMUP_ITERATIONS);
        memset(tdo, 0, transfer_size);

        struct timespec t_start, t_end;
        clock_gettime(CLOCK_MONOTONIC, &t_start);

        int rc = rp1_jtag_shift(jtag, (uint32_t)transfer_size * 8,
                                tms, tdi, tdo);

        clock_gettime(CLOCK_MONOTONIC, &t_end);

        if (rc != 0) {
            printf("  [%s iter %d] FAILED: rc=%d\n",
                   is_warmup ? "warmup" : "measured", iter, rc);
            errors++;
            continue;
        }

        /* Verify data integrity */
        int mismatches = 0;
        int first_mismatch = -1;
        for (int i = 0; i < transfer_size; i++) {
            if (tdo[i] != tdi[i]) {
                if (first_mismatch < 0)
                    first_mismatch = i;
                mismatches++;
            }
        }

        if (mismatches > 0) {
            printf("  [%s iter %d] DATA ERROR: %d mismatches "
                   "(first at byte %d: got 0x%02x, expected 0x%02x)\n",
                   is_warmup ? "warmup" : "measured", iter,
                   mismatches, first_mismatch,
                   tdo[first_mismatch], tdi[first_mismatch]);
            errors++;
            continue;
        }

        /* Calculate throughput */
        double elapsed_s = (t_end.tv_sec - t_start.tv_sec)
                         + (t_end.tv_nsec - t_start.tv_nsec) / 1e9;

        /* TX throughput: transfer_size bytes sent */
        double tx_mbps = (transfer_size / 1e6) / elapsed_s;
        /* RX throughput: transfer_size bytes received */
        double rx_mbps = tx_mbps;  /* Symmetric in loopback */
        /* Aggregate: TX + RX combined */
        double agg_mbps = tx_mbps + rx_mbps;

        if (is_warmup) {
            printf("  [warmup %d] %.1f ms, TX %.1f MB/s, "
                   "RX %.1f MB/s, aggregate %.1f MB/s\n",
                   iter, elapsed_s * 1000.0, tx_mbps, rx_mbps, agg_mbps);
        } else {
            printf("  [iter %d]   %.1f ms, TX %.1f MB/s, "
                   "RX %.1f MB/s, aggregate %.1f MB/s\n",
                   iter - WARMUP_ITERATIONS,
                   elapsed_s * 1000.0, tx_mbps, rx_mbps, agg_mbps);
            throughputs[measured++] = agg_mbps;
        }
    }

    /* Statistics */
    printf("\n");
    if (errors > 0) {
        printf("ERRORS: %d iterations failed\n", errors);
    }

    if (measured > 0) {
        /* Sort for median */
        qsort(throughputs, measured, sizeof(double), compare_double);

        double sum = 0;
        for (int i = 0; i < measured; i++)
            sum += throughputs[i];

        double min = throughputs[0];
        double max = throughputs[measured - 1];
        double mean = sum / measured;
        double median;
        if (measured % 2 == 0)
            median = (throughputs[measured / 2 - 1]
                    + throughputs[measured / 2]) / 2.0;
        else
            median = throughputs[measured / 2];

        printf("Results (%d iterations, %d KB transfers):\n",
               measured, transfer_size / 1024);
        printf("  Min:    %7.1f MB/s\n", min);
        printf("  Max:    %7.1f MB/s\n", max);
        printf("  Mean:   %7.1f MB/s\n", mean);
        printf("  Median: %7.1f MB/s\n", median);
        printf("\n");
        printf("(Aggregate = TX + RX combined, 1 MB = 1,000,000 bytes)\n");
    } else {
        printf("No successful iterations to report.\n");
    }

    free(throughputs);
    free(tms);
    free(tdi);
    free(tdo);
    rp1_jtag_close(jtag);
    return errors > 0 ? 1 : 0;
}
