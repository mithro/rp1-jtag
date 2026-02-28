/*
 * test_many_shifts.c - Find DMA transfer size threshold
 *
 * Tests progressively larger JTAG shifts to find exactly
 * where DMA starts failing.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rp1_jtag.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int do_shift(rp1_jtag_t *jtag, int num_bits, const char *label)
{
    uint32_t bytes = (num_bits + 7) / 8;
    uint8_t *tms = calloc(bytes, 1);
    uint8_t *tdi = calloc(bytes, 1);
    uint8_t *tdo = calloc(bytes, 1);
    if (!tms || !tdi || !tdo) {
        free(tms); free(tdi); free(tdo);
        return -1;
    }

    int rc = rp1_jtag_shift(jtag, num_bits, tms, tdi, tdo);
    free(tms);
    free(tdi);
    free(tdo);
    return rc;
}

int main(void)
{
    printf("DMA transfer size threshold test\n");
    printf("=================================\n\n");

    rp1_jtag_pins_t pins = {
        .tck = 4, .tms = 17, .tdi = 27, .tdo = 22,
        .srst = -1, .trst = -1
    };

    rp1_jtag_t *jtag = rp1_jtag_init(&pins);
    if (!jtag) {
        fprintf(stderr, "FAIL: init failed\n");
        return 1;
    }

    /* Use default 10 MHz — don't override frequency */

    /* Test progressively larger shifts */
    int sizes[] = {5, 32, 33, 64, 96, 128, 160, 192, 224, 256,
                   288, 320, 512, 1024, 2048, 4096, 8192, 16384};
    int nsizes = sizeof(sizes) / sizeof(sizes[0]);

    for (int i = 0; i < nsizes; i++) {
        int rc = do_shift(jtag, sizes[i], "probe");
        printf("shift %6d bits  (DMA: tx0=%4d rx0=%4d tx1=%4d bytes): %s\n",
               sizes[i],
               (int)(((sizes[i]+31)/32 + 1) * 4),   /* sm0 TX */
               (int)((sizes[i]/32 + 1) * 4),         /* sm0 RX */
               (int)(((sizes[i]+31)/32) * 4),         /* sm1 TX */
               rc == 0 ? "OK" : "FAILED");
        if (rc != 0) {
            printf("\nThreshold: fails at %d bits\n", sizes[i]);
            if (i > 0) {
                printf("Last successful: %d bits\n", sizes[i-1]);
                /* Binary search between last success and first failure */
                int lo = sizes[i-1], hi = sizes[i];
                while (hi - lo > 1) {
                    int mid = (lo + hi) / 2;
                    /* Need fresh PIO for each attempt */
                    rp1_jtag_close(jtag);
                    jtag = rp1_jtag_init(&pins);
                    if (!jtag) break;
                    rc = do_shift(jtag, mid, "bsearch");
                    printf("  bsearch %d bits: %s\n", mid,
                           rc == 0 ? "OK" : "FAILED");
                    if (rc == 0) lo = mid; else hi = mid;
                }
                printf("Exact threshold: %d bits OK, %d bits FAIL\n", lo, hi);
            }
            break;
        }
    }

    rp1_jtag_close(jtag);
    return 0;
}
