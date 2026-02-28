/*
 * test_single_shift.c - Test a single JTAG shift of a given size
 *
 * Usage: test_single_shift <num_bits>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rp1_jtag.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char *argv[])
{
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <num_bits>\n", argv[0]);
        return 2;
    }

    int num_bits = atoi(argv[1]);
    if (num_bits <= 0) {
        fprintf(stderr, "Invalid num_bits: %s\n", argv[1]);
        return 2;
    }

    printf("Testing single shift of %d bits\n", num_bits);

    rp1_jtag_pins_t pins = {
        .tck = 4, .tms = 17, .tdi = 27, .tdo = 22,
        .srst = -1, .trst = -1
    };

    rp1_jtag_t *jtag = rp1_jtag_init(&pins);
    if (!jtag) {
        fprintf(stderr, "FAIL: init failed\n");
        return 1;
    }

    uint32_t bytes = (num_bits + 7) / 8;
    uint8_t *tms = calloc(bytes, 1);
    uint8_t *tdi = calloc(bytes, 1);
    uint8_t *tdo = calloc(bytes, 1);
    if (!tms || !tdi || !tdo) {
        fprintf(stderr, "FAIL: malloc failed\n");
        return 1;
    }

    int rc = rp1_jtag_shift(jtag, num_bits, tms, tdi, tdo);
    printf("Result: %s (rc=%d)\n", rc == 0 ? "OK" : "FAILED", rc);

    free(tms);
    free(tdi);
    free(tdo);
    rp1_jtag_close(jtag);
    return rc == 0 ? 0 : 1;
}
