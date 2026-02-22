/*
 * idcode_read.c - Minimal IDCODE read example
 *
 * Reads the JTAG IDCODE from a connected target device.
 *
 * Usage: sudo ./idcode_read [--tck N] [--tms N] [--tdi N] [--tdo N]
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rp1_jtag.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void usage(const char *prog)
{
    fprintf(stderr, "Usage: %s [--tck N] [--tms N] [--tdi N] [--tdo N]\n", prog);
    fprintf(stderr, "  Default pins: TCK=4 TMS=17 TDI=27 TDO=22\n");
}

int main(int argc, char *argv[])
{
    rp1_jtag_pins_t pins = {
        .tck = 4, .tms = 17, .tdi = 27, .tdo = 22,
        .srst = -1, .trst = -1
    };

    /* Parse args */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--tck") == 0 && i + 1 < argc)
            pins.tck = atoi(argv[++i]);
        else if (strcmp(argv[i], "--tms") == 0 && i + 1 < argc)
            pins.tms = atoi(argv[++i]);
        else if (strcmp(argv[i], "--tdi") == 0 && i + 1 < argc)
            pins.tdi = atoi(argv[++i]);
        else if (strcmp(argv[i], "--tdo") == 0 && i + 1 < argc)
            pins.tdo = atoi(argv[++i]);
        else {
            usage(argv[0]);
            return 1;
        }
    }

    rp1_jtag_t *jtag = rp1_jtag_init(&pins);
    if (!jtag) {
        fprintf(stderr, "Failed to initialize JTAG. Run with sudo.\n");
        return 1;
    }

    rp1_jtag_set_freq(jtag, 1000000);

    /* Reset TAP: 5 TCK with TMS=1 */
    rp1_jtag_toggle_clk(jtag, 5, true, true);

    /* Run-Test/Idle: 1 TCK with TMS=0 */
    rp1_jtag_toggle_clk(jtag, 1, false, true);

    /* Navigate to Shift-DR: TMS=1,0,0 */
    uint8_t nav_tms[] = {0x01};
    uint8_t nav_tdi[] = {0xFF};
    rp1_jtag_shift(jtag, 3, nav_tms, nav_tdi, NULL);

    /* Read 32-bit IDCODE (TMS=0 for 31 bits, TMS=1 for last bit) */
    uint8_t tms[4] = {0x00, 0x00, 0x00, 0x80};
    uint8_t tdi[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t tdo[4] = {0};

    int rc = rp1_jtag_shift(jtag, 32, tms, tdi, tdo);
    if (rc < 0) {
        fprintf(stderr, "IDCODE read failed: %d\n", rc);
        rp1_jtag_close(jtag);
        return 1;
    }

    uint32_t idcode = tdo[0] | (tdo[1] << 8) | (tdo[2] << 16) | (tdo[3] << 24);
    printf("IDCODE: 0x%08X\n", idcode);

    /* Return to Run-Test/Idle */
    rp1_jtag_toggle_clk(jtag, 1, true, true);
    rp1_jtag_toggle_clk(jtag, 1, false, true);

    rp1_jtag_close(jtag);
    return 0;
}
