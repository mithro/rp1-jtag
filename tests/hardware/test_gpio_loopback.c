/*
 * test_gpio_loopback.c - Simple wire loopback test
 *
 * Tests JTAG shift through real GPIO with a single jumper wire
 * connecting TDI output to TDO input. Uses the real jtag_shift
 * PIO program.
 *
 * Requires: RPi 5 + 1 jumper wire (TDI GPIO -> TDO GPIO)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rp1_jtag.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Default pins (NeTV2 wiring) */
#define DEFAULT_TCK 4
#define DEFAULT_TMS 17
#define DEFAULT_TDI 27
#define DEFAULT_TDO 22

int main(int argc, char *argv[])
{
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
    printf("Connect jumper wire: GPIO %d (TDI) -> GPIO %d (TDO)\n\n",
           pins.tdi, pins.tdo);

    rp1_jtag_t *jtag = rp1_jtag_init(&pins);
    if (!jtag) {
        fprintf(stderr, "Failed to initialize JTAG (need sudo?)\n");
        return 1;
    }

    /* Set frequency to conservative 1 MHz */
    rp1_jtag_set_freq(jtag, 1000000);

    int pass = 0, fail = 0;

    /* Test 1: 8-bit transfer */
    {
        uint8_t tms[] = {0x00};
        uint8_t tdi[] = {0xA5};
        uint8_t tdo[1] = {0};

        int rc = rp1_jtag_shift(jtag, 8, tms, tdi, tdo);
        if (rc == 0 && tdo[0] == tdi[0]) {
            printf("PASS: 8-bit loopback (0x%02X)\n", tdo[0]);
            pass++;
        } else {
            printf("FAIL: 8-bit loopback (got 0x%02X, expected 0x%02X, rc=%d)\n",
                   tdo[0], tdi[0], rc);
            fail++;
        }
    }

    /* Test 2: 256-bit transfer */
    {
        int bytes = 32;
        uint8_t tms[32] = {0};
        uint8_t tdi[32], tdo[32] = {0};
        for (int i = 0; i < bytes; i++)
            tdi[i] = (uint8_t)i;

        int rc = rp1_jtag_shift(jtag, bytes * 8, tms, tdi, tdo);
        int match = (rc == 0 && memcmp(tdi, tdo, bytes) == 0);
        if (match) {
            printf("PASS: 256-bit loopback\n");
            pass++;
        } else {
            printf("FAIL: 256-bit loopback (rc=%d)\n", rc);
            fail++;
        }
    }

    printf("\n%d passed, %d failed\n", pass, fail);
    rp1_jtag_close(jtag);
    return fail > 0 ? 1 : 0;
}
