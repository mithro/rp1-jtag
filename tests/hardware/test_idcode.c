/*
 * test_idcode.c - Read FPGA IDCODE via JTAG
 *
 * Navigates the JTAG TAP state machine to Shift-DR and reads
 * the 32-bit IDCODE register. Expects 0x13631093 (XC7A100T).
 *
 * Requires: RPi 5 + NeTV2 FPGA board
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rp1_jtag.h"
#include <stdio.h>
#include <string.h>

/* Default NeTV2 pins */
#define TCK 4
#define TMS 17
#define TDI 27
#define TDO 22

/* Expected IDCODE for Xilinx Artix-7 XC7A100T */
#define EXPECTED_IDCODE 0x13631093

int main(void)
{
    printf("JTAG IDCODE test (requires RPi 5 + NeTV2)\n");
    printf("==========================================\n\n");

    rp1_jtag_pins_t pins = {
        .tck = TCK, .tms = TMS, .tdi = TDI, .tdo = TDO,
        .srst = -1, .trst = -1
    };

    rp1_jtag_t *jtag = rp1_jtag_init(&pins);
    if (!jtag) {
        fprintf(stderr, "Failed to initialize JTAG\n");
        return 1;
    }

    /* Set conservative frequency */
    rp1_jtag_set_freq(jtag, 1000000);

    /*
     * TAP state machine navigation to read IDCODE:
     *
     * 1. Reset to Test-Logic-Reset: 5 TCK with TMS=1
     * 2. Go to Run-Test/Idle: 1 TCK with TMS=0
     * 3. Go to Select-DR-Scan: 1 TCK with TMS=1
     * 4. Go to Capture-DR: 1 TCK with TMS=0
     * 5. Go to Shift-DR: 1 TCK with TMS=0
     * 6. Shift out 32 bits with TMS=0 (IDCODE data)
     *    (last bit with TMS=1 to exit Shift-DR)
     * 7. Go to Update-DR: 1 TCK with TMS=1
     * 8. Go to Run-Test/Idle: 1 TCK with TMS=0
     */

    int rc;

    /* Step 1: Reset (5 clocks with TMS=1) */
    rc = rp1_jtag_toggle_clk(jtag, 5, true, true);
    if (rc < 0) { fprintf(stderr, "Reset failed: %d\n", rc); return 1; }

    /* Step 2: Run-Test/Idle (1 clock TMS=0) */
    rc = rp1_jtag_toggle_clk(jtag, 1, false, true);
    if (rc < 0) { fprintf(stderr, "RTI failed: %d\n", rc); return 1; }

    /* Steps 3-5: Navigate to Shift-DR: TMS=1,0,0 */
    {
        uint8_t tms[] = {0x01};  /* bits: 1,0,0 */
        uint8_t tdi[] = {0xFF};
        rc = rp1_jtag_shift(jtag, 3, tms, tdi, NULL);
        if (rc < 0) { fprintf(stderr, "Navigate to Shift-DR failed: %d\n", rc); return 1; }
    }

    /* Step 6: Read 32-bit IDCODE from Shift-DR
     * TMS=0 for first 31 bits, TMS=1 for last bit (exit Shift-DR) */
    {
        uint8_t tms[4] = {0x00, 0x00, 0x00, 0x80}; /* bit 31 = 1 */
        uint8_t tdi[4] = {0xFF, 0xFF, 0xFF, 0xFF};  /* TDI high per convention */
        uint8_t tdo[4] = {0};

        rc = rp1_jtag_shift(jtag, 32, tms, tdi, tdo);
        if (rc < 0) { fprintf(stderr, "IDCODE read failed: %d\n", rc); return 1; }

        uint32_t idcode = tdo[0] | (tdo[1] << 8) | (tdo[2] << 16) | (tdo[3] << 24);
        printf("IDCODE: 0x%08X\n", idcode);

        if (idcode == EXPECTED_IDCODE) {
            printf("PASS: matches XC7A100T\n");
        } else {
            printf("FAIL: expected 0x%08X\n", EXPECTED_IDCODE);
            rp1_jtag_close(jtag);
            return 1;
        }
    }

    /* Steps 7-8: Update-DR -> Run-Test/Idle */
    rc = rp1_jtag_toggle_clk(jtag, 1, true, true);   /* TMS=1: Update-DR */
    rc = rp1_jtag_toggle_clk(jtag, 1, false, true);  /* TMS=0: RTI */

    rp1_jtag_close(jtag);
    return 0;
}
