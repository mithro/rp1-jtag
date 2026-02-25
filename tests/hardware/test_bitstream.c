/*
 * test_bitstream.c - Load bitstream to FPGA and verify
 *
 * Loads a Xilinx bitstream (.bit) file to the NeTV2's XC7A100T
 * FPGA via JTAG and verifies configuration succeeded.
 *
 * Requires: RPi 5 + NeTV2 FPGA board + bitstream file
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rp1_jtag.h"
#include <stdio.h>

int main(int argc, char *argv[])
{
    printf("Bitstream load test (requires RPi 5 + NeTV2 + bitstream)\n");
    printf("========================================================\n\n");

    /* TODO: Implement bitstream loading.
     *
     * This requires:
     * 1. Parse .bit file header (sync word, device ID)
     * 2. Navigate TAP to Shift-IR
     * 3. Load CFG_IN instruction
     * 4. Navigate TAP to Shift-DR
     * 5. Stream bitstream data through Shift-DR
     * 6. Navigate to Run-Test/Idle
     * 7. Check DONE pin or read status register
     *
     * For now this is a placeholder. The real implementation will
     * be exercised via openFPGALoader integration (task 16-17).
     */

    if (argc < 2) {
        fprintf(stderr, "Usage: %s <bitstream.bit>\n", argv[0]);
        return 1;
    }

    printf("Bitstream: %s\n", argv[1]);
    printf("NOT YET IMPLEMENTED -- use openFPGALoader integration instead.\n");
    return 0;
}
