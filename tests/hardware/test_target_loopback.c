/*
 * test_target_loopback.c - Full JTAG via PIO target SM
 *
 * Runs SM1 as a JTAG target (jtag_target PIO program) on separate
 * GPIO pins. User wires host outputs to target inputs with 4 jumper
 * wires. Tests the complete JTAG data path through real GPIO with
 * realistic timing and signal integrity.
 *
 * Requires: RPi 5 + 4 jumper wires
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rp1_jtag.h"
#include <stdio.h>
#include <string.h>

int main(int argc, char *argv[])
{
    printf("PIO target loopback test (requires RPi 5 + 4 jumper wires)\n");
    printf("==========================================================\n\n");

    /* TODO: Implement when PIOLib multi-SM support is tested.
     *
     * This test requires:
     * 1. Initialize SM0 as JTAG host (jtag_shift program)
     * 2. Initialize SM1 as JTAG target (jtag_target program)
     * 3. Wire host outputs to target inputs:
     *    - GPIO 4 (TCK) -> GPIO target_tck (e.g. 5)
     *    - GPIO 17 (TMS) -> GPIO target_tms (e.g. 6)
     *    - GPIO 27 (TDI) -> GPIO target_tdi (e.g. 12)
     *    - GPIO 22 (TDO) <- GPIO target_tdo (e.g. 13)
     * 4. Shift data through host, verify target echoes with 1-bit delay
     *
     * The target PIO program needs its WAIT GPIO instructions patched
     * with the actual target TCK GPIO number before loading.
     */

    printf("NOT YET IMPLEMENTED -- needs multi-SM PIOLib support\n");
    printf("See plan Phase 1, step 12-13 for implementation details.\n");
    return 0;
}
