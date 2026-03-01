/*
 * test_bypass_loopback.c - JTAG data integrity test via FPGA BYPASS
 *
 * Verifies bit-level accuracy of PIO JTAG shifts by using the FPGA's
 * BYPASS register as a known loopback. In BYPASS mode, TDI appears
 * on TDO delayed by exactly 1 bit. This detects:
 *   - Off-by-one bit errors
 *   - Extra/missing TCK pulses
 *   - Data corruption at chunk boundaries
 *   - Bit packing/unpacking bugs
 *
 * Requires: FPGA connected to NeTV2 JTAG pins.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rp1_jtag.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* NeTV2 JTAG pin assignments */
#define PIN_TCK 4
#define PIN_TMS 17
#define PIN_TDI 27
#define PIN_TDO 22

/* Xilinx 7-series 6-bit IR codes */
#define IR_BYPASS 0x3F
#define IR_LEN    6

/* Simple PRNG for generating test patterns */
static uint32_t xorshift32(uint32_t *state) {
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

/* Navigate TAP to Test-Logic-Reset */
static int tap_reset(rp1_jtag_t *jtag) {
    /* 6 clocks with TMS=1 guarantees TLR from any state */
    uint8_t tms[1] = {0x3F};  /* 6 bits all 1 */
    uint8_t tdi[1] = {0x00};
    return rp1_jtag_shift(jtag, 6, tms, tdi, NULL);
}

/* Navigate from TLR to Run-Test/Idle */
static int tap_to_rti(rp1_jtag_t *jtag) {
    uint8_t tms[1] = {0x00};  /* TMS=0 */
    uint8_t tdi[1] = {0x00};
    return rp1_jtag_shift(jtag, 1, tms, tdi, NULL);
}

/* Load an IR value from RTI. Ends in RTI. */
static int load_ir(rp1_jtag_t *jtag, uint8_t ir_val, uint8_t *captured) {
    int rc;

    /* RTI → Select-DR(TMS=1) → Select-IR(TMS=1) →
     * Capture-IR(TMS=0) → Shift-IR(TMS=0) */
    {
        uint8_t tms[1] = {0x03};  /* bits: 1,1,0,0 */
        uint8_t tdi[1] = {0x00};
        rc = rp1_jtag_shift(jtag, 4, tms, tdi, NULL);
        if (rc < 0) return rc;
    }

    /* Shift IR_LEN bits. Last bit has TMS=1 → Exit1-IR */
    {
        uint8_t tms[1] = {1u << (IR_LEN - 1)};  /* TMS: 0,0,0,0,0,1 */
        uint8_t tdi[1] = {ir_val};
        uint8_t tdo[1] = {0};
        rc = rp1_jtag_shift(jtag, IR_LEN, tms, tdi, tdo);
        if (rc < 0) return rc;
        if (captured) *captured = tdo[0];
    }

    /* Exit1-IR → Update-IR(TMS=1) → RTI(TMS=0) */
    {
        uint8_t tms[1] = {0x01};  /* bits: 1,0 */
        uint8_t tdi[1] = {0x00};
        rc = rp1_jtag_shift(jtag, 2, tms, tdi, NULL);
        if (rc < 0) return rc;
    }

    return 0;
}

/* Navigate from RTI to Shift-DR */
static int tap_to_shift_dr(rp1_jtag_t *jtag) {
    /* RTI → Select-DR(TMS=1) → Capture-DR(TMS=0) → Shift-DR(TMS=0) */
    uint8_t tms[1] = {0x01};  /* bits: 1,0,0 */
    uint8_t tdi[1] = {0x00};
    return rp1_jtag_shift(jtag, 3, tms, tdi, NULL);
}

/* Navigate from Exit1-DR to RTI */
static int tap_exit1dr_to_rti(rp1_jtag_t *jtag) {
    /* Exit1-DR → Update-DR(TMS=1) → RTI(TMS=0) */
    uint8_t tms[1] = {0x01};  /* bits: 1,0 */
    uint8_t tdi[1] = {0x00};
    return rp1_jtag_shift(jtag, 2, tms, tdi, NULL);
}

/*
 * Test BYPASS loopback at a given size.
 *
 * In BYPASS mode, TDO[i] should equal TDI[i-1] for i >= 1.
 * TDO[0] is the captured DR value (0 for BYPASS).
 *
 * Returns 0 on success, number of bit errors on failure.
 */
static int test_bypass(rp1_jtag_t *jtag, uint32_t num_bits) {
    int rc;
    uint32_t bytes = (num_bits + 7) / 8;

    uint8_t *tms = calloc(bytes, 1);  /* all zeros = stay in Shift-DR */
    uint8_t *tdi = malloc(bytes);
    uint8_t *tdo = calloc(bytes, 1);

    if (!tms || !tdi || !tdo) {
        free(tms); free(tdi); free(tdo);
        return -1;
    }

    /* Generate pseudo-random test pattern */
    uint32_t seed = 0xDEADBEEF ^ num_bits;
    for (uint32_t i = 0; i < bytes; i += 4) {
        uint32_t r = xorshift32(&seed);
        uint32_t remaining = bytes - i;
        if (remaining >= 4)
            memcpy(tdi + i, &r, 4);
        else
            memcpy(tdi + i, &r, remaining);
    }

    /* Navigate to Shift-DR */
    rc = tap_to_shift_dr(jtag);
    if (rc < 0) {
        fprintf(stderr, "  Failed to navigate to Shift-DR\n");
        free(tms); free(tdi); free(tdo);
        return -1;
    }

    /* Shift data through BYPASS (TMS=0, stay in Shift-DR).
     * We use end=0 equivalent: all TMS bits are 0.
     * After shifting, we still need to exit Shift-DR. */
    rc = rp1_jtag_shift(jtag, num_bits, tms, tdi, tdo);
    if (rc < 0) {
        fprintf(stderr, "  Shift failed: rc=%d\n", rc);
        free(tms); free(tdi); free(tdo);
        return -1;
    }

    /* Exit Shift-DR: shift 1 more bit with TMS=1 → Exit1-DR */
    {
        uint8_t exit_tms[1] = {0x01};
        uint8_t exit_tdi[1] = {0x00};
        uint8_t exit_tdo[1] = {0x00};
        rc = rp1_jtag_shift(jtag, 1, exit_tms, exit_tdi, exit_tdo);
        if (rc < 0) {
            fprintf(stderr, "  Exit shift failed: rc=%d\n", rc);
            free(tms); free(tdi); free(tdo);
            return -1;
        }
    }

    /* Return to RTI */
    rc = tap_exit1dr_to_rti(jtag);
    if (rc < 0) {
        fprintf(stderr, "  Failed to return to RTI\n");
    }

    /* Verify: TDO[i] should equal TDI[i-1] for i >= 1.
     * TDO[0] is the captured BYPASS register value (0).
     *
     * In BYPASS, the DR is a single flip-flop initialized to 0.
     * On each TCK rising edge: capture TDI into the flip-flop,
     * output old flip-flop value on TDO.
     * So TDO[0] = initial flip-flop = 0
     *    TDO[1] = TDI[0]
     *    TDO[2] = TDI[1]
     *    ...
     *    TDO[N-1] = TDI[N-2]
     */
    int errors = 0;
    int first_error_bit = -1;

    /* Check TDO[0] = 0 (bypass register capture value) */
    if (tdo[0] & 1) {
        errors++;
        first_error_bit = 0;
    }

    /* Check TDO[i] = TDI[i-1] for i = 1 to num_bits-1 */
    for (uint32_t i = 1; i < num_bits; i++) {
        int expected = (tdi[(i-1)/8] >> ((i-1) % 8)) & 1;
        int actual   = (tdo[i/8] >> (i % 8)) & 1;
        if (actual != expected) {
            errors++;
            if (first_error_bit < 0)
                first_error_bit = (int)i;
        }
    }

    if (errors > 0) {
        fprintf(stderr, "  %d errors in %u bits (first at bit %d)\n",
                errors, num_bits, first_error_bit);
        /* Print context around first error */
        if (first_error_bit >= 0) {
            int start = first_error_bit > 4 ? first_error_bit - 4 : 0;
            int end = first_error_bit + 12;
            if ((uint32_t)end > num_bits) end = (int)num_bits;
            fprintf(stderr, "  Context around first error (bit %d):\n",
                    first_error_bit);
            fprintf(stderr, "    Bit:      ");
            for (int b = start; b < end; b++)
                fprintf(stderr, "%5d", b);
            fprintf(stderr, "\n    TDI[b-1]: ");
            for (int b = start; b < end; b++) {
                if (b == 0)
                    fprintf(stderr, "    -");
                else
                    fprintf(stderr, "%5d", (tdi[(b-1)/8] >> ((b-1)%8)) & 1);
            }
            fprintf(stderr, "\n    TDO[b]:   ");
            for (int b = start; b < end; b++)
                fprintf(stderr, "%5d", (tdo[b/8] >> (b%8)) & 1);
            fprintf(stderr, "\n    Match:    ");
            for (int b = start; b < end; b++) {
                if (b == 0) {
                    fprintf(stderr, "%5c", (tdo[0]&1) == 0 ? '.' : 'X');
                } else {
                    int exp = (tdi[(b-1)/8] >> ((b-1)%8)) & 1;
                    int act = (tdo[b/8] >> (b%8)) & 1;
                    fprintf(stderr, "%5c", exp == act ? '.' : 'X');
                }
            }
            fprintf(stderr, "\n");
        }
    }

    free(tms);
    free(tdi);
    free(tdo);
    return errors;
}

int main(void) {
    printf("JTAG BYPASS Loopback Data Integrity Test\n");
    printf("=========================================\n\n");

    rp1_jtag_pins_t pins = {
        .tck = PIN_TCK, .tms = PIN_TMS,
        .tdi = PIN_TDI, .tdo = PIN_TDO,
        .srst = -1, .trst = -1
    };

    rp1_jtag_t *jtag = rp1_jtag_init(&pins);
    if (!jtag) {
        fprintf(stderr, "FAIL: init failed (check /dev/pio0 permissions)\n");
        return 1;
    }

    int rc;

    /* Reset TAP and go to RTI */
    rc = tap_reset(jtag);
    if (rc < 0) { fprintf(stderr, "FAIL: TAP reset\n"); return 1; }
    rc = tap_to_rti(jtag);
    if (rc < 0) { fprintf(stderr, "FAIL: to RTI\n"); return 1; }

    /* Read IDCODE first (default after TLR) */
    printf("Step 1: Read IDCODE\n");
    rc = tap_to_shift_dr(jtag);
    if (rc < 0) { fprintf(stderr, "FAIL: to Shift-DR\n"); return 1; }
    {
        uint8_t tms[4] = {0,0,0,0};
        uint8_t tdi[4] = {0,0,0,0};
        uint8_t tdo[4] = {0,0,0,0};
        /* Last bit TMS=1 to exit */
        tms[3] = 0x80;  /* bit 31 = TMS=1 */
        rc = rp1_jtag_shift(jtag, 32, tms, tdi, tdo);
        if (rc < 0) { fprintf(stderr, "FAIL: IDCODE shift\n"); return 1; }
        uint32_t idcode = tdo[0] | (tdo[1]<<8) | (tdo[2]<<16) | (tdo[3]<<24);
        printf("  IDCODE: 0x%08x", idcode);
        if (idcode == 0x13631093)
            printf(" (XC7A100T)\n");
        else if (idcode == 0)
            printf(" ERROR: 0 — JTAG not connected?\n");
        else
            printf(" (unknown)\n");
        if (idcode != 0x13631093) {
            fprintf(stderr, "FAIL: wrong IDCODE\n");
            rp1_jtag_close(jtag);
            return 1;
        }
    }
    /* Return to RTI from Exit1-DR */
    rc = tap_exit1dr_to_rti(jtag);
    if (rc < 0) { fprintf(stderr, "FAIL: to RTI\n"); return 1; }

    /* Load BYPASS instruction */
    printf("\nStep 2: Load BYPASS (IR=0x%02x)\n", IR_BYPASS);
    uint8_t captured_ir;
    rc = load_ir(jtag, IR_BYPASS, &captured_ir);
    if (rc < 0) { fprintf(stderr, "FAIL: load IR\n"); return 1; }
    printf("  Captured IR: 0x%02x (DONE=%d INIT=%d)\n",
           captured_ir, (captured_ir >> 5) & 1, (captured_ir >> 4) & 1);

    /* Test various shift sizes */
    printf("\nStep 3: BYPASS loopback tests\n");
    printf("  Each test shifts pseudo-random data and verifies TDO = TDI delayed by 1 bit\n\n");

    /* Test sizes that exercise different code paths:
     * - Small: 1-32 bits (below word boundary)
     * - Medium: 33-1024 bits (single chunk)
     * - Large: >1024 bits (multi-chunk at old BULK_CHUNK_BITS=1024)
     * - Very large: >30720 bits (multi-chunk at current BULK_CHUNK_BITS=30720)
     * - Bitstream-scale: 306056 bits (matching openFPGALoader's writeTDI chunk size)
     */
    uint32_t sizes[] = {
        /* Small */
        1, 2, 7, 8, 15, 16, 31, 32,
        /* Near word boundaries */
        33, 63, 64, 65, 95, 96, 97,
        /* Chunk boundaries */
        128, 256, 512, 1024, 2048,
        /* Multi-chunk (old threshold) */
        4096, 8192, 16384,
        /* Near BULK_CHUNK_BITS boundary */
        30719, 30720, 30721,
        /* Multi-chunk at current threshold */
        61440, 92160,
        /* openFPGALoader writeTDI size */
        306056,
    };
    int nsizes = sizeof(sizes) / sizeof(sizes[0]);
    int total_pass = 0, total_fail = 0;

    for (int i = 0; i < nsizes; i++) {
        int errs = test_bypass(jtag, sizes[i]);
        if (errs < 0) {
            printf("  %8u bits: ERROR (shift failed)\n", sizes[i]);
            total_fail++;
        } else if (errs == 0) {
            printf("  %8u bits: PASS\n", sizes[i]);
            total_pass++;
        } else {
            printf("  %8u bits: FAIL (%d bit errors)\n", sizes[i], errs);
            total_fail++;
        }

        /* Re-enter BYPASS after each test (TAP may have been disturbed) */
        tap_reset(jtag);
        tap_to_rti(jtag);
        load_ir(jtag, IR_BYPASS, NULL);
    }

    printf("\n=== RESULT ===\n");
    printf("  %d/%d tests passed, %d failed\n",
           total_pass, nsizes, total_fail);

    rp1_jtag_close(jtag);
    return total_fail > 0 ? 1 : 0;
}
