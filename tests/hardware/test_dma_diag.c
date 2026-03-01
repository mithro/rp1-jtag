/*
 * test_dma_diag.c - Direct DMA diagnostic for JTAG shift
 *
 * Tests whether the TX side or RX side is broken by:
 * 1. Shifting all-0xFF through BYPASS (if TDO=0 after bit 32, TX is broken)
 * 2. Shifting 64 bits at different TCK speeds (if speed-dependent, timing issue)
 * 3. Shifting with TDO=NULL then reading back (isolates TX from RX)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rp1_jtag.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define PIN_TCK 4
#define PIN_TMS 17
#define PIN_TDI 27
#define PIN_TDO 22

#define IR_BYPASS 0x3F
#define IR_LEN    6

static int tap_reset(rp1_jtag_t *jtag) {
    uint8_t tms[1] = {0x3F};
    uint8_t tdi[1] = {0x00};
    return rp1_jtag_shift(jtag, 6, tms, tdi, NULL);
}

static int tap_to_rti(rp1_jtag_t *jtag) {
    uint8_t tms[1] = {0x00};
    uint8_t tdi[1] = {0x00};
    return rp1_jtag_shift(jtag, 1, tms, tdi, NULL);
}

static int load_ir(rp1_jtag_t *jtag, uint8_t ir_val) {
    int rc;
    { uint8_t t[1]={0x03}, d[1]={0}; rc=rp1_jtag_shift(jtag,4,t,d,NULL); if(rc<0)return rc; }
    { uint8_t t[1]={1u<<(IR_LEN-1)}, d[1]={ir_val}, o[1]={0};
      rc=rp1_jtag_shift(jtag,IR_LEN,t,d,o); if(rc<0)return rc; }
    { uint8_t t[1]={0x01}, d[1]={0}; rc=rp1_jtag_shift(jtag,2,t,d,NULL); if(rc<0)return rc; }
    return 0;
}

static int tap_to_shift_dr(rp1_jtag_t *jtag) {
    uint8_t tms[1] = {0x01};
    uint8_t tdi[1] = {0x00};
    return rp1_jtag_shift(jtag, 3, tms, tdi, NULL);
}

static int tap_exit1dr_to_rti(rp1_jtag_t *jtag) {
    uint8_t tms[1] = {0x01};
    uint8_t tdi[1] = {0x00};
    return rp1_jtag_shift(jtag, 2, tms, tdi, NULL);
}

static void dump_bits(const char *label, const uint8_t *data, uint32_t num_bits) {
    printf("  %s: ", label);
    for (uint32_t i = 0; i < num_bits && i < 96; i++) {
        if (i > 0 && i % 32 == 0) printf(" | ");
        else if (i > 0 && i % 8 == 0) printf(" ");
        printf("%d", (data[i/8] >> (i%8)) & 1);
    }
    if (num_bits > 96) printf(" ...");
    printf("\n");
}

/* Shift through BYPASS with known pattern, check results */
static int test_pattern(rp1_jtag_t *jtag, uint32_t num_bits,
                        uint8_t fill_byte, const char *desc)
{
    int rc;
    uint32_t bytes = (num_bits + 7) / 8;

    uint8_t *tms = calloc(bytes, 1);
    uint8_t *tdi = malloc(bytes);
    uint8_t *tdo = calloc(bytes, 1);
    if (!tms || !tdi || !tdo) { free(tms); free(tdi); free(tdo); return -1; }

    memset(tdi, fill_byte, bytes);
    /* Mask unused bits in last byte */
    if (num_bits % 8)
        tdi[bytes-1] &= (1u << (num_bits % 8)) - 1;

    printf("\n--- %s: %u bits, TDI=0x%02x ---\n", desc, num_bits, fill_byte);

    /* Navigate to Shift-DR */
    rc = tap_to_shift_dr(jtag);
    if (rc < 0) { printf("  FAIL: to Shift-DR\n"); goto out; }

    /* Shift data */
    rc = rp1_jtag_shift(jtag, num_bits, tms, tdi, tdo);
    if (rc < 0) { printf("  FAIL: shift rc=%d\n", rc); goto out; }

    /* Exit Shift-DR */
    { uint8_t et[1]={1}, ed[1]={0}, eo[1]={0};
      rp1_jtag_shift(jtag, 1, et, ed, eo); }
    tap_exit1dr_to_rti(jtag);

    dump_bits("TDI", tdi, num_bits);
    dump_bits("TDO", tdo, num_bits);

    /* Check BYPASS relationship: TDO[0]=0, TDO[i]=TDI[i-1] */
    int errors = 0;
    if (tdo[0] & 1) errors++;
    for (uint32_t i = 1; i < num_bits; i++) {
        int expected = (tdi[(i-1)/8] >> ((i-1)%8)) & 1;
        int actual = (tdo[i/8] >> (i%8)) & 1;
        if (actual != expected) errors++;
    }
    printf("  Result: %d errors in %u bits\n", errors, num_bits);

    /* Show word-by-word hex for analysis */
    printf("  TDI words:");
    for (uint32_t w = 0; w < (num_bits+31)/32; w++) {
        uint32_t word = 0;
        memcpy(&word, tdi + w*4, (bytes - w*4) >= 4 ? 4 : (bytes - w*4));
        printf(" 0x%08x", word);
    }
    printf("\n");
    printf("  TDO words:");
    for (uint32_t w = 0; w < (num_bits+31)/32; w++) {
        uint32_t word = 0;
        uint32_t avail = bytes - w*4;
        memcpy(&word, tdo + w*4, avail >= 4 ? 4 : avail);
        printf(" 0x%08x", word);
    }
    printf("\n");

    rc = errors;
out:
    free(tms); free(tdi); free(tdo);
    return rc;
}

int main(void) {
    printf("JTAG DMA Diagnostic Test\n");
    printf("========================\n");

    rp1_jtag_pins_t pins = {
        .tck = PIN_TCK, .tms = PIN_TMS,
        .tdi = PIN_TDI, .tdo = PIN_TDO,
        .srst = -1, .trst = -1
    };

    rp1_jtag_t *jtag = rp1_jtag_init(&pins);
    if (!jtag) { fprintf(stderr, "FAIL: init\n"); return 1; }

    int rc;
    rc = tap_reset(jtag);
    if (rc < 0) { fprintf(stderr, "FAIL: reset\n"); return 1; }
    rc = tap_to_rti(jtag);
    if (rc < 0) { fprintf(stderr, "FAIL: to RTI\n"); return 1; }

    /* Read IDCODE */
    rc = tap_to_shift_dr(jtag);
    if (rc < 0) { fprintf(stderr, "FAIL: to Shift-DR\n"); return 1; }
    {
        uint8_t tms[4] = {0,0,0,0}, tdi[4] = {0,0,0,0}, tdo[4] = {0,0,0,0};
        tms[3] = 0x80;
        rc = rp1_jtag_shift(jtag, 32, tms, tdi, tdo);
        if (rc < 0) { fprintf(stderr, "FAIL: IDCODE\n"); return 1; }
        uint32_t id = tdo[0]|(tdo[1]<<8)|(tdo[2]<<16)|(tdo[3]<<24);
        printf("IDCODE: 0x%08x\n", id);
        if (id != 0x13631093) { fprintf(stderr, "Wrong IDCODE\n"); return 1; }
    }
    tap_exit1dr_to_rti(jtag);

    /* Load BYPASS */
    rc = load_ir(jtag, IR_BYPASS);
    if (rc < 0) { fprintf(stderr, "FAIL: load IR\n"); return 1; }

    printf("\n=== Test 1: All-ones TDI (isolates TX vs RX) ===\n");
    printf("If TDO=0 after bit 32, TX is broken (SM gets wrong data)\n");
    printf("If TDO=1 after bit 32, TX works, RX capture is broken\n");

    /* Test all-ones at various sizes */
    int sizes[] = {32, 33, 48, 63, 64, 96, 128, 256};
    for (int i = 0; i < 8; i++) {
        tap_reset(jtag); tap_to_rti(jtag); load_ir(jtag, IR_BYPASS);
        test_pattern(jtag, sizes[i], 0xFF, "All-ones");
    }

    printf("\n=== Test 2: All-zeros TDI (control) ===\n");
    printf("TDO should be all zeros (BYPASS of all zeros)\n");
    tap_reset(jtag); tap_to_rti(jtag); load_ir(jtag, IR_BYPASS);
    test_pattern(jtag, 128, 0x00, "All-zeros");

    printf("\n=== Test 3: Alternating pattern 0xAA ===\n");
    tap_reset(jtag); tap_to_rti(jtag); load_ir(jtag, IR_BYPASS);
    test_pattern(jtag, 128, 0xAA, "Alternating-AA");

    printf("\n=== Test 4: Alternating pattern 0x55 ===\n");
    tap_reset(jtag); tap_to_rti(jtag); load_ir(jtag, IR_BYPASS);
    test_pattern(jtag, 128, 0x55, "Alternating-55");

    rp1_jtag_close(jtag);
    return 0;
}
