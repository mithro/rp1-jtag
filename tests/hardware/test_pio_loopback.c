/*
 * test_pio_loopback.c - PIO internal loopback test
 *
 * Tests the full host<->PIO data path (PIOLib RPC, kernel ioctl,
 * PIO SM, FIFO) using the jtag_loopback PIO program. No external
 * wiring or FPGA needed -- just an RPi 5.
 *
 * Requires: sudo (for /dev/pio0 access)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rp1_jtag.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int tests_passed = 0;
static int tests_failed = 0;

#define ASSERT(cond, msg) do { \
    if (!(cond)) { \
        fprintf(stderr, "FAIL: %s\n", msg); \
        tests_failed++; \
        return; \
    } \
} while(0)

#define ASSERT_EQ(a, b, msg) do { \
    if ((a) != (b)) { \
        fprintf(stderr, "FAIL: %s (got 0x%x, expected 0x%x)\n", \
                msg, (unsigned)(a), (unsigned)(b)); \
        tests_failed++; \
        return; \
    } \
} while(0)

#define PASS() do { tests_passed++; } while(0)

/* Test: small transfer (32 bits) */
static void test_loopback_32bit(void)
{
    rp1_jtag_t *jtag = rp1_jtag_init_loopback();
    ASSERT(jtag != NULL, "init_loopback failed");

    uint8_t tms[] = {0x00, 0x00, 0x00, 0x00};
    uint8_t tdi[] = {0xEF, 0xBE, 0xAD, 0xDE};
    uint8_t tdo[4] = {0};

    int rc = rp1_jtag_shift(jtag, 32, tms, tdi, tdo);
    ASSERT_EQ(rc, 0, "shift should succeed");

    /* In loopback mode, TDO should equal TDI */
    for (int i = 0; i < 4; i++) {
        ASSERT_EQ(tdo[i], tdi[i], "loopback mismatch");
    }

    rp1_jtag_close(jtag);
    PASS();
}

/* Test: large transfer (1 KB) */
static void test_loopback_1kb(void)
{
    rp1_jtag_t *jtag = rp1_jtag_init_loopback();
    ASSERT(jtag != NULL, "init_loopback failed");

    int bytes = 1024;
    int bits = bytes * 8;
    uint8_t *tms = calloc(bytes, 1);
    uint8_t *tdi = malloc(bytes);
    uint8_t *tdo = calloc(bytes, 1);

    /* Fill TDI with pattern */
    for (int i = 0; i < bytes; i++)
        tdi[i] = (uint8_t)(i & 0xFF);

    int rc = rp1_jtag_shift(jtag, bits, tms, tdi, tdo);
    ASSERT_EQ(rc, 0, "shift should succeed");

    int mismatches = 0;
    for (int i = 0; i < bytes; i++) {
        if (tdo[i] != tdi[i])
            mismatches++;
    }
    ASSERT_EQ(mismatches, 0, "loopback data mismatch");

    free(tms);
    free(tdi);
    free(tdo);
    rp1_jtag_close(jtag);
    PASS();
}

int main(void)
{
    printf("PIO loopback test (requires RPi 5)\n");
    printf("==================================\n\n");

    test_loopback_32bit();
    test_loopback_1kb();

    printf("\n%d passed, %d failed\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
