/*
 * test_pio_loopback.c - PIO internal loopback test
 *
 * Tests the full host<->PIO data path (PIOLib RPC, kernel ioctl,
 * PIO SM, FIFO, DMA) using the jtag_loopback PIO program. No external
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
#include <time.h>

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

/* Test: 1 KB transfer */
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

/*
 * Helper: run a loopback transfer of given size and report throughput.
 * Returns 0 on success, -1 on failure.
 */
static int run_loopback_test(const char *label, int bytes)
{
    rp1_jtag_t *jtag = rp1_jtag_init_loopback();
    if (!jtag) {
        fprintf(stderr, "FAIL: %s: init_loopback failed\n", label);
        tests_failed++;
        return -1;
    }

    int bits = bytes * 8;
    uint8_t *tms = calloc(bytes, 1);
    uint8_t *tdi = malloc(bytes);
    uint8_t *tdo = calloc(bytes, 1);

    if (!tms || !tdi || !tdo) {
        fprintf(stderr, "FAIL: %s: allocation failed (%d bytes)\n",
                label, bytes);
        free(tms);
        free(tdi);
        free(tdo);
        rp1_jtag_close(jtag);
        tests_failed++;
        return -1;
    }

    /* Fill TDI with repeating pattern */
    for (int i = 0; i < bytes; i++)
        tdi[i] = (uint8_t)(i & 0xFF);

    struct timespec t_start, t_end;
    clock_gettime(CLOCK_MONOTONIC, &t_start);

    int rc = rp1_jtag_shift(jtag, bits, tms, tdi, tdo);

    clock_gettime(CLOCK_MONOTONIC, &t_end);

    if (rc != 0) {
        fprintf(stderr, "FAIL: %s: shift returned %d\n", label, rc);
        free(tms);
        free(tdi);
        free(tdo);
        rp1_jtag_close(jtag);
        tests_failed++;
        return -1;
    }

    /* Verify data integrity */
    int mismatches = 0;
    int first_mismatch = -1;
    for (int i = 0; i < bytes; i++) {
        if (tdo[i] != tdi[i]) {
            if (first_mismatch < 0)
                first_mismatch = i;
            mismatches++;
        }
    }

    /* Calculate throughput */
    double elapsed_s = (t_end.tv_sec - t_start.tv_sec)
                     + (t_end.tv_nsec - t_start.tv_nsec) / 1e9;
    /* Aggregate throughput: TX + RX combined */
    double mbps = (2.0 * bytes / 1e6) / elapsed_s;

    if (mismatches > 0) {
        fprintf(stderr, "FAIL: %s: %d mismatches (first at byte %d: "
                "got 0x%02x, expected 0x%02x)\n",
                label, mismatches, first_mismatch,
                tdo[first_mismatch], tdi[first_mismatch]);
        free(tms);
        free(tdi);
        free(tdo);
        rp1_jtag_close(jtag);
        tests_failed++;
        return -1;
    }

    printf("PASS: %s (%.1f ms, %.1f MB/s aggregate)\n",
           label, elapsed_s * 1000.0, mbps);
    tests_passed++;

    free(tms);
    free(tdi);
    free(tdo);
    rp1_jtag_close(jtag);
    return 0;
}

/* Test: 64 KB transfer (exercises DMA with large buffer) */
static void test_loopback_64kb(void)
{
    run_loopback_test("64 KB loopback", 64 * 1024);
}

/* Test: 256 KB transfer (bulk DMA throughput) */
static void test_loopback_256kb(void)
{
    run_loopback_test("256 KB loopback", 256 * 1024);
}

int main(void)
{
    printf("PIO loopback test (requires RPi 5)\n");
    printf("==================================\n\n");

    test_loopback_32bit();
    test_loopback_1kb();
    test_loopback_64kb();
    test_loopback_256kb();

    printf("\n%d passed, %d failed\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
