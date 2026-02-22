/*
 * test_tms_splitting.c - Test TMS run-length splitting
 *
 * Verifies the tms_scan_runs() function correctly identifies
 * contiguous runs of constant TMS value and splits at boundaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rp1_jtag.h"
#include "rp1_jtag_internal.h"
#include <stdio.h>
#include <string.h>

static int tests_passed = 0;
static int tests_failed = 0;

#define ASSERT(cond, msg) do { \
    if (!(cond)) { \
        fprintf(stderr, "FAIL [%s:%d]: %s\n", __func__, __LINE__, msg); \
        tests_failed++; \
        return; \
    } \
} while(0)

#define ASSERT_EQ(a, b, msg) do { \
    if ((a) != (b)) { \
        fprintf(stderr, "FAIL [%s:%d]: %s (got %d, expected %d)\n", \
                __func__, __LINE__, msg, (int)(a), (int)(b)); \
        tests_failed++; \
        return; \
    } \
} while(0)

#define PASS() do { tests_passed++; } while(0)

/* Test: all zeros (single run) */
static void test_all_zeros(void)
{
    uint8_t tms[] = {0x00};  /* 8 bits of TMS=0 */
    tms_run_t runs[8];

    int count = tms_scan_runs(tms, 8, runs, 8);
    ASSERT_EQ(count, 1, "all-zero should be 1 run");
    ASSERT_EQ(runs[0].start_bit, 0, "start_bit");
    ASSERT_EQ(runs[0].num_bits, 8, "num_bits");
    ASSERT_EQ(runs[0].tms_value, false, "tms_value");
    PASS();
}

/* Test: all ones (single run) */
static void test_all_ones(void)
{
    uint8_t tms[] = {0xFF};
    tms_run_t runs[8];

    int count = tms_scan_runs(tms, 8, runs, 8);
    ASSERT_EQ(count, 1, "all-ones should be 1 run");
    ASSERT_EQ(runs[0].tms_value, true, "tms_value should be 1");
    ASSERT_EQ(runs[0].num_bits, 8, "num_bits");
    PASS();
}

/* Test: alternating 0-1 (8 runs of 1 bit each) */
static void test_alternating(void)
{
    uint8_t tms[] = {0x55};  /* 01010101 */
    tms_run_t runs[16];

    int count = tms_scan_runs(tms, 8, runs, 16);
    ASSERT_EQ(count, 8, "alternating should be 8 runs");
    for (int i = 0; i < 8; i++) {
        ASSERT_EQ(runs[i].start_bit, i, "start_bit");
        ASSERT_EQ(runs[i].num_bits, 1, "num_bits should be 1");
        ASSERT_EQ(runs[i].tms_value, (i % 2) != 0, "tms_value alternates");
    }
    PASS();
}

/* Test: typical openFPGALoader pattern -- N-1 bits TMS=0, last bit TMS=1 */
static void test_typical_shift(void)
{
    /* 16 bits: first 15 are 0, last is 1 = 0x8000 in little-endian bit order
     * Byte 0: 0x00 (bits 0-7 all 0)
     * Byte 1: 0x80 (bits 8-14 are 0, bit 15 is 1) */
    uint8_t tms[] = {0x00, 0x80};
    tms_run_t runs[4];

    int count = tms_scan_runs(tms, 16, runs, 4);
    ASSERT_EQ(count, 2, "typical shift should be 2 runs");
    ASSERT_EQ(runs[0].start_bit, 0, "run 0 start");
    ASSERT_EQ(runs[0].num_bits, 15, "run 0 length");
    ASSERT_EQ(runs[0].tms_value, false, "run 0 TMS=0");
    ASSERT_EQ(runs[1].start_bit, 15, "run 1 start");
    ASSERT_EQ(runs[1].num_bits, 1, "run 1 length");
    ASSERT_EQ(runs[1].tms_value, true, "run 1 TMS=1");
    PASS();
}

/* Test: zero bits */
static void test_zero_bits(void)
{
    uint8_t tms[] = {0};
    tms_run_t runs[4];

    int count = tms_scan_runs(tms, 0, runs, 4);
    ASSERT_EQ(count, 0, "zero bits should give 0 runs");
    PASS();
}

/* Test: single bit */
static void test_single_bit(void)
{
    uint8_t tms[] = {0x01};
    tms_run_t runs[4];

    int count = tms_scan_runs(tms, 1, runs, 4);
    ASSERT_EQ(count, 1, "single bit should be 1 run");
    ASSERT_EQ(runs[0].num_bits, 1, "num_bits");
    ASSERT_EQ(runs[0].tms_value, true, "tms_value for bit=1");
    PASS();
}

/* Test: TAP state navigation -- TMS=11100 (5 bits) */
static void test_tap_navigation(void)
{
    uint8_t tms[] = {0x07};  /* bits: 1,1,1,0,0 (LSB first) */
    tms_run_t runs[4];

    int count = tms_scan_runs(tms, 5, runs, 4);
    ASSERT_EQ(count, 2, "11100 should be 2 runs");
    ASSERT_EQ(runs[0].start_bit, 0, "run 0 start");
    ASSERT_EQ(runs[0].num_bits, 3, "run 0: 3 bits of TMS=1");
    ASSERT_EQ(runs[0].tms_value, true, "run 0 TMS=1");
    ASSERT_EQ(runs[1].start_bit, 3, "run 1 start");
    ASSERT_EQ(runs[1].num_bits, 2, "run 1: 2 bits of TMS=0");
    ASSERT_EQ(runs[1].tms_value, false, "run 1 TMS=0");
    PASS();
}

/* Test: max_runs overflow */
static void test_overflow(void)
{
    uint8_t tms[] = {0x55};  /* 8 alternating runs */
    tms_run_t runs[4];

    int count = tms_scan_runs(tms, 8, runs, 4);
    ASSERT_EQ(count, -1, "should return -1 when max_runs exceeded");
    PASS();
}

int main(void)
{
    printf("TMS run-length splitting tests\n");
    printf("==============================\n\n");

    test_all_zeros();
    test_all_ones();
    test_alternating();
    test_typical_shift();
    test_zero_bits();
    test_single_bit();
    test_tap_navigation();
    test_overflow();

    printf("\n%d passed, %d failed\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
