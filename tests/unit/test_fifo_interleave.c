/*
 * test_fifo_interleave.c - Test TX/RX FIFO interleaving
 *
 * Verifies the pio_shift_run() function correctly interleaves
 * TX writes and RX reads, handling word boundaries for various
 * transfer sizes.
 *
 * Uses the mock PIOLib backend.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rp1_jtag.h"
#include "rp1_jtag_internal.h"
#include "pio_backend.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

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
    uint32_t _a = (uint32_t)(a), _b = (uint32_t)(b); \
    if (_a != _b) { \
        fprintf(stderr, "FAIL [%s:%d]: %s (got 0x%08x, expected 0x%08x)\n", \
                __func__, __LINE__, msg, _a, _b); \
        tests_failed++; \
        return; \
    } \
} while(0)

#define PASS() do { tests_passed++; } while(0)

/* External: test helper to create context with mock backend */
extern rp1_jtag_t *rp1_jtag_init_with_backend(pio_backend_t *backend,
                                                const rp1_jtag_pins_t *pins);

static rp1_jtag_t *create_mock_context(void)
{
    pio_backend_t *be = pio_backend_mock_create();
    if (!be) return NULL;

    rp1_jtag_pins_t pins = {
        .tck = 4, .tms = 17, .tdi = 27, .tdo = 22,
        .srst = -1, .trst = -1
    };

    return rp1_jtag_init_with_backend(be, &pins);
}

/* Test: single-bit shift writes count word + 1 data word */
static void test_single_bit(void)
{
    rp1_jtag_t *jtag = create_mock_context();
    ASSERT(jtag != NULL, "create failed");

    uint8_t tms[] = {0x00};
    uint8_t tdi[] = {0x01};  /* 1 bit: TDI=1 */
    uint8_t tdo[1] = {0};

    /* Pre-load mock with TDO response */
    uint32_t tdo_data[] = {0x00000001};
    pio_backend_mock_set_tdo_data(jtag->backend, tdo_data, 1);

    int rc = rp1_jtag_shift(jtag, 1, tms, tdi, tdo);
    ASSERT_EQ(rc, 0, "shift should succeed");

    /* Verify mock received: count word (0) + 1 data word */
    uint32_t tx_words[4];
    int tx_count = pio_backend_mock_get_tdi_data(jtag->backend, tx_words, 4);
    ASSERT_EQ(tx_count, 2, "should write 2 words (count + 1 data)");
    ASSERT_EQ(tx_words[0], 0, "count word should be 0 (1 bit - 1)");
    ASSERT_EQ(tx_words[1], 0x00000001, "data word should be 0x01");

    rp1_jtag_close(jtag);
    PASS();
}

/* Test: 32-bit shift writes count + 1 data word */
static void test_32bit(void)
{
    rp1_jtag_t *jtag = create_mock_context();
    ASSERT(jtag != NULL, "create failed");

    uint8_t tms[] = {0x00, 0x00, 0x00, 0x00};
    uint8_t tdi[] = {0xEF, 0xBE, 0xAD, 0xDE};
    uint8_t tdo[4] = {0};

    uint32_t tdo_data[] = {0x12345678};
    pio_backend_mock_set_tdo_data(jtag->backend, tdo_data, 1);

    int rc = rp1_jtag_shift(jtag, 32, tms, tdi, tdo);
    ASSERT_EQ(rc, 0, "shift should succeed");

    uint32_t tx_words[4];
    int tx_count = pio_backend_mock_get_tdi_data(jtag->backend, tx_words, 4);
    ASSERT_EQ(tx_count, 2, "count + 1 data word");
    ASSERT_EQ(tx_words[0], 31, "count = 31 for 32 bits");
    ASSERT_EQ(tx_words[1], 0xDEADBEEF, "data word");

    rp1_jtag_close(jtag);
    PASS();
}

/* Test: 64-bit shift writes count + 2 data words */
static void test_64bit(void)
{
    rp1_jtag_t *jtag = create_mock_context();
    ASSERT(jtag != NULL, "create failed");

    uint8_t tms[8];
    memset(tms, 0, sizeof(tms));
    uint8_t tdi[] = {0x78, 0x56, 0x34, 0x12, 0xEF, 0xCD, 0xAB, 0x90};
    uint8_t tdo[8] = {0};

    uint32_t tdo_data[] = {0x11111111, 0x22222222};
    pio_backend_mock_set_tdo_data(jtag->backend, tdo_data, 2);

    int rc = rp1_jtag_shift(jtag, 64, tms, tdi, tdo);
    ASSERT_EQ(rc, 0, "shift should succeed");

    uint32_t tx_words[8];
    int tx_count = pio_backend_mock_get_tdi_data(jtag->backend, tx_words, 8);
    ASSERT_EQ(tx_count, 3, "count + 2 data words");
    ASSERT_EQ(tx_words[0], 63, "count = 63 for 64 bits");
    ASSERT_EQ(tx_words[1], 0x12345678, "data word 0");
    ASSERT_EQ(tx_words[2], 0x90ABCDEF, "data word 1");

    rp1_jtag_close(jtag);
    PASS();
}

/* Test: TMS splitting generates correct GPIO calls */
static void test_tms_gpio(void)
{
    rp1_jtag_t *jtag = create_mock_context();
    ASSERT(jtag != NULL, "create failed");

    /* 4 bits: TMS = 0,0,1,1 = 0x0C */
    uint8_t tms[] = {0x0C};
    uint8_t tdi[] = {0x0F};
    uint8_t tdo[1] = {0};

    uint32_t tdo_data[] = {0, 0};
    pio_backend_mock_set_tdo_data(jtag->backend, tdo_data, 2);

    int rc = rp1_jtag_shift(jtag, 4, tms, tdi, tdo);
    ASSERT_EQ(rc, 0, "shift should succeed");

    /* Should have been split into 2 runs:
     * Run 0: bits 0-1 (TMS=0), 2 bits
     * Run 1: bits 2-3 (TMS=1), 2 bits
     * Each run sets TMS via GPIO, then does a PIO transfer */

    /* Verify TMS GPIO was set (last value should be 1 from run 1) */
    ASSERT(pio_backend_mock_get_gpio(jtag->backend, 17),
           "TMS GPIO should be 1 after last run");

    rp1_jtag_close(jtag);
    PASS();
}

/* Test: NULL TDO is allowed (discard output) */
static void test_null_tdo(void)
{
    rp1_jtag_t *jtag = create_mock_context();
    ASSERT(jtag != NULL, "create failed");

    uint8_t tms[] = {0x00};
    uint8_t tdi[] = {0xFF};

    uint32_t tdo_data[] = {0xFFFFFFFF};
    pio_backend_mock_set_tdo_data(jtag->backend, tdo_data, 1);

    int rc = rp1_jtag_shift(jtag, 8, tms, tdi, NULL);
    ASSERT_EQ(rc, 0, "shift with NULL tdo should succeed");

    rp1_jtag_close(jtag);
    PASS();
}

int main(void)
{
    printf("FIFO interleave tests\n");
    printf("=====================\n\n");

    test_single_bit();
    test_32bit();
    test_64bit();
    test_tms_gpio();
    test_null_tdo();

    printf("\n%d passed, %d failed\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
