/*
 * test_dma_buffer_packing.c - Test DMA buffer layout calculations
 *
 * Verifies bits_to_word_count(), SM0/SM1 TX buffer layout, and the
 * partial-word alignment fix needed for RX data from the PIO ISR.
 * All calculations, no hardware or PIOLib required.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rp1_jtag.h"
#include "rp1_jtag_internal.h"
#include <stdio.h>
#include <stdlib.h>
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
    uint32_t _a = (uint32_t)(a), _b = (uint32_t)(b); \
    if (_a != _b) { \
        fprintf(stderr, "FAIL [%s:%d]: %s (got 0x%08x, expected 0x%08x)\n", \
                __func__, __LINE__, msg, _a, _b); \
        tests_failed++; \
        return; \
    } \
} while(0)

#define PASS() do { tests_passed++; } while(0)

/* ---- bits_to_word_count() tests ---- */

static void test_word_count_zero(void)
{
    ASSERT_EQ(bits_to_word_count(0), 0, "0 bits -> 0 words");
    PASS();
}

static void test_word_count_one(void)
{
    ASSERT_EQ(bits_to_word_count(1), 1, "1 bit -> 1 word");
    PASS();
}

static void test_word_count_31(void)
{
    ASSERT_EQ(bits_to_word_count(31), 1, "31 bits -> 1 word");
    PASS();
}

static void test_word_count_32(void)
{
    ASSERT_EQ(bits_to_word_count(32), 1, "32 bits -> 1 word");
    PASS();
}

static void test_word_count_33(void)
{
    ASSERT_EQ(bits_to_word_count(33), 2, "33 bits -> 2 words");
    PASS();
}

static void test_word_count_64(void)
{
    ASSERT_EQ(bits_to_word_count(64), 2, "64 bits -> 2 words");
    PASS();
}

static void test_word_count_65(void)
{
    ASSERT_EQ(bits_to_word_count(65), 3, "65 bits -> 3 words");
    PASS();
}

/* ---- SM0 TX buffer layout tests ---- */

/*
 * SM0 TX buffer format:
 *   [0]   = num_bits - 1   (count word for PIO counted loop)
 *   [1..] = TDI data packed as 32-bit words, LSB-first
 */

static void test_sm0_tx_layout_8bits(void)
{
    uint32_t num_bits = 8;
    uint32_t num_data_words = bits_to_word_count(num_bits);
    size_t sm0_tx_words = 1 + num_data_words;  /* count + data */

    uint32_t *sm0_tx = calloc(sm0_tx_words, sizeof(uint32_t));
    ASSERT(sm0_tx != NULL, "calloc failed");

    /* Fill count word */
    sm0_tx[0] = num_bits - 1;

    /* Fill TDI data */
    uint8_t tdi[] = {0xA5};
    bits_to_words(tdi, 0, num_bits, &sm0_tx[1]);

    ASSERT_EQ(sm0_tx[0], 7, "count word = num_bits-1 = 7");
    ASSERT_EQ(sm0_tx[1], 0x000000A5, "TDI word 0");
    ASSERT_EQ(sm0_tx_words, 2, "total SM0 TX = 2 words (1 count + 1 data)");

    free(sm0_tx);
    PASS();
}

static void test_sm0_tx_layout_48bits(void)
{
    uint32_t num_bits = 48;
    uint32_t num_data_words = bits_to_word_count(num_bits);
    size_t sm0_tx_words = 1 + num_data_words;

    uint32_t *sm0_tx = calloc(sm0_tx_words, sizeof(uint32_t));
    ASSERT(sm0_tx != NULL, "calloc failed");

    sm0_tx[0] = num_bits - 1;

    uint8_t tdi[] = {0x78, 0x56, 0x34, 0x12, 0xEF, 0xCD};
    bits_to_words(tdi, 0, num_bits, &sm0_tx[1]);

    ASSERT_EQ(sm0_tx[0], 47, "count word = 47");
    ASSERT_EQ(sm0_tx[1], 0x12345678, "TDI word 0");
    ASSERT_EQ(sm0_tx[2], 0x0000CDEF, "TDI word 1 (16 bits)");
    ASSERT_EQ(sm0_tx_words, 3, "total SM0 TX = 3 words (1 count + 2 data)");

    free(sm0_tx);
    PASS();
}

/* ---- SM1 TX buffer layout tests ---- */

/*
 * SM1 TX buffer format:
 *   TMS data packed as 32-bit words, LSB-first (no count word)
 */

static void test_sm1_tx_layout(void)
{
    uint32_t num_bits = 8;
    uint32_t num_data_words = bits_to_word_count(num_bits);

    uint32_t *sm1_tx = calloc(num_data_words, sizeof(uint32_t));
    ASSERT(sm1_tx != NULL, "calloc failed");

    /* TMS pattern: 0b11000001 = transition at start and end */
    uint8_t tms[] = {0xC1};
    bits_to_words(tms, 0, num_bits, sm1_tx);

    ASSERT_EQ(sm1_tx[0], 0x000000C1, "TMS word 0");
    ASSERT_EQ(num_data_words, 1, "SM1 TX = 1 word (same count as data words)");

    free(sm1_tx);
    PASS();
}

/* ---- Partial-word alignment fix tests ---- */

/*
 * With in_shift_right=true (autopush at 32), the PIO ISR shifts data
 * from MSB. Full 32-bit words are correctly aligned by autopush.
 * But the last partial word (from explicit push) has valid bits at
 * [(32-N)..31] instead of [0..(N-1)], so it needs right-shifting
 * by (32 - N%32).
 */

static void test_partial_word_no_fix_needed(void)
{
    /* 32 bits: no remainder, no fix needed */
    uint32_t num_bits = 32;
    uint32_t remainder = num_bits % BITS_PER_WORD;

    ASSERT_EQ(remainder, 0, "32 bits has no remainder");
    /* No shift should be applied when remainder == 0 */
    PASS();
}

static void test_partial_word_fix_8bits(void)
{
    /* 8 bits: remainder=8, last word needs >>= 24 */
    uint32_t num_bits = 8;
    uint32_t num_data_words = bits_to_word_count(num_bits);
    uint32_t remainder = num_bits % BITS_PER_WORD;

    ASSERT_EQ(remainder, 8, "8 bits remainder = 8");
    ASSERT_EQ(BITS_PER_WORD - remainder, 24, "shift amount = 24");

    /* Simulate what PIO returns: 8 bits in upper bits of word */
    uint32_t rx_word = 0xA5000000;  /* 0xA5 in bits [31:24] */
    rx_word >>= (BITS_PER_WORD - remainder);

    ASSERT_EQ(rx_word, 0x000000A5, "after fix: 0xA5 in bits [7:0]");
    ASSERT_EQ(num_data_words, 1, "1 data word");
    PASS();
}

static void test_partial_word_fix_12bits(void)
{
    /* 12 bits: remainder=12, last word needs >>= 20 */
    uint32_t num_bits = 12;
    uint32_t remainder = num_bits % BITS_PER_WORD;

    ASSERT_EQ(remainder, 12, "12 bits remainder = 12");
    ASSERT_EQ(BITS_PER_WORD - remainder, 20, "shift amount = 20");

    /* Simulate: 12 bits 0xABC in upper bits */
    uint32_t rx_word = 0xABC00000;  /* 0xABC in bits [31:20] */
    rx_word >>= (BITS_PER_WORD - remainder);

    ASSERT_EQ(rx_word, 0x00000ABC, "after fix: 0xABC in bits [11:0]");
    PASS();
}

static void test_partial_word_fix_48bits(void)
{
    /* 48 bits = 1 full word + 16-bit partial.
     * Only the last word (word[1]) needs the fix. */
    uint32_t num_bits = 48;
    uint32_t num_data_words = bits_to_word_count(num_bits);
    uint32_t remainder = num_bits % BITS_PER_WORD;

    ASSERT_EQ(num_data_words, 2, "48 bits -> 2 words");
    ASSERT_EQ(remainder, 16, "48 bits remainder = 16");

    /* Simulate RX buffer: word[0] is full (fine), word[1] is partial */
    uint32_t sm0_rx[2];
    sm0_rx[0] = 0xDEADBEEF;     /* full word, aligned correctly */
    sm0_rx[1] = 0xCAFE0000;     /* 16 bits in upper half */

    /* Apply the fix to the last word only */
    if (remainder != 0) {
        sm0_rx[num_data_words - 1] >>= (BITS_PER_WORD - remainder);
    }

    ASSERT_EQ(sm0_rx[0], 0xDEADBEEF, "word[0] unchanged (full word)");
    ASSERT_EQ(sm0_rx[1], 0x0000CAFE, "word[1] after fix: 0xCAFE in bits [15:0]");
    PASS();
}

/* ---- Buffer size calculation tests ---- */

static void test_buffer_sizes(void)
{
    uint32_t num_bits = 100;
    uint32_t num_data_words = bits_to_word_count(num_bits);

    /* SM0 TX: 1 count word + data words */
    size_t sm0_tx_bytes = (1 + num_data_words) * sizeof(uint32_t);
    /* SM0 RX: data words only */
    size_t sm0_rx_bytes = num_data_words * sizeof(uint32_t);
    /* SM1 TX: data words only (same count as SM0 data) */
    size_t sm1_tx_bytes = num_data_words * sizeof(uint32_t);

    ASSERT_EQ(num_data_words, 4, "100 bits -> 4 words");
    ASSERT_EQ(sm0_tx_bytes, 5 * 4, "SM0 TX = 20 bytes (1+4 words)");
    ASSERT_EQ(sm0_rx_bytes, 4 * 4, "SM0 RX = 16 bytes (4 words)");
    ASSERT_EQ(sm1_tx_bytes, 4 * 4, "SM1 TX = 16 bytes (4 words)");
    PASS();
}

int main(void)
{
    printf("DMA buffer packing tests\n");
    printf("========================\n\n");

    /* bits_to_word_count */
    test_word_count_zero();
    test_word_count_one();
    test_word_count_31();
    test_word_count_32();
    test_word_count_33();
    test_word_count_64();
    test_word_count_65();

    /* SM0 TX layout */
    test_sm0_tx_layout_8bits();
    test_sm0_tx_layout_48bits();

    /* SM1 TX layout */
    test_sm1_tx_layout();

    /* Partial-word alignment fix */
    test_partial_word_no_fix_needed();
    test_partial_word_fix_8bits();
    test_partial_word_fix_12bits();
    test_partial_word_fix_48bits();

    /* Buffer size calculations */
    test_buffer_sizes();

    printf("\n%d passed, %d failed\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
