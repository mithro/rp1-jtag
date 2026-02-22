/*
 * test_data_packing.c - Test bit-to-word packing for PIO FIFO format
 *
 * Verifies bits_to_words() and words_to_bits() correctly convert
 * between byte-level bit vectors and 32-bit FIFO words.
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
    uint32_t _a = (uint32_t)(a), _b = (uint32_t)(b); \
    if (_a != _b) { \
        fprintf(stderr, "FAIL [%s:%d]: %s (got 0x%08x, expected 0x%08x)\n", \
                __func__, __LINE__, msg, _a, _b); \
        tests_failed++; \
        return; \
    } \
} while(0)

#define PASS() do { tests_passed++; } while(0)

/* Test: pack 8 bits into 1 word */
static void test_pack_8bits(void)
{
    uint8_t src[] = {0xA5};  /* 10100101 */
    uint32_t words[1];

    int count = bits_to_words(src, 0, 8, words);
    ASSERT_EQ(count, 1, "8 bits -> 1 word");
    ASSERT_EQ(words[0], 0x000000A5, "should preserve byte value");
    PASS();
}

/* Test: pack 32 bits into 1 word */
static void test_pack_32bits(void)
{
    uint8_t src[] = {0xEF, 0xBE, 0xAD, 0xDE};  /* 0xDEADBEEF in LE */
    uint32_t words[1];

    int count = bits_to_words(src, 0, 32, words);
    ASSERT_EQ(count, 1, "32 bits -> 1 word");
    ASSERT_EQ(words[0], 0xDEADBEEF, "should pack LE bytes into word");
    PASS();
}

/* Test: pack 64 bits into 2 words */
static void test_pack_64bits(void)
{
    uint8_t src[] = {0x78, 0x56, 0x34, 0x12, 0xEF, 0xCD, 0xAB, 0x90};
    uint32_t words[2];

    int count = bits_to_words(src, 0, 64, words);
    ASSERT_EQ(count, 2, "64 bits -> 2 words");
    ASSERT_EQ(words[0], 0x12345678, "word 0");
    ASSERT_EQ(words[1], 0x90ABCDEF, "word 1");
    PASS();
}

/* Test: pack partial word (e.g. 12 bits) */
static void test_pack_partial(void)
{
    uint8_t src[] = {0xFF, 0x0F};  /* 12 bits: all 1s */
    uint32_t words[1];

    int count = bits_to_words(src, 0, 12, words);
    ASSERT_EQ(count, 1, "12 bits -> 1 word");
    ASSERT_EQ(words[0], 0x00000FFF, "12 bits of 1 should be 0xFFF");
    PASS();
}

/* Test: pack with start_bit offset */
static void test_pack_offset(void)
{
    /* Source: byte0=0xAB, byte1=0xCD
     * Bits at offset 4: bit4=1, bit5=0, bit6=1, bit7=0, bit8=1, ...
     * Bytes: 0xAB = 10101011, starting at bit 4: 1010 (bits 4-7)
     *        0xCD = 11001101, bits 8-11: 1101 */
    uint8_t src[] = {0xAB, 0xCD};
    uint32_t words[1];

    int count = bits_to_words(src, 4, 8, words);
    ASSERT_EQ(count, 1, "8 bits from offset 4 -> 1 word");
    /* bits 4-11: 1010 1101 = 0xDA... wait let me recalculate.
     * 0xAB = bits [7:0] = 1,0,1,0,1,0,1,1
     * 0xCD = bits [15:8] = 1,1,0,0,1,1,0,1
     * Bits 4-11: bit4=1, bit5=0, bit6=1, bit7=0, bit8=1, bit9=1, bit10=0, bit11=0
     * As a byte (LSB-first): 10100110 -> wait, bit4 is bit0 of result,
     * bit5 is bit1, etc. So: 1,0,1,0,1,1,0,0 = 0x35... no.
     * bit4=1(b0), bit5=0(b1), bit6=1(b2), bit7=0(b3),
     * bit8=1(b4), bit9=1(b5), bit10=0(b6), bit11=0(b7)
     * = 0b00110101 = 0x35 ... hmm, let me re-think.
     *
     * Actually: bit4 of src = (src[0] >> 4) & 1 = (0xAB >> 4) & 1 = 0xA & 1 = 0
     * Wait: 0xAB = 0b10101011
     * bit0 = 1, bit1=1, bit2=0, bit3=1, bit4=0, bit5=1, bit6=0, bit7=1
     * 0xCD = 0b11001101
     * bit8=1, bit9=0, bit10=1, bit11=1, bit12=0, bit13=0, bit14=1, bit15=1
     *
     * bits_to_words(src, 4, 8, words):
     * Extracts bits 4-11: bit4=0, bit5=1, bit6=0, bit7=1, bit8=1, bit9=0, bit10=1, bit11=1
     * As word: bit4->bit0, bit5->bit1, ... = 0,1,0,1,1,0,1,1 = 0b11010110 = 0xD6 ... hmm.
     * Wait: LSB-first means bit4 goes to position 0 of output word.
     * 0,1,0,1,1,0,1,1 from bit0 to bit7 = 0xDA
     * Actually: bit0=0, bit1=1, bit2=0, bit3=1, bit4=1, bit5=0, bit6=1, bit7=1
     * = 0b11010100 + bit0 and bit1... let me just do it properly.
     * val = 0*1 + 1*2 + 0*4 + 1*8 + 1*16 + 0*32 + 1*64 + 1*128
     * = 2 + 8 + 16 + 64 + 128 = 218 = 0xDA */
    ASSERT_EQ(words[0], 0xDA, "bits 4-11 of 0xAB,0xCD should be 0xDA");
    PASS();
}

/* Test: unpack words back to bits */
static void test_unpack_roundtrip(void)
{
    uint8_t original[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x12, 0x34};
    uint32_t words[2];

    /* Pack 48 bits starting at offset 0 */
    int nw = bits_to_words(original, 0, 48, words);
    ASSERT_EQ(nw, 2, "48 bits -> 2 words");

    /* Unpack back */
    uint8_t result[6];
    memset(result, 0, sizeof(result));
    words_to_bits(words, nw, result, 0, 48);

    for (int i = 0; i < 6; i++) {
        ASSERT_EQ(result[i], original[i], "roundtrip mismatch");
    }
    PASS();
}

/* Test: zero bits */
static void test_zero_bits(void)
{
    uint8_t src[] = {0xFF};
    uint32_t words[1];

    int count = bits_to_words(src, 0, 0, words);
    ASSERT_EQ(count, 0, "0 bits -> 0 words");
    PASS();
}

int main(void)
{
    printf("Data packing tests\n");
    printf("==================\n\n");

    test_pack_8bits();
    test_pack_32bits();
    test_pack_64bits();
    test_pack_partial();
    test_pack_offset();
    test_unpack_roundtrip();
    test_zero_bits();

    printf("\n%d passed, %d failed\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
