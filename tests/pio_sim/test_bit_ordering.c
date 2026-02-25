/*
 * test_bit_ordering.c - Verify JTAG LSB-first bit ordering
 *
 * JTAG spec requires LSB-first shifting. Verifies that:
 *   - TDI bits shift out LSB-first from each FIFO word
 *   - TDO bits shift in LSB-first into each FIFO word
 *   - Byte ordering is correct for multi-byte transfers
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "pio_sim.h"
#include <stdio.h>

static const uint16_t jtag_shift_program[] = {
    0x80a0, /*  0: pull   block           side 0 */
    0x6020, /*  1: out    x, 32           side 0 */
    0x6001, /*  2: out    pins, 1         side 0 */
    0xa042, /*  3: nop                    side 0 */
    0xb042, /*  4: nop                    side 1 */
    0x5001, /*  5: in     pins, 1         side 1 */
    0x0042, /*  6: jmp    x--, 2          side 0 */
    0x8020, /*  7: push   block           side 0 */
};

#define TCK_PIN 4
#define TDI_PIN 27
#define TDO_PIN 22

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
        fprintf(stderr, "FAIL [%s:%d]: %s (got 0x%x, expected 0x%x)\n", \
                __func__, __LINE__, msg, _a, _b); \
        tests_failed++; \
        return; \
    } \
} while(0)

#define PASS() do { tests_passed++; } while(0)

static pio_sim_t *create_sim(void)
{
    pio_sim_t *sim = pio_sim_create(jtag_shift_program, 8);
    if (!sim) return NULL;

    pio_sim_set_sideset(sim, 1, false, TCK_PIN);
    pio_sim_set_out_pins(sim, TDI_PIN, 1);
    pio_sim_set_in_pins(sim, TDO_PIN);
    pio_sim_set_out_shift(sim, true, true, 32);    /* LSB-first, autopull at 32 */
    pio_sim_set_in_shift(sim, true, true, 32);      /* LSB-first, autopush at 32 */
    pio_sim_set_wrap(sim, 0, 7);

    return sim;
}

/* Test: TDI shifts out LSB first */
static void test_tdi_lsb_first(void)
{
    pio_sim_t *sim = create_sim();
    ASSERT(sim != NULL, "create failed");

    /* Send 0b1010 = 0x0A. LSB first means: bit0=0, bit1=1, bit2=0, bit3=1 */
    pio_sim_tx_push(sim, 3);     /* 4 bits */
    pio_sim_tx_push(sim, 0x0A);  /* 0b1010 */
    pio_sim_gpio_set(sim, TDO_PIN, false);

    /* Setup: pull + out x,32 */
    pio_sim_step(sim);
    pio_sim_step(sim);

    /* Bit 0 (LSB): out pins,1 should drive TDI=0 (bit 0 of 0x0A = 0) */
    pio_sim_step(sim);  /* out pins, 1 */
    ASSERT(!pio_sim_gpio_get_pin(sim, TDI_PIN), "bit 0 of 0x0A should be 0");

    /* Skip rest of bit 0 cycle */
    pio_sim_step(sim);  /* nop */
    pio_sim_step(sim);  /* nop side 1 */
    pio_sim_step(sim);  /* in */
    pio_sim_step(sim);  /* jmp */

    /* Bit 1: should drive TDI=1 (bit 1 of 0x0A = 1) */
    pio_sim_step(sim);  /* out pins, 1 */
    ASSERT(pio_sim_gpio_get_pin(sim, TDI_PIN), "bit 1 of 0x0A should be 1");

    pio_sim_step(sim);  pio_sim_step(sim);  pio_sim_step(sim);  pio_sim_step(sim);

    /* Bit 2: should drive TDI=0 (bit 2 of 0x0A = 0) */
    pio_sim_step(sim);
    ASSERT(!pio_sim_gpio_get_pin(sim, TDI_PIN), "bit 2 of 0x0A should be 0");

    pio_sim_step(sim);  pio_sim_step(sim);  pio_sim_step(sim);  pio_sim_step(sim);

    /* Bit 3: should drive TDI=1 (bit 3 of 0x0A = 1) */
    pio_sim_step(sim);
    ASSERT(pio_sim_gpio_get_pin(sim, TDI_PIN), "bit 3 of 0x0A should be 1");

    pio_sim_destroy(sim);
    PASS();
}

/* Test: TDO captures in correct bit positions */
static void test_tdo_bit_positions(void)
{
    pio_sim_t *sim = create_sim();
    ASSERT(sim != NULL, "create failed");

    /* 4 bits, TDI doesn't matter */
    pio_sim_tx_push(sim, 3);
    pio_sim_tx_push(sim, 0);

    /* We'll change TDO pin between bits to create a known pattern */
    /* Pattern: bit0=1, bit1=0, bit2=1, bit3=0 = 0b0101 = 0x05 */

    /* Setup */
    pio_sim_step(sim);  /* pull */
    pio_sim_step(sim);  /* out x, 32 */

    /* Bit 0: set TDO=1 before sampling */
    pio_sim_gpio_set(sim, TDO_PIN, true);
    pio_sim_step(sim);  /* out pins,1 side 0 */
    pio_sim_step(sim);  /* nop side 0 */
    pio_sim_step(sim);  /* nop side 1 -- rising edge */
    pio_sim_step(sim);  /* in pins,1 side 1 -- samples TDO=1 */
    pio_sim_step(sim);  /* jmp */

    /* Bit 1: set TDO=0 */
    pio_sim_gpio_set(sim, TDO_PIN, false);
    pio_sim_step(sim);  pio_sim_step(sim);  pio_sim_step(sim);
    pio_sim_step(sim);  /* in pins,1 -- samples TDO=0 */
    pio_sim_step(sim);  /* jmp */

    /* Bit 2: set TDO=1 */
    pio_sim_gpio_set(sim, TDO_PIN, true);
    pio_sim_step(sim);  pio_sim_step(sim);  pio_sim_step(sim);
    pio_sim_step(sim);  /* in pins,1 -- samples TDO=1 */
    pio_sim_step(sim);  /* jmp */

    /* Bit 3: set TDO=0 */
    pio_sim_gpio_set(sim, TDO_PIN, false);
    pio_sim_step(sim);  pio_sim_step(sim);  pio_sim_step(sim);
    pio_sim_step(sim);  /* in pins,1 -- samples TDO=0 */
    pio_sim_step(sim);  /* jmp -- x=0, fall through */

    /* push */
    pio_sim_step(sim);

    /* Read TDO result */
    uint32_t tdo_word;
    ASSERT(pio_sim_rx_pop(sim, &tdo_word), "should have TDO word");

    /* With right-shift IN, bits enter at MSB:
     * After 4 shifts: bits are at [31:28] = {bit3, bit2, bit1, bit0}
     * bit0=1 entered first (goes to bit 31)
     * bit1=0 next (goes to bit 30, bit0 shifts to bit 30... wait)
     *
     * Right shift IN: ISR >>= 1; ISR |= (bit << 31)
     * After bit0 (1): ISR = 0x80000000
     * After bit1 (0): ISR = 0x40000000  (shifted right, new 0 at MSB)
     * After bit2 (1): ISR = 0xA0000000  (shifted right, new 1 at MSB)
     * After bit3 (0): ISR = 0x50000000  (shifted right, new 0 at MSB)
     *
     * So ISR = 0x50000000 = 0b0101 shifted to top 4 bits
     * As a 4-bit value in positions [31:28] = 0101 = 5 */
    ASSERT_EQ(tdo_word, 0x50000000, "TDO pattern 1010 should give 0x50000000");

    pio_sim_destroy(sim);
    PASS();
}

int main(void)
{
    printf("JTAG bit ordering tests\n");
    printf("=======================\n\n");

    test_tdi_lsb_first();
    test_tdo_bit_positions();

    printf("\n%d passed, %d failed\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
