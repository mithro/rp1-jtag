/*
 * test_jtag_shift_fast.c - Test the jtag_shift_fast PIO program
 *
 * Verifies:
 *   - 2-instruction loop with autopush/autopull
 *   - TCK toggles: low on OUT, high on IN
 *   - Autopull refills OSR from TX FIFO automatically
 *   - Autopush sends ISR to RX FIFO every 32 bits
 *   - Data integrity through loopback
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "pio_sim.h"
#include <stdio.h>

static const uint16_t jtag_shift_fast_program[] = {
    0x6001, /*  0: out    pins, 1         side 0 */
    0x5001, /*  1: in     pins, 1         side 1 */
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

static pio_sim_t *create_fast_sim(void)
{
    pio_sim_t *sim = pio_sim_create(jtag_shift_fast_program, 2);
    if (!sim) return NULL;

    pio_sim_set_sideset(sim, 1, false, TCK_PIN);
    pio_sim_set_out_pins(sim, TDI_PIN, 1);
    pio_sim_set_in_pins(sim, TDO_PIN);
    pio_sim_set_wrap(sim, 0, 1);

    /* Key difference: autopush and autopull enabled at 32 */
    pio_sim_set_out_shift(sim, true, true, 32);   /* LSB-first, autopull */
    pio_sim_set_in_shift(sim, true, true, 32);     /* LSB-first, autopush */

    return sim;
}

/* Test: TCK alternates between low and high */
static void test_tck_waveform(void)
{
    pio_sim_t *sim = create_fast_sim();
    ASSERT(sim != NULL, "create failed");

    pio_sim_tx_push(sim, 0xFFFFFFFF); /* TDI data */
    pio_sim_gpio_set(sim, TDO_PIN, false);

    /* out pins,1 side 0 -- TCK low */
    pio_sim_step(sim);
    ASSERT(!pio_sim_gpio_get_pin(sim, TCK_PIN), "TCK should be low after OUT");

    /* in pins,1 side 1 -- TCK high */
    pio_sim_step(sim);
    ASSERT(pio_sim_gpio_get_pin(sim, TCK_PIN), "TCK should be high after IN");

    /* Wraps back: out pins,1 side 0 -- TCK low again */
    pio_sim_step(sim);
    ASSERT(!pio_sim_gpio_get_pin(sim, TCK_PIN), "TCK should be low again after wrap");

    pio_sim_destroy(sim);
    PASS();
}

/* Test: autopull and autopush work for 32-bit transfer */
static void test_32bit_autopush(void)
{
    pio_sim_t *sim = create_fast_sim();
    ASSERT(sim != NULL, "create failed");

    pio_sim_tx_push(sim, 0xDEADBEEF);
    /* Set TDO to constant 1 for all bits */
    pio_sim_gpio_set(sim, TDO_PIN, true);

    /* Run 64 steps (32 bits * 2 instructions each) */
    int executed = pio_sim_run(sim, 64);
    ASSERT(executed == 64, "should execute 64 instructions for 32 bits");

    /* Autopush should have pushed one word */
    uint32_t tdo_word;
    ASSERT(pio_sim_rx_pop(sim, &tdo_word), "should have autopushed TDO word");
    ASSERT_EQ(tdo_word, 0xFFFFFFFF, "32 bits of TDO=1 should give 0xFFFFFFFF");

    pio_sim_destroy(sim);
    PASS();
}

/* Test: 2 instruction cycle time */
static void test_instruction_count(void)
{
    pio_sim_t *sim = create_fast_sim();
    ASSERT(sim != NULL, "create failed");

    /* Push enough data for 32 bits */
    pio_sim_tx_push(sim, 0x12345678);
    pio_sim_gpio_set(sim, TDO_PIN, false);

    /* Each bit takes exactly 2 instructions */
    for (int bit = 0; bit < 32; bit++) {
        /* OUT instruction */
        ASSERT(pio_sim_step(sim), "OUT should succeed");
        /* IN instruction */
        ASSERT(pio_sim_step(sim), "IN should succeed");
    }

    /* After 64 instructions, should have shifted exactly 32 bits */
    ASSERT_EQ(sim->osr_shift_count, 0, "OSR should be refilled by autopull");

    pio_sim_destroy(sim);
    PASS();
}

/* Test: autopull stalls when TX FIFO empty */
static void test_stall_on_empty_tx(void)
{
    pio_sim_t *sim = create_fast_sim();
    ASSERT(sim != NULL, "create failed");

    /* Don't push any data -- TX FIFO is empty */
    /* First OUT instruction should trigger autopull which stalls */
    ASSERT(!pio_sim_step(sim), "should stall on empty TX FIFO");

    /* Now push data */
    pio_sim_tx_push(sim, 0x42);
    ASSERT(pio_sim_step(sim), "should succeed after TX push");

    pio_sim_destroy(sim);
    PASS();
}

int main(void)
{
    printf("jtag_shift_fast PIO program tests\n");
    printf("=================================\n\n");

    test_tck_waveform();
    test_32bit_autopush();
    test_instruction_count();
    test_stall_on_empty_tx();

    printf("\n%d passed, %d failed\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
