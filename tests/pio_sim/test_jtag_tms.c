/*
 * test_jtag_tms.c - Test the jtag_tms PIO program via simulator
 *
 * Verifies:
 *   - TMS bits read from TX FIFO via autopull
 *   - TMS output synchronized to TCK via wait gpio
 *   - Stalls correctly when TCK is in wrong state
 *   - Multiple TMS bits output with simulated TCK toggling
 *
 * The jtag_tms program runs on SM1 alongside jtag_shift on SM0.
 * It uses wait gpio to synchronize TMS output to SM0's TCK signal.
 * TCK (GPIO4) is set via gpio_in to simulate SM0's sideset output.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "pio_sim.h"
#include <stdio.h>

/* From pioasm-generated header (instruction words only) */
static const uint16_t jtag_tms_program[] = {
    0x2004, /*  0: wait   0 gpio, 4 */
    0x6001, /*  1: out    pins, 1   */
    0x2084, /*  2: wait   1 gpio, 4 */
};

#define TCK_PIN 4
#define TMS_PIN 17

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

static pio_sim_t *create_tms_sim(void)
{
    pio_sim_t *sim = pio_sim_create(jtag_tms_program, 3);
    if (!sim) return NULL;

    /* No sideset for jtag_tms */
    pio_sim_set_out_pins(sim, TMS_PIN, 1);
    pio_sim_set_out_shift(sim, true, true, 32);    /* LSB-first, autopull at 32 */
    pio_sim_set_in_shift(sim, true, false, 32);     /* Not used, no autopush */
    pio_sim_set_wrap(sim, 0, 2);

    return sim;
}

/* Test: stalls on wait 0 gpio 4 when TCK is HIGH */
static void test_stall_wait_tck_low(void)
{
    pio_sim_t *sim = create_tms_sim();
    ASSERT(sim != NULL, "create failed");

    pio_sim_tx_push(sim, 0x00000001);  /* TMS data */

    /* Set TCK (gpio_in bit 4) HIGH -- wait 0 gpio 4 should stall */
    pio_sim_gpio_set(sim, TCK_PIN, true);
    ASSERT(!pio_sim_step(sim), "should stall on wait 0 gpio 4 when TCK is HIGH");
    ASSERT_EQ(sim->pc, 0, "PC should stay at 0 (stalled)");

    /* Set TCK LOW -- should proceed past wait */
    pio_sim_gpio_set(sim, TCK_PIN, false);
    ASSERT(pio_sim_step(sim), "should proceed with TCK LOW");
    ASSERT_EQ(sim->pc, 1, "PC should advance to 1 (out pins)");

    pio_sim_destroy(sim);
    PASS();
}

/* Test: stalls on wait 1 gpio 4 when TCK is LOW */
static void test_stall_wait_tck_high(void)
{
    pio_sim_t *sim = create_tms_sim();
    ASSERT(sim != NULL, "create failed");

    pio_sim_tx_push(sim, 0x00000001);  /* TMS data */

    /* Start with TCK LOW so wait 0 gpio 4 passes, then out pins */
    pio_sim_gpio_set(sim, TCK_PIN, false);
    ASSERT(pio_sim_step(sim), "wait 0 gpio 4 should pass with TCK LOW");
    ASSERT(pio_sim_step(sim), "out pins,1 should succeed");

    /* Now at PC=2: wait 1 gpio 4. TCK is still LOW -- should stall */
    ASSERT(!pio_sim_step(sim), "should stall on wait 1 gpio 4 when TCK is LOW");
    ASSERT_EQ(sim->pc, 2, "PC should stay at 2 (stalled)");

    /* Set TCK HIGH -- should proceed */
    pio_sim_gpio_set(sim, TCK_PIN, true);
    ASSERT(pio_sim_step(sim), "should proceed with TCK HIGH");
    /* After wrap, PC should go back to 0 */
    ASSERT_EQ(sim->pc, 0, "PC should wrap back to 0");

    pio_sim_destroy(sim);
    PASS();
}

/* Test: single TMS bit output */
static void test_single_tms_bit(void)
{
    pio_sim_t *sim = create_tms_sim();
    ASSERT(sim != NULL, "create failed");

    /* TMS data = 1 (LSB) */
    pio_sim_tx_push(sim, 0x00000001);

    /* Simulate one TCK cycle: start LOW, go HIGH */
    pio_sim_gpio_set(sim, TCK_PIN, false);

    /* wait 0 gpio 4 -- passes (TCK LOW) */
    ASSERT(pio_sim_step(sim), "wait 0 should pass");

    /* out pins,1 -- drives TMS = bit 0 of data = 1 */
    ASSERT(pio_sim_step(sim), "out pins should succeed");
    ASSERT(pio_sim_gpio_get_pin(sim, TMS_PIN), "TMS should be 1 (bit 0 of 0x01)");

    /* wait 1 gpio 4 -- TCK still LOW, stalls */
    ASSERT(!pio_sim_step(sim), "should stall waiting for TCK HIGH");

    /* Set TCK HIGH -- wait completes */
    pio_sim_gpio_set(sim, TCK_PIN, true);
    ASSERT(pio_sim_step(sim), "wait 1 should pass with TCK HIGH");

    pio_sim_destroy(sim);
    PASS();
}

/* Test: multiple TMS bits with simulated TCK toggling */
static void test_multi_tms_bits(void)
{
    pio_sim_t *sim = create_tms_sim();
    ASSERT(sim != NULL, "create failed");

    /* TMS data = 0b1010 = 0x0A (LSB first: bit0=0, bit1=1, bit2=0, bit3=1) */
    pio_sim_tx_push(sim, 0x0000000A);

    int expected_tms[] = { 0, 1, 0, 1 };

    for (int bit = 0; bit < 4; bit++) {
        /* Simulate SM0 TCK LOW phase */
        pio_sim_gpio_set(sim, TCK_PIN, false);

        /* wait 0 gpio 4 -- should pass (TCK LOW) */
        ASSERT(pio_sim_step(sim), "wait 0 should pass for bit");

        /* out pins,1 -- drive TMS */
        ASSERT(pio_sim_step(sim), "out pins should succeed for bit");

        bool tms_val = pio_sim_gpio_get_pin(sim, TMS_PIN);
        if (expected_tms[bit]) {
            ASSERT(tms_val, "TMS should be 1");
        } else {
            ASSERT(!tms_val, "TMS should be 0");
        }

        /* Simulate SM0 TCK HIGH phase */
        pio_sim_gpio_set(sim, TCK_PIN, true);

        /* wait 1 gpio 4 -- should pass (TCK HIGH) */
        ASSERT(pio_sim_step(sim), "wait 1 should pass for bit");
    }

    pio_sim_destroy(sim);
    PASS();
}

/* Test: TMS stalls when TCK stops toggling (SM0 finished) */
static void test_stall_when_tck_stops(void)
{
    pio_sim_t *sim = create_tms_sim();
    ASSERT(sim != NULL, "create failed");

    /* TMS data for 2 bits */
    pio_sim_tx_push(sim, 0x00000003);  /* TMS = 0b11 */

    /* Process one complete TMS bit */
    pio_sim_gpio_set(sim, TCK_PIN, false);
    ASSERT(pio_sim_step(sim), "wait 0 pass");
    ASSERT(pio_sim_step(sim), "out pins pass");
    pio_sim_gpio_set(sim, TCK_PIN, true);
    ASSERT(pio_sim_step(sim), "wait 1 pass");

    /* Now simulate SM0 stopping: TCK stays HIGH (SM0's last side 1)
     * SM1 wraps to wait 0 gpio 4, which stalls because TCK is HIGH */
    ASSERT(!pio_sim_step(sim), "should stall on wait 0 gpio 4 (TCK stuck HIGH)");
    ASSERT_EQ(sim->pc, 0, "PC should be at 0 (wait 0 gpio 4)");

    /* Verify it keeps stalling */
    ASSERT(!pio_sim_step(sim), "should keep stalling");
    ASSERT_EQ(sim->pc, 0, "PC should still be at 0");

    pio_sim_destroy(sim);
    PASS();
}

/* Test: autopull provides TMS bits from TX FIFO */
static void test_autopull_tms(void)
{
    pio_sim_t *sim = create_tms_sim();
    ASSERT(sim != NULL, "create failed");

    /* Push one word of TMS data (all 1s) */
    pio_sim_tx_push(sim, 0xFFFFFFFF);

    /* Run 8 TMS bits with TCK toggling */
    for (int bit = 0; bit < 8; bit++) {
        pio_sim_gpio_set(sim, TCK_PIN, false);
        ASSERT(pio_sim_step(sim), "wait 0 should pass");
        ASSERT(pio_sim_step(sim), "out pins should succeed");
        ASSERT(pio_sim_gpio_get_pin(sim, TMS_PIN), "TMS should be 1 (all-ones data)");
        pio_sim_gpio_set(sim, TCK_PIN, true);
        ASSERT(pio_sim_step(sim), "wait 1 should pass");
    }

    pio_sim_destroy(sim);
    PASS();
}

int main(void)
{
    printf("jtag_tms PIO program tests\n");
    printf("==========================\n\n");

    test_stall_wait_tck_low();
    test_stall_wait_tck_high();
    test_single_tms_bit();
    test_multi_tms_bits();
    test_stall_when_tck_stops();
    test_autopull_tms();

    printf("\n%d passed, %d failed\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
