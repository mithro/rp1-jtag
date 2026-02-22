/*
 * test_pio_sim.c - Self-tests for the PIO simulator
 *
 * Validates the simulator itself against known PIO behavior before
 * trusting it for JTAG program tests.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "pio_sim.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_ASSERT(cond, msg) do { \
    if (!(cond)) { \
        fprintf(stderr, "FAIL [%s:%d]: %s\n", __func__, __LINE__, msg); \
        tests_failed++; \
        return; \
    } \
} while(0)

#define TEST_ASSERT_EQ(a, b, msg) do { \
    uint32_t _a = (uint32_t)(a), _b = (uint32_t)(b); \
    if (_a != _b) { \
        fprintf(stderr, "FAIL [%s:%d]: %s (got 0x%08x, expected 0x%08x)\n", \
                __func__, __LINE__, msg, _a, _b); \
        tests_failed++; \
        return; \
    } \
} while(0)

#define TEST_PASS() do { tests_passed++; } while(0)

/* ---- FIFO tests ---- */

static void test_fifo_push_pop(void)
{
    /* Simple loopback: MOV x, ::y (copy Y to X, just using set/mov) */
    /* Actually, test the FIFO by using PULL + PUSH */
    uint16_t prog[] = {
        0x80a0, /* pull block  side 0 */
        0x8020, /* push block  side 0 */
    };

    pio_sim_t *sim = pio_sim_create(prog, 2);
    TEST_ASSERT(sim != NULL, "create failed");

    pio_sim_set_sideset(sim, 1, false, 0);
    pio_sim_set_wrap(sim, 0, 1);

    /* Push a word into TX FIFO */
    TEST_ASSERT(pio_sim_tx_push(sim, 0xDEADBEEF), "tx push failed");

    /* Execute pull -- should move TX FIFO word into OSR */
    TEST_ASSERT(pio_sim_step(sim), "pull stalled");

    /* Execute push -- should move ISR (0) to RX FIFO
     * Note: this pushes ISR which is 0, not OSR */
    TEST_ASSERT(pio_sim_step(sim), "push stalled");

    uint32_t word;
    TEST_ASSERT(pio_sim_rx_pop(sim, &word), "rx pop failed");
    TEST_ASSERT_EQ(word, 0, "push sends ISR which is 0");

    pio_sim_destroy(sim);
    TEST_PASS();
}

static void test_fifo_full_stall(void)
{
    /* PULL block should stall when TX FIFO is empty */
    uint16_t prog[] = {
        0x80a0, /* pull block side 0 */
    };

    pio_sim_t *sim = pio_sim_create(prog, 1);
    TEST_ASSERT(sim != NULL, "create failed");
    pio_sim_set_sideset(sim, 1, false, 0);

    /* TX FIFO is empty -- pull should stall */
    TEST_ASSERT(!pio_sim_step(sim), "should stall on empty TX FIFO");
    TEST_ASSERT(sim->stalled, "stalled flag should be set");
    TEST_ASSERT_EQ(sim->pc, 0, "PC should not advance on stall");

    /* Now push data and retry */
    pio_sim_tx_push(sim, 0x12345678);
    TEST_ASSERT(pio_sim_step(sim), "should succeed after TX push");
    TEST_ASSERT_EQ(sim->osr, 0x12345678, "OSR should have pulled value");

    pio_sim_destroy(sim);
    TEST_PASS();
}

/* ---- Shift register tests ---- */

static void test_out_shift_right(void)
{
    /* Test OUT x, 8 with right shift (LSB-first) */
    uint16_t prog[] = {
        0x80a0, /* pull block side 0 */
        0x6028, /* out x, 8   side 0 */
    };

    pio_sim_t *sim = pio_sim_create(prog, 2);
    TEST_ASSERT(sim != NULL, "create failed");
    pio_sim_set_sideset(sim, 1, false, 0);
    pio_sim_set_out_shift(sim, true, false, 32);  /* shift right, no autopull */

    pio_sim_tx_push(sim, 0xAABBCCDD);
    pio_sim_step(sim);  /* pull */
    pio_sim_step(sim);  /* out x, 8 */

    /* With right shift, lowest 8 bits come out first */
    TEST_ASSERT_EQ(sim->x, 0xDD, "OUT x,8 should get LSB byte 0xDD");
    /* OSR should have shifted right by 8: 0x00AABBCC */
    TEST_ASSERT_EQ(sim->osr, 0x00AABBCC, "OSR should shift right by 8");

    pio_sim_destroy(sim);
    TEST_PASS();
}

static void test_in_shift_right(void)
{
    /* Test IN pins, 1 with right shift -- new bit enters at MSB */
    uint16_t prog[] = {
        0x4001, /* in pins, 1 side 0 (no side-set for simplicity) */
    };

    pio_sim_t *sim = pio_sim_create(prog, 1);
    TEST_ASSERT(sim != NULL, "create failed");
    pio_sim_set_in_pins(sim, 5);
    pio_sim_set_in_shift(sim, true, false, 32);

    /* Set pin 5 high */
    pio_sim_gpio_set(sim, 5, true);
    pio_sim_step(sim);

    /* With right shift, 1 bit enters at bit 31 */
    TEST_ASSERT_EQ(sim->isr, 0x80000000, "IN 1 bit right-shift enters at MSB");
    TEST_ASSERT_EQ(sim->isr_shift_count, 1, "shift count should be 1");

    pio_sim_destroy(sim);
    TEST_PASS();
}

/* ---- Side-set tests ---- */

static void test_sideset(void)
{
    /* NOP side 0, NOP side 1 -- should toggle GPIO */
    uint16_t prog[] = {
        0xa042, /* nop side 0 */
        0xb042, /* nop side 1 */
    };

    pio_sim_t *sim = pio_sim_create(prog, 2);
    TEST_ASSERT(sim != NULL, "create failed");
    pio_sim_set_sideset(sim, 1, false, 4);  /* side-set on GPIO 4 */

    /* Execute NOP side 0 */
    pio_sim_step(sim);
    TEST_ASSERT(!pio_sim_gpio_get_pin(sim, 4), "GPIO 4 should be low after side 0");

    /* Execute NOP side 1 */
    pio_sim_step(sim);
    TEST_ASSERT(pio_sim_gpio_get_pin(sim, 4), "GPIO 4 should be high after side 1");

    pio_sim_destroy(sim);
    TEST_PASS();
}

/* ---- JMP tests ---- */

static void test_jmp_x_dec(void)
{
    /* Count down from 2: 3 iterations (2, 1, 0) */
    uint16_t prog[] = {
        0x80a0, /* 0: pull block side 0 */
        0x6020, /* 1: out x, 32  side 0 */
        0xa042, /* 2: nop        side 0 (loop body) */
        0x0042, /* 3: jmp x--, 2 side 0 */
        0xa042, /* 4: nop        side 0 (after loop) */
    };

    pio_sim_t *sim = pio_sim_create(prog, 5);
    TEST_ASSERT(sim != NULL, "create failed");
    pio_sim_set_sideset(sim, 1, false, 0);
    pio_sim_set_out_shift(sim, true, false, 32);

    pio_sim_tx_push(sim, 2);  /* count = 2 */
    pio_sim_step(sim);  /* pull */
    pio_sim_step(sim);  /* out x, 32  -- x = 2 */

    /* Iteration 1: x=2, nop, jmp x-- 2 (x becomes 1, jump taken) */
    pio_sim_step(sim);  /* nop at addr 2 */
    TEST_ASSERT_EQ(sim->pc, 3, "should be at jmp");
    pio_sim_step(sim);  /* jmp x--, 2 -- x=2!=0, take, x becomes 1 */
    TEST_ASSERT_EQ(sim->pc, 2, "should jump back to 2");
    TEST_ASSERT_EQ(sim->x, 1, "x should be 1");

    /* Iteration 2: x=1 */
    pio_sim_step(sim);  /* nop */
    pio_sim_step(sim);  /* jmp x--, 2 -- x=1!=0, take, x becomes 0 */
    TEST_ASSERT_EQ(sim->pc, 2, "should jump back to 2");
    TEST_ASSERT_EQ(sim->x, 0, "x should be 0");

    /* Iteration 3: x=0 */
    pio_sim_step(sim);  /* nop */
    pio_sim_step(sim);  /* jmp x--, 2 -- x=0, don't take, x wraps to 0xFFFFFFFF */
    TEST_ASSERT_EQ(sim->pc, 4, "should fall through after x=0");

    pio_sim_destroy(sim);
    TEST_PASS();
}

/* ---- WAIT instruction test ---- */

static void test_wait_gpio(void)
{
    /* wait 1 gpio 5 ; wait 0 gpio 5 */
    uint16_t prog[] = {
        0x2085, /* wait 1 gpio 5 */
        0x2005, /* wait 0 gpio 5 */
    };

    pio_sim_t *sim = pio_sim_create(prog, 2);
    TEST_ASSERT(sim != NULL, "create failed");

    /* GPIO 5 is low -- wait 1 should stall */
    pio_sim_gpio_set(sim, 5, false);
    TEST_ASSERT(!pio_sim_step(sim), "should stall waiting for GPIO 5 high");
    TEST_ASSERT_EQ(sim->pc, 0, "PC should stay at 0");

    /* Set GPIO 5 high -- should proceed */
    pio_sim_gpio_set(sim, 5, true);
    TEST_ASSERT(pio_sim_step(sim), "should proceed with GPIO 5 high");
    TEST_ASSERT_EQ(sim->pc, 1, "PC should advance to 1");

    /* GPIO 5 is still high -- wait 0 should stall */
    TEST_ASSERT(!pio_sim_step(sim), "should stall waiting for GPIO 5 low");

    /* Set GPIO 5 low -- should proceed */
    pio_sim_gpio_set(sim, 5, false);
    TEST_ASSERT(pio_sim_step(sim), "should proceed with GPIO 5 low");

    pio_sim_destroy(sim);
    TEST_PASS();
}

/* ---- Autopush test ---- */

static void test_autopush(void)
{
    /* IN y, 1 with autopush at 32. After 32 shifts, ISR auto-pushes. */
    uint16_t prog[] = {
        0x4041, /* in y, 1 */
    };

    pio_sim_t *sim = pio_sim_create(prog, 1);
    TEST_ASSERT(sim != NULL, "create failed");
    pio_sim_set_wrap(sim, 0, 0);
    pio_sim_set_in_shift(sim, true, true, 32);  /* right shift, autopush at 32 */

    sim->y = 1;

    /* Shift 32 bits of 1 into ISR */
    for (int i = 0; i < 32; i++) {
        TEST_ASSERT(pio_sim_step(sim), "step should succeed");
    }

    /* Autopush should have fired, pushing ISR to RX FIFO */
    uint32_t word;
    TEST_ASSERT(pio_sim_rx_pop(sim, &word), "rx pop should succeed after autopush");
    TEST_ASSERT_EQ(word, 0xFFFFFFFF, "32 bits of 1 should give 0xFFFFFFFF");

    pio_sim_destroy(sim);
    TEST_PASS();
}

/* ---- OUT pins test ---- */

static void test_out_pins(void)
{
    /* OUT pins, 1 with out_base=27 */
    uint16_t prog[] = {
        0x80a0, /* pull block side 0 */
        0x6020, /* out x, 32  side 0 -- load count (discard) */
        0x6001, /* out pins, 1 side 0 */
    };

    pio_sim_t *sim = pio_sim_create(prog, 3);
    TEST_ASSERT(sim != NULL, "create failed");
    pio_sim_set_sideset(sim, 1, false, 4);   /* TCK on GPIO 4 */
    pio_sim_set_out_pins(sim, 27, 1);        /* TDI on GPIO 27 */
    pio_sim_set_out_shift(sim, true, false, 32);

    /* Push count=0 (not used here) and data=0b10101 */
    pio_sim_tx_push(sim, 0);          /* count */
    pio_sim_tx_push(sim, 0x00000015); /* TDI data: 10101 in binary */
    pio_sim_step(sim);  /* pull -- gets count word */
    pio_sim_step(sim);  /* out x, 32 -- discards count */
    pio_sim_step(sim);  /* out pins, 1 -- drives LSB of data to GPIO 27 */

    /* LSB of 0x15 = 1, so GPIO 27 should be high */
    TEST_ASSERT(pio_sim_gpio_get_pin(sim, 27), "GPIO 27 should be high (TDI=1)");

    /* TCK (GPIO 4) should be low (side 0) */
    TEST_ASSERT(!pio_sim_gpio_get_pin(sim, 4), "GPIO 4 should be low (TCK side 0)");

    pio_sim_destroy(sim);
    TEST_PASS();
}

/* ---- Wrap test ---- */

static void test_wrap(void)
{
    uint16_t prog[] = {
        0xa042, /* 0: nop side 0 */
        0xb042, /* 1: nop side 1 */
    };

    pio_sim_t *sim = pio_sim_create(prog, 2);
    TEST_ASSERT(sim != NULL, "create failed");
    pio_sim_set_sideset(sim, 1, false, 0);
    pio_sim_set_wrap(sim, 0, 1);

    pio_sim_step(sim);  /* addr 0: nop side 0 */
    TEST_ASSERT_EQ(sim->pc, 1, "pc should be 1");

    pio_sim_step(sim);  /* addr 1: nop side 1 -- wraps to 0 */
    TEST_ASSERT_EQ(sim->pc, 0, "pc should wrap back to 0");

    pio_sim_destroy(sim);
    TEST_PASS();
}

/* ---- MOV test ---- */

static void test_mov_y_to_x(void)
{
    /* SET Y, 7 then MOV X, Y */
    uint16_t prog[] = {
        0xe047, /* set y, 7 */
        0xa021, /* mov x, y  (dest=001(x), op=00(none), src=010(y)) */
    };

    pio_sim_t *sim = pio_sim_create(prog, 2);
    TEST_ASSERT(sim != NULL, "create failed");

    pio_sim_step(sim);  /* set y, 7 */
    TEST_ASSERT_EQ(sim->y, 7, "Y should be 7");

    pio_sim_step(sim);  /* mov x, y */
    TEST_ASSERT_EQ(sim->x, 7, "X should be 7 (copied from Y)");

    pio_sim_destroy(sim);
    TEST_PASS();
}

/* ---- Main ---- */

int main(void)
{
    printf("PIO simulator self-tests\n");
    printf("========================\n\n");

    test_fifo_push_pop();
    test_fifo_full_stall();
    test_out_shift_right();
    test_in_shift_right();
    test_sideset();
    test_jmp_x_dec();
    test_wait_gpio();
    test_autopush();
    test_out_pins();
    test_wrap();
    test_mov_y_to_x();

    printf("\n%d passed, %d failed\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
