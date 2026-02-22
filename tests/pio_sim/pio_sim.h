/*
 * pio_sim.h - Minimal PIO simulator for testing JTAG programs
 *
 * Simulates the RP2040/RP1 PIO instruction set with enough fidelity
 * to verify our JTAG PIO programs: instruction decoding, side-set,
 * autopush/autopull, TX/RX FIFOs, and GPIO state tracking.
 *
 * Only implements the ~8 opcodes our programs use. Not a general-purpose
 * PIO emulator.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PIO_SIM_H
#define PIO_SIM_H

#include <stdint.h>
#include <stdbool.h>

/* FIFO depth (RP1 has 8-deep FIFOs per SM) */
#define PIO_SIM_FIFO_DEPTH 8

/* Maximum program length (RP1 has 32-instruction shared memory) */
#define PIO_SIM_MAX_PROGRAM 32

/* Maximum GPIO pins tracked */
#define PIO_SIM_MAX_GPIO 32

typedef struct pio_sim pio_sim_t;

/* FIFO queue */
typedef struct {
    uint32_t data[PIO_SIM_FIFO_DEPTH];
    int head;
    int tail;
    int count;
} pio_sim_fifo_t;

/* Simulator state */
struct pio_sim {
    /* Program memory */
    uint16_t program[PIO_SIM_MAX_PROGRAM];
    int program_len;

    /* Program counter */
    int pc;

    /* Wrap addresses */
    int wrap_target;
    int wrap_bottom;

    /* Scratch registers */
    uint32_t x;
    uint32_t y;

    /* Shift registers */
    uint32_t osr;           /* Output shift register */
    int osr_shift_count;    /* Bits shifted out of OSR */
    uint32_t isr;           /* Input shift register */
    int isr_shift_count;    /* Bits shifted into ISR */

    /* FIFOs */
    pio_sim_fifo_t tx_fifo;
    pio_sim_fifo_t rx_fifo;

    /* Autopush/autopull */
    bool autopush;
    int autopush_threshold;  /* Push when ISR has this many bits */
    bool autopull;
    int autopull_threshold;  /* Pull when OSR has shifted out this many bits */

    /* Shift direction (true = shift right / LSB-first) */
    bool out_shift_right;
    bool in_shift_right;

    /* Side-set configuration */
    int sideset_count;       /* Number of side-set bits (0-5) */
    bool sideset_opt;        /* Optional side-set */
    int sideset_base;        /* Base GPIO for side-set */

    /* Pin configuration */
    int out_base;
    int out_count;
    int in_base;
    int in_count;
    int set_base;
    int set_count;

    /* GPIO state (simulated) */
    uint32_t gpio_out;       /* Output pin values */
    uint32_t gpio_dir;       /* Pin directions (1 = output) */
    uint32_t gpio_in;        /* Input pin values (set by test) */

    /* Execution state */
    bool stalled;            /* True if SM is stalled (FIFO empty/full) */
    uint64_t cycle_count;    /* Total cycles executed */

    /* Status */
    bool halted;             /* True if program has completed */
};

/* Create a new simulator instance with a PIO program. */
pio_sim_t *pio_sim_create(const uint16_t *program, int len);

/* Configure wrap addresses. */
void pio_sim_set_wrap(pio_sim_t *sim, int wrap_target, int wrap_bottom);

/* Configure side-set. */
void pio_sim_set_sideset(pio_sim_t *sim, int count, bool opt, int base);

/* Configure OUT pins. */
void pio_sim_set_out_pins(pio_sim_t *sim, int base, int count);

/* Configure IN pins. */
void pio_sim_set_in_pins(pio_sim_t *sim, int base);

/* Configure SET pins. */
void pio_sim_set_set_pins(pio_sim_t *sim, int base, int count);

/* Configure shift directions and auto-push/pull. */
void pio_sim_set_out_shift(pio_sim_t *sim, bool shift_right,
                           bool autopull, int threshold);
void pio_sim_set_in_shift(pio_sim_t *sim, bool shift_right,
                          bool autopush, int threshold);

/* Push a word into the TX FIFO. Returns false if FIFO full. */
bool pio_sim_tx_push(pio_sim_t *sim, uint32_t word);

/* Pop a word from the RX FIFO. Returns false if FIFO empty. */
bool pio_sim_rx_pop(pio_sim_t *sim, uint32_t *word);

/* Check FIFO status. */
bool pio_sim_tx_full(pio_sim_t *sim);
bool pio_sim_tx_empty(pio_sim_t *sim);
bool pio_sim_rx_full(pio_sim_t *sim);
bool pio_sim_rx_empty(pio_sim_t *sim);

/* Execute one PIO instruction. Returns false if stalled. */
bool pio_sim_step(pio_sim_t *sim);

/* Execute up to max_cycles instructions. Returns cycles actually executed. */
int pio_sim_run(pio_sim_t *sim, int max_cycles);

/* Read all GPIO output states as a bitmask. */
uint32_t pio_sim_gpio_get(pio_sim_t *sim);

/* Set input pin values (simulates external signal). */
void pio_sim_gpio_set(pio_sim_t *sim, int pin, bool value);

/* Read a specific output pin value. */
bool pio_sim_gpio_get_pin(pio_sim_t *sim, int pin);

/* Destroy simulator instance. */
void pio_sim_destroy(pio_sim_t *sim);

#endif /* PIO_SIM_H */
