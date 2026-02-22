/*
 * pio_backend.h - PIOLib abstraction layer (mockable for tests)
 *
 * This interface separates PIOLib calls from library logic, enabling
 * unit tests with a mock backend that simulates FIFO behavior without
 * requiring /dev/pio0 or real PIO hardware.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PIO_BACKEND_H
#define PIO_BACKEND_H

#include <stdint.h>
#include <stdbool.h>

typedef struct pio_backend pio_backend_t;

/* PIO program identifier */
typedef enum {
    PIO_PROG_JTAG_SHIFT,      /* jtag_shift.pio - counted loop */
    PIO_PROG_JTAG_SHIFT_FAST, /* jtag_shift_fast.pio - autopush/pull */
    PIO_PROG_JTAG_LOOPBACK,   /* jtag_loopback.pio - internal loopback */
    PIO_PROG_JTAG_TARGET,     /* jtag_target.pio - simulated target */
} pio_program_id_t;

/* Pin configuration for a PIO state machine */
typedef struct {
    int sideset_base;  /* SIDESET pin (TCK) */
    int out_base;      /* OUT pin (TDI) */
    int in_base;       /* IN pin (TDO) */
    float clk_div;     /* Clock divider */
} pio_sm_pins_t;

/*
 * Backend operations (vtable).
 *
 * All methods return 0 on success, negative error code on failure,
 * except where noted.
 */
typedef struct {
    /*
     * Initialize the PIO subsystem and claim a state machine.
     * Loads the specified PIO program and configures pins.
     */
    int (*init)(pio_backend_t *be, pio_program_id_t prog,
                const pio_sm_pins_t *pins);

    /* Release the state machine and PIO resources. */
    void (*close)(pio_backend_t *be);

    /* Set clock divider (PIO_CLK / freq_hz). */
    int (*set_clk_div)(pio_backend_t *be, float div);

    /*
     * Set a GPIO pin to a specific value (for TMS control).
     * pin: BCM GPIO number
     * value: 0 or 1
     */
    int (*gpio_set)(pio_backend_t *be, int pin, bool value);

    /*
     * Write a word to the TX FIFO (blocking).
     * Blocks until space is available.
     */
    int (*sm_put)(pio_backend_t *be, uint32_t word);

    /*
     * Read a word from the RX FIFO (blocking).
     * Blocks until data is available.
     */
    int (*sm_get)(pio_backend_t *be, uint32_t *word);

    /*
     * Check if TX FIFO has space for at least one word.
     * Returns true if space available.
     */
    bool (*tx_fifo_has_space)(pio_backend_t *be);

    /*
     * Check if RX FIFO has data available.
     * Returns true if data available.
     */
    bool (*rx_fifo_has_data)(pio_backend_t *be);

    /*
     * Enable/disable the state machine.
     */
    int (*sm_set_enabled)(pio_backend_t *be, bool enabled);

    /*
     * Load a different PIO program (for program switching).
     * The SM should be disabled first.
     */
    int (*load_program)(pio_backend_t *be, pio_program_id_t prog);

} pio_backend_ops_t;

struct pio_backend {
    const pio_backend_ops_t *ops;
    /* Backend-specific data follows (allocated by create function) */
};

/* Create the real RP1 PIOLib backend. Returns NULL on failure. */
pio_backend_t *pio_backend_rp1_create(void);

/* Create the mock backend for unit tests. Returns NULL on failure. */
pio_backend_t *pio_backend_mock_create(void);

/* Destroy a backend (calls close if needed, frees memory). */
void pio_backend_destroy(pio_backend_t *be);

/*
 * Mock backend test helpers.
 * Only valid when using the mock backend.
 */

/* Pre-load TDO data that mock will return from sm_get(). */
void pio_backend_mock_set_tdo_data(pio_backend_t *be,
                                   const uint32_t *words,
                                   int num_words);

/* Get the TDI data that was written to mock via sm_put(). */
int pio_backend_mock_get_tdi_data(pio_backend_t *be,
                                  uint32_t *words,
                                  int max_words);

/* Get the most recent GPIO state set via gpio_set(). */
bool pio_backend_mock_get_gpio(pio_backend_t *be, int pin);

/* Get number of sm_put() calls made. */
int pio_backend_mock_get_put_count(pio_backend_t *be);

/* Get number of sm_get() calls made. */
int pio_backend_mock_get_get_count(pio_backend_t *be);

/* Reset mock state (clear FIFOs, counters, GPIO state). */
void pio_backend_mock_reset(pio_backend_t *be);

#endif /* PIO_BACKEND_H */
