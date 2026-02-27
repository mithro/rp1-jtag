/*
 * rp1_jtag.h - Public API for RP1 PIO JTAG library
 *
 * Phase 2 architecture: Two PIO state machines with bulk DMA.
 *   SM0: TDI/TDO/TCK (jtag_shift program, 4 instr/bit counted loop)
 *   SM1: TMS (jtag_tms program, synchronized to SM0's TCK via GPIO wait)
 * Three concurrent pthreads handle DMA: SM0 TX, SM0 RX, SM1 TX.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RP1_JTAG_H
#define RP1_JTAG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct rp1_jtag rp1_jtag_t;

/* GPIO pin configuration. Set unused pins to -1. */
typedef struct {
    int tck;     /* TCK output pin (BCM GPIO number) */
    int tms;     /* TMS output pin */
    int tdi;     /* TDI output pin */
    int tdo;     /* TDO input pin */
    int srst;    /* SRST output pin, -1 if unused */
    int trst;    /* TRST output pin, -1 if unused */
} rp1_jtag_pins_t;

/* Error codes (returned as negative values) */
#define RP1_JTAG_OK          0
#define RP1_JTAG_ERR_NODEV  -1  /* /dev/pio0 not found */
#define RP1_JTAG_ERR_BUSY   -2  /* No free state machines */
#define RP1_JTAG_ERR_PIN    -3  /* GPIO pin conflict or invalid */
#define RP1_JTAG_ERR_IO     -4  /* PIOLib ioctl/RPC failure */
#define RP1_JTAG_ERR_PARAM  -5  /* Invalid parameter */

/* Lifecycle */
rp1_jtag_t *rp1_jtag_init(const rp1_jtag_pins_t *pins);
void         rp1_jtag_close(rp1_jtag_t *jtag);

/*
 * Initialize in loopback mode (no GPIO, PIO internal loopback).
 * Uses the jtag_loopback PIO program for testing without hardware.
 */
rp1_jtag_t *rp1_jtag_init_loopback(void);

/* Clock configuration */
int      rp1_jtag_set_freq(rp1_jtag_t *jtag, uint32_t freq_hz);
uint32_t rp1_jtag_get_freq(rp1_jtag_t *jtag);

/*
 * Core shift operation (XVC-compatible).
 *
 * Shifts num_bits through the JTAG chain. For each bit:
 *   - TDI is driven before the rising edge of TCK (adequate setup time)
 *   - TMS is driven before the rising edge of TCK
 *   - TDO is sampled after the falling edge of TCK (valid from target)
 *
 * Internally, the library scans the TMS vector and splits it into contiguous
 * runs of constant TMS value. Each run becomes one PIO transfer with TMS set
 * via GPIO. This means the common case (long runs of TMS=0 for DR/IR shifts)
 * uses fast PIO bulk transfer, while TMS transitions (short TAP navigation
 * sequences) are handled correctly but with per-run GPIO overhead (~10us/run).
 *
 * tms/tdi: input bit vectors, LSB-first, ceil(num_bits/8) bytes each
 * tdo:     output bit vector, may be NULL if TDO capture not needed
 * Returns 0 on success, negative error code on failure.
 */
int rp1_jtag_shift(rp1_jtag_t *jtag, uint32_t num_bits,
                   const uint8_t *tms, const uint8_t *tdi,
                   uint8_t *tdo);

/*
 * Clock toggle with constant TMS/TDI (for Run-Test/Idle, etc).
 * Returns 0 on success, negative error code on failure.
 */
int rp1_jtag_toggle_clk(rp1_jtag_t *jtag, uint32_t num_clocks,
                        bool tms, bool tdi);

/* Reset signals. Active-low: 0 = asserted, 1 = deasserted, -1 = unchanged */
int rp1_jtag_reset(rp1_jtag_t *jtag, int srst, int trst);

#ifdef __cplusplus
}
#endif

#endif /* RP1_JTAG_H */
