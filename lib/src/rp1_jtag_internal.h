/*
 * rp1_jtag_internal.h - Internal types and macros
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RP1_JTAG_INTERNAL_H
#define RP1_JTAG_INTERNAL_H

#include "rp1_jtag.h"
#include "pio_backend.h"

/* PIO clock frequency (RP1 PIO runs at 200 MHz) */
#define RP1_PIO_CLK_HZ  200000000

/* Default JTAG clock frequency */
#define DEFAULT_FREQ_HZ  6000000  /* 6 MHz */

/* Maximum valid BCM GPIO number on RPi 5 header */
#define MAX_GPIO_PIN  27

/* Number of bits per FIFO word */
#define BITS_PER_WORD  32

/* Operating mode */
typedef enum {
    MODE_JTAG,      /* Normal JTAG mode with GPIO pins */
    MODE_LOOPBACK,  /* Internal PIO loopback (no GPIO) */
} rp1_jtag_mode_t;

/* Instructions per bit for each PIO program */
#define INSTR_PER_BIT_COUNTED  5  /* jtag_shift / jtag_loopback */
#define INSTR_PER_BIT_FAST     2  /* jtag_shift_fast */

/* Main library context */
struct rp1_jtag {
    rp1_jtag_pins_t pins;
    rp1_jtag_mode_t mode;
    uint32_t freq_hz;
    int instr_per_bit;    /* PIO cycles per JTAG bit (5 counted, 2 fast) */
    pio_program_id_t current_program;  /* Currently loaded PIO program */
    bool dma_configured;  /* Whether DMA channels are set up */
    pio_backend_t *backend;
};

/*
 * TMS run descriptor: a contiguous run of constant TMS value.
 * Used by the TMS scanner to split bit vectors into PIO transfers.
 */
typedef struct {
    uint32_t start_bit;  /* Bit offset within the original vector */
    uint32_t num_bits;   /* Number of bits in this run */
    bool tms_value;      /* TMS value for this entire run */
} tms_run_t;

/*
 * Scan TMS vector and split into contiguous runs of constant value.
 *
 * tms:      Input TMS bit vector, LSB-first
 * num_bits: Total number of bits
 * runs:     Output array (caller allocates)
 * max_runs: Size of runs array
 *
 * Returns the number of runs found, or -1 if max_runs exceeded.
 */
int tms_scan_runs(const uint8_t *tms, uint32_t num_bits,
                  tms_run_t *runs, int max_runs);

/*
 * Extract a sub-range of bits from a bit vector into FIFO words.
 *
 * src:       Source bit vector, LSB-first
 * start_bit: First bit to extract
 * num_bits:  Number of bits to extract
 * words:     Output array of 32-bit words (caller allocates)
 *
 * Returns the number of words written.
 */
int bits_to_words(const uint8_t *src, uint32_t start_bit,
                  uint32_t num_bits, uint32_t *words);

/*
 * Unpack FIFO words back into a bit vector sub-range.
 *
 * words:     Input array of 32-bit words from RX FIFO
 * num_words: Number of words
 * dst:       Destination bit vector, LSB-first
 * start_bit: First bit position in dst to write
 * num_bits:  Number of bits to unpack
 */
void words_to_bits(const uint32_t *words, int num_words,
                   uint8_t *dst, uint32_t start_bit,
                   uint32_t num_bits);

/*
 * Execute a single PIO shift transfer for a TMS run.
 *
 * jtag:      Library context
 * tms_value: TMS level for this run (set via GPIO before shifting)
 * tdi:       TDI bit vector (full original vector)
 * tdo:       TDO bit vector (full original vector, may be NULL)
 * start_bit: Starting bit offset within tdi/tdo vectors
 * num_bits:  Number of bits to shift in this run
 *
 * Returns 0 on success, negative error code on failure.
 */
int pio_shift_run(rp1_jtag_t *jtag, bool tms_value,
                  const uint8_t *tdi, uint8_t *tdo,
                  uint32_t start_bit, uint32_t num_bits);

#endif /* RP1_JTAG_INTERNAL_H */
