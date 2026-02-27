/*
 * rp1_jtag_internal.h - Internal types for librp1jtag (Phase 2 DMA)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RP1_JTAG_INTERNAL_H
#define RP1_JTAG_INTERNAL_H

#include "rp1_jtag.h"
#include <stdbool.h>
#include <stdint.h>

/* PIO clock frequency (RP1 PIO runs at 200 MHz) */
#define RP1_PIO_CLK_HZ          200000000

/* Default JTAG clock frequency (10 MHz).
 * With jtag_shift (4 instr/bit): divider = 200M / (10M * 4) = 5,
 * giving adequate TDI setup time for Artix-7.
 * openFPGALoader's --freq flag overrides this default. */
#define DEFAULT_FREQ_HZ         10000000

/* Maximum valid BCM GPIO number on RPi 5 header */
#define MAX_GPIO_PIN            27

/* Number of bits per FIFO word */
#define BITS_PER_WORD           32

/* PIO instructions per JTAG bit (jtag_shift program: 4 instruction loop) */
#define INSTR_PER_BIT           4

/* DMA FIFO threshold register value.
 * 0xC0000108: DREQ enable (bit 31) | threshold=8 (bits 4:0) | other config.
 * Ensures FIFO threshold matches the 8-beat burst of heavy DMA channels 0/1. */
#define DMA_FIFO_THRESHOLD      0xC0000108

/* Operating mode */
typedef enum {
    MODE_JTAG,      /* Normal JTAG mode: two SMs, GPIO pins */
    MODE_LOOPBACK,  /* Internal PIO loopback: single SM, no GPIO */
} rp1_jtag_mode_t;

/* Main library context */
struct rp1_jtag {
    rp1_jtag_pins_t pins;
    rp1_jtag_mode_t mode;
    uint32_t freq_hz;
    void *pio;              /* PIO instance (PIO type from PIOLib) */
    int sm0;                /* SM0: TDI/TDO/TCK */
    int sm1;                /* SM1: TMS (-1 in loopback mode) */
    unsigned int offset0;   /* SM0 program offset in instruction memory */
    unsigned int offset1;   /* SM1 program offset in instruction memory */
};

/* Convert a bit count to the number of 32-bit words needed */
static inline uint32_t bits_to_word_count(uint32_t num_bits) {
    return (num_bits + BITS_PER_WORD - 1) / BITS_PER_WORD;
}

#endif /* RP1_JTAG_INTERNAL_H */
