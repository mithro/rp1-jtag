/*
 * rp1_jtag_internal.h - Internal types for librp1jtag
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

/* Minimum DMA transfer size in bytes.
 * PIOLib/kernel requires at least 8 bytes (2 words) for DMA transfers;
 * single-word (4 byte) transfers time out on RP1. */
#define MIN_DMA_BYTES           8

/* PIO instructions per JTAG bit (jtag_shift program: 4 instruction loop) */
#define INSTR_PER_BIT           4

/* Maximum bits per DMA transfer chunk.
 *
 * Each chunk is a complete DMA round-trip: SM restart, TX DMA
 * (count word + TDI data), RX DMA (TDO data + explicit push),
 * with pthread for RX. Larger chunks reduce ioctl and thread
 * overhead per bitstream.
 *
 * Size limit: TX bytes = (1 + ceil(N/32)) * 4 must fit in
 * DMA_BUF_SIZE (4096). At 30720 bits: TX = 3844 bytes.
 *
 * Verified working up to 16384 bits by test_many_shifts.
 * 30720 gives ~995 chunks per 30M-bit bitstream (vs ~29,889
 * at 1024 bits), reducing ioctl overhead by ~30x.
 *
 * Must be a multiple of BITS_PER_WORD (32). */
#define BULK_CHUNK_BITS         30720

/* Operating mode */
typedef enum {
    MODE_JTAG,      /* Normal JTAG mode: SM0 DMA + SM1 TMS GPIO */
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

/* Pack bits from a bit vector into 32-bit words, LSB-first.
 * Returns the number of words written. */
int bits_to_words(const uint8_t *src, uint32_t start_bit,
                  uint32_t num_bits, uint32_t *words);

/* Unpack 32-bit words back into a bit vector sub-range, LSB-first. */
void words_to_bits(const uint32_t *words, uint32_t num_words,
                   uint8_t *dst, uint32_t start_bit,
                   uint32_t num_bits);

#endif /* RP1_JTAG_INTERNAL_H */
