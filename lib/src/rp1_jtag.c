/*
 * rp1_jtag.c - Core library implementation
 *
 * All JTAG shifts use SM0-only DMA:
 *   SM0: TDI/TDO/TCK (jtag_shift program, 4 instr/bit counted loop)
 *   SM1: TMS driven as GPIO output via set_pins_with_mask (no DMA)
 *
 * Variable-TMS vectors are split into constant-value runs, each
 * dispatched as a separate SM0 DMA transfer with TMS set via GPIO.
 * SM1 has no PIO program — it is used solely for its output enable
 * (OE) on TMS. TMS value is set via set_pins_with_mask.
 *
 * Two DMA paths:
 *   1. Streaming (>= STREAM_THRESHOLD): SM runs continuously with
 *      concurrent TX+RX DMA via pthreads, 256 KB per xfer_data call.
 *      Same architecture as rpi5-rp1-pio-bench (42 MB/s).
 *   2. Sequential (< STREAM_THRESHOLD): 224-bit chunks with SM restart
 *      per chunk, sequential TX then RX DMA.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define _GNU_SOURCE

#include "rp1_jtag_internal.h"
#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifdef HAVE_PIOLIB
#include "piolib.h"
#include "pio/generated/jtag_shift.pio.h"
/* jtag_tms.pio.h no longer needed — SM1 uses set_pins_with_mask */
#include "pio/generated/jtag_loopback.pio.h"
#endif

/* ---- Bit vector utilities ---- */

static inline bool bit_get(const uint8_t *vec, uint32_t bit)
{
    return (vec[bit / 8] >> (bit % 8)) & 1;
}

static inline void bit_set(uint8_t *vec, uint32_t bit, bool value)
{
    if (value)
        vec[bit / 8] |= (1u << (bit % 8));
    else
        vec[bit / 8] &= ~(1u << (bit % 8));
}

/*
 * Pack bits from a bit vector into 32-bit words, LSB-first.
 * Returns the number of words written.
 *
 * src:       Source bit vector, LSB-first
 * start_bit: First bit to extract
 * num_bits:  Number of bits to extract
 * words:     Output array of 32-bit words (caller allocates)
 */
int bits_to_words(const uint8_t *src, uint32_t start_bit,
                  uint32_t num_bits, uint32_t *words)
{
    if (num_bits == 0)
        return 0;

    uint32_t num_words = bits_to_word_count(num_bits);

    for (uint32_t w = 0; w < num_words; w++) {
        uint32_t word = 0;
        uint32_t bits_this_word = num_bits - w * BITS_PER_WORD;
        if (bits_this_word > BITS_PER_WORD)
            bits_this_word = BITS_PER_WORD;

        for (uint32_t b = 0; b < bits_this_word; b++) {
            if (bit_get(src, start_bit + w * BITS_PER_WORD + b))
                word |= (1u << b);
        }
        words[w] = word;
    }

    return (int)num_words;
}

/*
 * Unpack 32-bit words back into a bit vector sub-range, LSB-first.
 *
 * words:     Input array of 32-bit words from RX FIFO
 * num_words: Number of words
 * dst:       Destination bit vector, LSB-first
 * start_bit: First bit position in dst to write
 * num_bits:  Number of bits to unpack
 */
void words_to_bits(const uint32_t *words, uint32_t num_words,
                   uint8_t *dst, uint32_t start_bit,
                   uint32_t num_bits)
{
    for (uint32_t w = 0; w < num_words; w++) {
        uint32_t word = words[w];
        uint32_t bits_this_word = num_bits - w * BITS_PER_WORD;
        if (bits_this_word > BITS_PER_WORD)
            bits_this_word = BITS_PER_WORD;

        for (uint32_t b = 0; b < bits_this_word; b++) {
            bit_set(dst, start_bit + w * BITS_PER_WORD + b,
                    (word >> b) & 1);
        }
    }
}

/* ---- DMA thread wrappers ---- */

#ifdef HAVE_PIOLIB

/* DMA bounce buffer size for config_xfer.
 *
 * Must be large enough for the biggest single xfer_data call.
 * The kernel rounds up to PAGE_SIZE (4096) for the bounce buffer
 * allocation anyway, so 4096 wastes no memory. */
#define DMA_BUF_SIZE 4096

/* DMACTRL register overrides for TX and RX DMA.
 *
 * Register layout: bit 31 = DREQ_EN, bit 30 = HIGH_PRIORITY,
 * bits 11:7 = TREQ_SEL (priority), bits 4:0 = DREQ threshold.
 *
 * Threshold=4 is used for both TX and RX. Empirical testing with
 * rpi5-rp1-pio-bench shows that threshold=8 causes 25% data corruption
 * for DMA transfers < 128 KB, while threshold<=4 works correctly at
 * all sizes (128 bytes to 256 KB, 0% error rate).
 *
 * For RX with fewer than 4 words (shifts < 96 bits), the threshold is
 * dynamically lowered to 1 per-chunk — otherwise DREQ never fires and
 * the DMA times out. This is safe for small transfers because the AXI
 * bus contention that breaks threshold=1 at large sizes is negligible
 * for transfers of only a few words.
 *
 * config_xfer is called at the start of each bulk shift to reset the
 * kernel's DMA state (semaphore, head/tail indices, pending transfers).
 * Without this reset, accumulated state from previous shifts can cause
 * spurious timeouts.
 *
 * Must be applied AFTER pio_sm_config_xfer() which resets DMACTRL. */
#define DMACTRL_TX      0xC0000104  /* enable + high_pri + threshold=4 */
#define DMACTRL_RX_4    0xC0000104  /* enable + high_pri + threshold=4 */
#define DMACTRL_RX_1    0xC0000101  /* enable + high_pri + threshold=1 */
#define DMACTRL_RX_THRESHOLD_MIN_WORDS 4  /* use threshold=4 when >= 4 RX words */

/* DMA transfer thread args */
typedef struct {
    PIO pio;
    uint sm;
    enum pio_xfer_dir dir;
    size_t size;        /* bytes to transfer */
    void *buf;          /* data buffer */
    int ret;
    int err;            /* errno captured on failure */
} xfer_args_t;

/* DMA transfer thread: single xfer_data call for the full transfer. */
static void *xfer_thread(void *arg)
{
    xfer_args_t *a = (xfer_args_t *)arg;
    errno = 0;
    a->ret = pio_sm_xfer_data(a->pio, a->sm, a->dir, a->size, a->buf);
    a->err = errno;
    return NULL;
}

/* DMA transfer thread with loop: breaks large transfers into
 * BULK_DMA_SIZE chunks. Each pio_sm_xfer_data call transfers up
 * to BULK_DMA_SIZE bytes; for transfers larger than that (e.g. a
 * 3.8 MB bitstream), we loop. Same pattern as rpi5-rp1-pio-bench. */
static void *xfer_loop_thread(void *arg)
{
    xfer_args_t *a = (xfer_args_t *)arg;
    uint8_t *ptr = (uint8_t *)a->buf;
    size_t remaining = a->size;
    a->ret = 0;
    while (remaining > 0) {
        size_t chunk = remaining > BULK_DMA_SIZE ? BULK_DMA_SIZE : remaining;
        errno = 0;
        a->ret = pio_sm_xfer_data(a->pio, a->sm, a->dir, chunk, ptr);
        a->err = errno;
        if (a->ret < 0)
            break;
        ptr += chunk;
        remaining -= chunk;
    }
    return NULL;
}


/* ---- Loopback shift via DMA (single SM, no TMS) ---- */

/*
 * Shift num_bits through internal PIO loopback using DMA with SM0 only.
 * The loopback program copies OSR -> ISR word-by-word, no count word.
 * Two concurrent pthreads handle TX and RX.
 */
static int loopback_shift_dma(rp1_jtag_t *jtag, uint32_t num_bits,
                               const uint8_t *tdi, uint8_t *tdo)
{
    PIO pio = (PIO)jtag->pio;
    uint sm0 = (uint)jtag->sm0;
    int ret = 0;

    /* Calculate buffer sizes (no count word for loopback).
     * Enforce MIN_DMA_BYTES since PIOLib can't DMA single words. */
    uint32_t num_data_words = bits_to_word_count(num_bits);
    size_t data_bytes = num_data_words * sizeof(uint32_t);
    size_t dma_bytes = data_bytes < MIN_DMA_BYTES ? MIN_DMA_BYTES : data_bytes;

    /* Allocate aligned buffers for DMA.
     * C11 requires aligned_alloc size to be a multiple of alignment. */
    size_t alloc_bytes = (dma_bytes + 63) & ~(size_t)63;
    uint32_t *tx_buf = (uint32_t *)aligned_alloc(64, alloc_bytes);
    uint32_t *rx_buf = (uint32_t *)aligned_alloc(64, alloc_bytes);
    if (!tx_buf || !rx_buf) {
        free(tx_buf);
        free(rx_buf);
        return RP1_JTAG_ERR_IO;
    }

    /* Pack TDI data into word buffer, zero-pad remainder */
    memset(tx_buf, 0, dma_bytes);
    bits_to_words(tdi, 0, num_bits, tx_buf);
    memset(rx_buf, 0, dma_bytes);

    /* Configure DMA channels for the full transfer size. */
    ret = pio_sm_config_xfer(pio, sm0, PIO_DIR_TO_SM, dma_bytes, 1);
    if (ret < 0) goto cleanup;
    ret = pio_sm_config_xfer(pio, sm0, PIO_DIR_FROM_SM, dma_bytes, 1);
    if (ret < 0) goto cleanup;

    /* Override DMACTRL for both TX and RX.
     * Must come AFTER config_xfer which resets DMACTRL to defaults.
     * RX threshold depends on word count (threshold=4 needs >= 4 words). */
    pio_sm_set_dmactrl(pio, sm0, true,  DMACTRL_TX);
    pio_sm_set_dmactrl(pio, sm0, false,
                       num_data_words >= DMACTRL_RX_THRESHOLD_MIN_WORDS
                       ? DMACTRL_RX_4 : DMACTRL_RX_1);

    /* Enable state machine */
    pio_sm_set_enabled(pio, sm0, true);

    /* Launch RX DMA first (background thread), TX in main thread.
     * RX must be ready before TX to prevent RX FIFO overflow. */
    xfer_args_t tx_args = {pio, sm0, PIO_DIR_TO_SM,   dma_bytes, tx_buf, 0, 0};
    xfer_args_t rx_args = {pio, sm0, PIO_DIR_FROM_SM, dma_bytes, rx_buf, 0, 0};

    pthread_t t_rx;
    pthread_create(&t_rx, NULL, xfer_thread, &rx_args);
    xfer_thread(&tx_args);
    pthread_join(t_rx, NULL);

    /* Disable state machine */
    pio_sm_set_enabled(pio, sm0, false);

    /* Check for DMA errors */
    if (tx_args.ret < 0 || rx_args.ret < 0) {
        fprintf(stderr, "rp1_jtag: loopback DMA failed "
                "(tx=%d[%s], rx=%d[%s], size=%zu)\n",
                tx_args.ret, strerror(tx_args.err),
                rx_args.ret, strerror(rx_args.err),
                data_bytes);
        ret = RP1_JTAG_ERR_IO;
        goto cleanup;
    }

    /* Unpack TDO words into output bit vector */
    if (tdo) {
        words_to_bits(rx_buf, num_data_words, tdo, 0, num_bits);
    }

    ret = 0;

cleanup:
    free(tx_buf);
    free(rx_buf);
    return ret;
}

/* ---- SM0-only JTAG shift via DMA (constant TMS) ---- */

/*
 * Streaming DMA path: continuous-SM with concurrent TX+RX pthreads.
 *
 * SM runs continuously (no restart between chunks). Uses the existing
 * jtag_shift program in "streaming mode": exec mov x,!null to set
 * X = 0xFFFFFFFF, exec jmp offset+2 to skip count-word preamble.
 * The loop runs for 2^32 bits (~512 MB) before X exhaustion — SM
 * stalls on autopull when TX FIFO empties (TCK stays LOW, safe).
 *
 * No count word in TX buffer (just TDI data words), 1:1 TX:RX ratio.
 * num_bits is padded to a 32-bit boundary so autopush produces
 * only complete words (no partial-word ISR alignment needed).
 *
 * Architecture matches rpi5-rp1-pio-bench (42 MB/s proven).
 */
static int jtag_shift_streaming(rp1_jtag_t *jtag, uint32_t num_bits,
                                bool tms_value, const uint8_t *tdi,
                                uint8_t *tdo, uint32_t start_bit)
{
    PIO pio = (PIO)jtag->pio;
    uint sm0 = (uint)jtag->sm0;
    uint sm1 = (uint)jtag->sm1;
    int ret = 0;

    /* Pad num_bits to next multiple of 32 so autopush produces
     * only complete words. Extra clocks shift zeros with TMS=0
     * (harmless in Shift-DR). */
    uint32_t padded_bits = (num_bits + 31) & ~31u;
    uint32_t num_words = padded_bits / 32;
    size_t xfer_bytes = (size_t)num_words * sizeof(uint32_t);

    /* Allocate aligned buffers for DMA */
    size_t alloc_bytes = (xfer_bytes + 63) & ~(size_t)63;
    uint32_t *tx_buf = (uint32_t *)aligned_alloc(64, alloc_bytes);
    uint32_t *rx_buf = (uint32_t *)aligned_alloc(64, alloc_bytes);
    if (!tx_buf || !rx_buf) {
        free(tx_buf);
        free(rx_buf);
        return RP1_JTAG_ERR_IO;
    }

    /* Pack TDI data — no count word, just data words */
    memset(tx_buf, 0, xfer_bytes);
    uint32_t abs_bit = start_bit;
    if (abs_bit % 8 == 0) {
        /* Byte-aligned: memcpy fast path */
        uint32_t byte_off = abs_bit / 8;
        uint32_t tdi_bytes = (num_bits + 7) / 8;
        memcpy(tx_buf, tdi + byte_off, tdi_bytes);
        /* Mask unused bits in last byte (rest is zero from memset) */
        if (num_bits % 8) {
            uint8_t *tx8 = (uint8_t *)tx_buf;
            tx8[tdi_bytes - 1] &= (1u << (num_bits % 8)) - 1;
        }
    } else {
        bits_to_words(tdi, abs_bit, num_bits, tx_buf);
    }
    memset(rx_buf, 0, xfer_bytes);

    /* Disable SM0 (may be stalled from a previous shift) */
    pio_sm_set_enabled(pio, sm0, false);

    /* Configure DMA for streaming: large buffer, single buf_count */
    size_t dma_buf_size = xfer_bytes > BULK_DMA_SIZE ? BULK_DMA_SIZE : xfer_bytes;
    if (pio_sm_config_xfer(pio, sm0, PIO_DIR_TO_SM, dma_buf_size, 1) < 0 ||
        pio_sm_config_xfer(pio, sm0, PIO_DIR_FROM_SM, dma_buf_size, 1) < 0) {
        fprintf(stderr, "rp1_jtag: streaming config_xfer failed\n");
        ret = RP1_JTAG_ERR_IO;
        goto cleanup;
    }

    /* Set DMACTRL: threshold=8 for both TX and RX (matches benchmark) */
    pio_sm_set_dmactrl(pio, sm0, true,  BULK_DMACTRL);
    pio_sm_set_dmactrl(pio, sm0, false, BULK_DMACTRL);

    /* Set TMS as constant GPIO output via SM1 */
    uint32_t tms_mask = 1u << jtag->pins.tms;
    uint32_t tms_val = tms_value ? tms_mask : 0;
    pio_sm_set_pins_with_mask(pio, sm1, tms_val, tms_mask);

    /* SM setup — streaming mode:
     * 1. Clear FIFOs and restart SM (resets shift counters)
     * 2. exec mov x, !null → X = 0xFFFFFFFF (4 billion iterations)
     * 3. exec jmp offset+2 → skip pull+out x, enter loop body directly
     * 4. Enable SM — it immediately starts shifting from TX FIFO */
    pio_sm_clear_fifos(pio, sm0);
    pio_sm_restart(pio, sm0);
    pio_sm_exec(pio, sm0, pio_encode_mov_not(pio_x, pio_null));
    pio_sm_exec(pio, sm0, pio_encode_jmp(jtag->offset0 + 2));
    pio_sm_set_enabled(pio, sm0, true);

    /* Launch concurrent TX+RX DMA pthreads (exactly like benchmark).
     * RX thread created first so it's ready before TX data arrives. */
    xfer_args_t tx_args = {pio, sm0, PIO_DIR_TO_SM, xfer_bytes, tx_buf, 0, 0};
    xfer_args_t rx_args = {pio, sm0, PIO_DIR_FROM_SM, xfer_bytes, rx_buf, 0, 0};

    pthread_t t_rx, t_tx;
    pthread_create(&t_rx, NULL, xfer_loop_thread, &rx_args);
    pthread_create(&t_tx, NULL, xfer_loop_thread, &tx_args);
    pthread_join(t_tx, NULL);
    pthread_join(t_rx, NULL);

    /* Disable SM immediately after DMA completes */
    pio_sm_set_enabled(pio, sm0, false);

    /* Check for DMA errors */
    if (tx_args.ret < 0 || rx_args.ret < 0) {
        fprintf(stderr, "rp1_jtag: streaming DMA failed "
                "(tx=%d[%s], rx=%d[%s], bytes=%zu)\n",
                tx_args.ret, strerror(tx_args.err),
                rx_args.ret, strerror(rx_args.err),
                xfer_bytes);
        ret = RP1_JTAG_ERR_IO;
        goto cleanup;
    }

    /* Unpack TDO — extract first num_bits from RX buffer.
     * No partial-word ISR fix needed: autopush at 32 bits always
     * produces complete, left-aligned words (unlike the explicit
     * push in the counted path which right-aligns partial words). */
    if (tdo) {
        if (abs_bit % 8 == 0) {
            uint32_t byte_off = abs_bit / 8;
            uint32_t tdo_bytes = (num_bits + 7) / 8;
            memcpy(tdo + byte_off, rx_buf, tdo_bytes);
            /* Mask unused bits in last TDO byte */
            if (num_bits % 8) {
                tdo[byte_off + tdo_bytes - 1] &= (1u << (num_bits % 8)) - 1;
            }
        } else {
            uint32_t data_words = bits_to_word_count(num_bits);
            words_to_bits(rx_buf, data_words, tdo, abs_bit, num_bits);
        }
    }

cleanup:
    free(tx_buf);
    free(rx_buf);

    /* Restore DMA config for sequential path (next call may be small) */
    pio_sm_config_xfer(pio, sm0, PIO_DIR_TO_SM, DMA_BUF_SIZE, 1);
    pio_sm_config_xfer(pio, sm0, PIO_DIR_FROM_SM, DMA_BUF_SIZE, 1);
    pio_sm_set_dmactrl(pio, sm0, true,  DMACTRL_TX);

    return ret;
}

/*
 * Sequential DMA path: 224-bit chunks with SM restart per chunk.
 *
 * Fallback for small transfers (< STREAM_THRESHOLD bits).
 * TX DMA then RX DMA per chunk (no concurrent DMA).
 * Each chunk: count word + data → SM → autopush + explicit push.
 */
static int jtag_shift_sequential(rp1_jtag_t *jtag, uint32_t num_bits,
                                 bool tms_value, const uint8_t *tdi,
                                 uint8_t *tdo, uint32_t start_bit)
{
    PIO pio = (PIO)jtag->pio;
    uint sm0 = (uint)jtag->sm0;
    uint sm1 = (uint)jtag->sm1;
    int ret = 0;

    /* Disable SM0 (may be stalled from a previous shift) */
    pio_sm_set_enabled(pio, sm0, false);

    /* Reset DMA state */
    if (pio_sm_config_xfer(pio, sm0, PIO_DIR_TO_SM, DMA_BUF_SIZE, 1) < 0 ||
        pio_sm_config_xfer(pio, sm0, PIO_DIR_FROM_SM, DMA_BUF_SIZE, 1) < 0) {
        fprintf(stderr, "rp1_jtag: config_xfer reset failed\n");
        return RP1_JTAG_ERR_IO;
    }
    pio_sm_set_dmactrl(pio, sm0, true, DMACTRL_TX);

    /* Set TMS */
    uint32_t tms_mask = 1u << jtag->pins.tms;
    uint32_t tms_val = tms_value ? tms_mask : 0;
    pio_sm_set_pins_with_mask(pio, sm1, tms_val, tms_mask);

    /* Allocate buffers for max chunk size */
    const uint32_t max_data_words = BULK_CHUNK_BITS / BITS_PER_WORD;
    const size_t max_tx_bytes = (1 + max_data_words) * sizeof(uint32_t);
    const size_t max_rx_bytes = (max_data_words + 1) * sizeof(uint32_t);
    size_t alloc = (max_tx_bytes + 63) & ~(size_t)63;
    uint32_t *tx_buf = (uint32_t *)aligned_alloc(64, alloc);
    uint32_t *rx_buf = (uint32_t *)aligned_alloc(64, alloc);
    if (!tx_buf || !rx_buf) {
        free(tx_buf);
        free(rx_buf);
        return RP1_JTAG_ERR_IO;
    }

    /* Process in BULK_CHUNK_BITS chunks with SM restart per chunk */
    uint32_t bit_offset = 0;
    while (bit_offset < num_bits) {
        uint32_t chunk_bits = num_bits - bit_offset;
        if (chunk_bits > BULK_CHUNK_BITS)
            chunk_bits = BULK_CHUNK_BITS;

        uint32_t chunk_data_words = bits_to_word_count(chunk_bits);
        uint32_t chunk_rx_words = chunk_bits / BITS_PER_WORD + 1;
        size_t chunk_tx_bytes = (1 + chunk_data_words) * sizeof(uint32_t);
        size_t chunk_rx_bytes = chunk_rx_words * sizeof(uint32_t);

        if (chunk_tx_bytes < MIN_DMA_BYTES) chunk_tx_bytes = MIN_DMA_BYTES;
        if (chunk_rx_bytes < MIN_DMA_BYTES) chunk_rx_bytes = MIN_DMA_BYTES;

        /* Pack TX: count word + TDI data */
        memset(tx_buf, 0, chunk_tx_bytes);
        tx_buf[0] = chunk_bits - 1;

        uint32_t abs_bit = start_bit + bit_offset;

        if (abs_bit % 8 == 0) {
            uint32_t byte_off = abs_bit / 8;
            uint32_t full_words = chunk_bits / 32;
            if (full_words > 0)
                memcpy(&tx_buf[1], tdi + byte_off, full_words * 4);
            if (chunk_bits % 32) {
                uint32_t last_word = 0;
                uint32_t remaining_bits = chunk_bits % 32;
                uint32_t remaining_bytes = (remaining_bits + 7) / 8;
                memcpy(&last_word, tdi + byte_off + full_words * 4,
                       remaining_bytes);
                last_word &= (1u << remaining_bits) - 1;
                tx_buf[1 + full_words] = last_word;
            }
        } else {
            bits_to_words(tdi, abs_bit, chunk_bits, &tx_buf[1]);
        }

        memset(rx_buf, 0, chunk_rx_bytes);

        uint32_t rx_dmactrl = (chunk_rx_words >= DMACTRL_RX_THRESHOLD_MIN_WORDS)
                              ? DMACTRL_RX_4 : DMACTRL_RX_1;
        pio_sm_set_dmactrl(pio, sm0, false, rx_dmactrl);

        /* SM restart per chunk */
        pio_sm_set_enabled(pio, sm0, false);
        pio_sm_clear_fifos(pio, sm0);
        pio_sm_restart(pio, sm0);
        pio_sm_exec(pio, sm0, pio_encode_jmp(jtag->offset0));
        pio_sm_set_enabled(pio, sm0, true);

        /* Sequential DMA: TX then RX */
        xfer_args_t tx_args = {pio, sm0, PIO_DIR_TO_SM,
                               chunk_tx_bytes, tx_buf, 0, 0};
        xfer_args_t rx_args = {pio, sm0, PIO_DIR_FROM_SM,
                               chunk_rx_bytes, rx_buf, 0, 0};

        xfer_thread(&tx_args);
        xfer_thread(&rx_args);

        if (tx_args.ret < 0 || rx_args.ret < 0) {
            uint32_t tx_level = pio_sm_get_tx_fifo_level(pio, sm0);
            uint32_t rx_level = pio_sm_get_rx_fifo_level(pio, sm0);
            fprintf(stderr, "rp1_jtag: DMA chunk failed at bit %u "
                    "(tx=%d[%s], rx=%d[%s])\n",
                    abs_bit,
                    tx_args.ret, strerror(tx_args.err),
                    rx_args.ret, strerror(rx_args.err));
            fprintf(stderr, "rp1_jtag: FIFO levels after fail: "
                    "TX=%u RX=%u (expected TX=0, RX=0)\n",
                    tx_level, rx_level);
            fprintf(stderr, "rp1_jtag: Transfer sizes: "
                    "chunk_bits=%u tx_bytes=%zu(%zu words) "
                    "rx_bytes=%zu(%zu words)\n",
                    chunk_bits, chunk_tx_bytes,
                    chunk_tx_bytes / 4, chunk_rx_bytes,
                    chunk_rx_bytes / 4);
            ret = RP1_JTAG_ERR_IO;
            break;
        }

        /* Fix partial-word alignment from PIO right-shift ISR */
        {
            uint32_t remainder = chunk_bits % BITS_PER_WORD;
            if (remainder != 0) {
                rx_buf[chunk_data_words - 1] >>= (BITS_PER_WORD - remainder);
            }
        }

        /* Unpack TDO */
        if (tdo) {
            if (abs_bit % 8 == 0) {
                uint32_t byte_off = abs_bit / 8;
                uint32_t full_words = chunk_bits / 32;
                if (full_words > 0)
                    memcpy(tdo + byte_off, rx_buf, full_words * 4);
                if (chunk_bits % 32) {
                    uint32_t remaining_bits = chunk_bits % 32;
                    uint32_t remaining_bytes = (remaining_bits + 7) / 8;
                    memcpy(tdo + byte_off + full_words * 4,
                           &rx_buf[full_words], remaining_bytes);
                }
            } else {
                words_to_bits(rx_buf, chunk_data_words,
                              tdo, abs_bit, chunk_bits);
            }
        }

        bit_offset += chunk_bits;
    }

    pio_sm_set_enabled(pio, sm0, false);
    free(tx_buf);
    free(rx_buf);
    return ret;
}

/*
 * Shift num_bits through JTAG using SM0-only DMA.
 * TMS is driven as a constant GPIO output via SM1's set_pins_with_mask.
 *
 * Dispatches to streaming (continuous-SM, concurrent DMA) for large
 * transfers, or sequential (224-bit chunks) for small transfers.
 */
static int jtag_shift_bulk(rp1_jtag_t *jtag, uint32_t num_bits,
                           bool tms_value, const uint8_t *tdi,
                           uint8_t *tdo, uint32_t start_bit)
{
    if (num_bits >= STREAM_THRESHOLD) {
        return jtag_shift_streaming(jtag, num_bits, tms_value,
                                    tdi, tdo, start_bit);
    }
    return jtag_shift_sequential(jtag, num_bits, tms_value,
                                  tdi, tdo, start_bit);
}

/* ---- Public API (with PIOLib) ---- */

rp1_jtag_t *rp1_jtag_init(const rp1_jtag_pins_t *pins)
{
    if (!pins)
        return NULL;

    /* Validate pin numbers */
    if (pins->tck < 0 || pins->tck > MAX_GPIO_PIN ||
        pins->tms < 0 || pins->tms > MAX_GPIO_PIN ||
        pins->tdi < 0 || pins->tdi > MAX_GPIO_PIN ||
        pins->tdo < 0 || pins->tdo > MAX_GPIO_PIN)
        return NULL;

    /* Check for pin conflicts among required JTAG pins */
    if (pins->tck == pins->tms || pins->tck == pins->tdi ||
        pins->tck == pins->tdo || pins->tms == pins->tdi ||
        pins->tms == pins->tdo || pins->tdi == pins->tdo)
        return NULL;

    /* Initialize PIO subsystem */
    if (pio_init() < 0) {
        fprintf(stderr, "rp1_jtag: pio_init() failed\n");
        return NULL;
    }

    /* Open PIO0 instance */
    PIO pio = pio_open(0);
    if (PIO_IS_ERR(pio)) {
        fprintf(stderr, "rp1_jtag: failed to open PIO0 "
                "(is /dev/pio0 available?)\n");
        return NULL;
    }

    /* Claim two state machines */
    int sm0 = pio_claim_unused_sm(pio, true);
    if (sm0 < 0) {
        fprintf(stderr, "rp1_jtag: no free state machine for SM0\n");
        pio_close(pio);
        return NULL;
    }

    int sm1 = pio_claim_unused_sm(pio, true);
    if (sm1 < 0) {
        fprintf(stderr, "rp1_jtag: no free state machine for SM1\n");
        pio_sm_unclaim(pio, (uint)sm0);
        pio_close(pio);
        return NULL;
    }

    /* Load jtag_shift program into instruction memory.
     * Only SM0 needs a PIO program; SM1 is used only for TMS
     * pin output via set_pins_with_mask (no program needed). */
    uint offset0 = pio_add_program(pio, &jtag_shift_program);
    if (offset0 == PIO_ORIGIN_INVALID) {
        fprintf(stderr, "rp1_jtag: failed to load jtag_shift program\n");
        pio_sm_unclaim(pio, (uint)sm1);
        pio_sm_unclaim(pio, (uint)sm0);
        pio_close(pio);
        return NULL;
    }

    /* Select PIO instance for sm_config_set_* functions */
    pio_select(pio);

    /* Configure SM0: TDI/TDO/TCK (jtag_shift program) */
    pio_sm_config c0 = jtag_shift_program_get_default_config(offset0);
    sm_config_set_out_pins(&c0, pins->tdi, 1);
    sm_config_set_in_pins(&c0, pins->tdo);
    sm_config_set_sideset_pins(&c0, pins->tck);
    sm_config_set_out_shift(&c0, true, true, 32);   /* right shift, autopull at 32 */
    sm_config_set_in_shift(&c0, true, true, 32);     /* right shift, autopush at 32 */
    float div0 = (float)RP1_PIO_CLK_HZ / ((float)DEFAULT_FREQ_HZ * INSTR_PER_BIT);
    sm_config_set_clkdiv(&c0, div0);
    pio_sm_init(pio, (uint)sm0, offset0, &c0);

    /* Set pin directions for SM0 */
    pio_gpio_init(pio, pins->tck);
    pio_gpio_init(pio, pins->tdi);
    pio_gpio_init(pio, pins->tdo);
    pio_sm_set_consecutive_pindirs(pio, (uint)sm0, pins->tck, 1, true);   /* TCK output */
    pio_sm_set_consecutive_pindirs(pio, (uint)sm0, pins->tdi, 1, true);   /* TDI output */
    pio_sm_set_consecutive_pindirs(pio, (uint)sm0, pins->tdo, 1, false);  /* TDO input */

    /* Configure SM1: TMS output only (no program, no DMA).
     *
     * SM1 is used solely for its output enable (OE) on the TMS pin.
     * TMS is driven as a constant GPIO output via set_pins_with_mask,
     * which works on a disabled SM. No PIO program is needed. */
    pio_sm_config c1 = pio_get_default_sm_config();
    sm_config_set_out_pins(&c1, pins->tms, 1);
    pio_sm_init(pio, (uint)sm1, 0, &c1);

    /* Set pin direction for SM1 */
    pio_gpio_init(pio, pins->tms);
    pio_sm_set_consecutive_pindirs(pio, (uint)sm1, pins->tms, 1, true);   /* TMS output */

    /* Initial DMA config for SM0. This sets up bounce buffers in the
     * kernel. jtag_shift_bulk() calls config_xfer again before each
     * shift to reset accumulated DMA state (semaphore, indices). */
    int dma_ret;
    dma_ret = pio_sm_config_xfer(pio, (uint)sm0, PIO_DIR_TO_SM, DMA_BUF_SIZE, 1);
    if (dma_ret < 0) {
        fprintf(stderr, "rp1_jtag: config_xfer sm0 TX failed: %d\n", dma_ret);
        pio_remove_program(pio, &jtag_shift_program, offset0);
        pio_sm_unclaim(pio, (uint)sm1);
        pio_sm_unclaim(pio, (uint)sm0);
        pio_close(pio);
        return NULL;
    }
    dma_ret = pio_sm_config_xfer(pio, (uint)sm0, PIO_DIR_FROM_SM, DMA_BUF_SIZE, 1);
    if (dma_ret < 0) {
        fprintf(stderr, "rp1_jtag: config_xfer sm0 RX failed: %d\n", dma_ret);
        pio_remove_program(pio, &jtag_shift_program, offset0);
        pio_sm_unclaim(pio, (uint)sm1);
        pio_sm_unclaim(pio, (uint)sm0);
        pio_close(pio);
        return NULL;
    }

    /* Allocate and fill context */
    rp1_jtag_t *jtag = calloc(1, sizeof(rp1_jtag_t));
    if (!jtag) {
        pio_remove_program(pio, &jtag_shift_program, offset0);
        pio_sm_unclaim(pio, (uint)sm1);
        pio_sm_unclaim(pio, (uint)sm0);
        pio_close(pio);
        return NULL;
    }

    jtag->pins = *pins;
    jtag->mode = MODE_JTAG;
    jtag->freq_hz = DEFAULT_FREQ_HZ;
    jtag->pio = pio;
    jtag->sm0 = sm0;
    jtag->sm1 = sm1;
    jtag->offset0 = offset0;
    jtag->offset1 = 0;  /* no TMS program loaded */

    return jtag;
}

rp1_jtag_t *rp1_jtag_init_loopback(void)
{
    /* Initialize PIO subsystem */
    if (pio_init() < 0) {
        fprintf(stderr, "rp1_jtag: pio_init() failed\n");
        return NULL;
    }

    /* Open PIO0 instance */
    PIO pio = pio_open(0);
    if (PIO_IS_ERR(pio)) {
        fprintf(stderr, "rp1_jtag: failed to open PIO0\n");
        return NULL;
    }

    /* Claim one state machine (loopback only needs SM0) */
    int sm0 = pio_claim_unused_sm(pio, true);
    if (sm0 < 0) {
        fprintf(stderr, "rp1_jtag: no free state machine\n");
        pio_close(pio);
        return NULL;
    }

    /* Load loopback program */
    uint offset0 = pio_add_program(pio, &jtag_loopback_program);
    if (offset0 == PIO_ORIGIN_INVALID) {
        fprintf(stderr, "rp1_jtag: failed to load jtag_loopback program\n");
        pio_sm_unclaim(pio, (uint)sm0);
        pio_close(pio);
        return NULL;
    }

    /* Select PIO instance for sm_config_set_* functions */
    pio_select(pio);

    /* Configure SM0 for loopback (no pins, word-at-a-time copy) */
    pio_sm_config c0 = jtag_loopback_program_get_default_config(offset0);
    sm_config_set_out_shift(&c0, true, true, 32);   /* right shift, autopull at 32 */
    sm_config_set_in_shift(&c0, true, true, 32);     /* right shift, autopush at 32 */
    sm_config_set_clkdiv(&c0, 1.0f);                /* full speed for loopback */
    pio_sm_init(pio, (uint)sm0, offset0, &c0);

    /* Allocate and fill context */
    rp1_jtag_t *jtag = calloc(1, sizeof(rp1_jtag_t));
    if (!jtag) {
        pio_remove_program(pio, &jtag_loopback_program, offset0);
        pio_sm_unclaim(pio, (uint)sm0);
        pio_close(pio);
        return NULL;
    }

    memset(&jtag->pins, -1, sizeof(jtag->pins));
    jtag->mode = MODE_LOOPBACK;
    jtag->freq_hz = DEFAULT_FREQ_HZ;
    jtag->pio = pio;
    jtag->sm0 = sm0;
    jtag->sm1 = -1;      /* no TMS SM in loopback mode */
    jtag->offset0 = offset0;
    jtag->offset1 = 0;

    return jtag;
}

void rp1_jtag_close(rp1_jtag_t *jtag)
{
    if (!jtag)
        return;

    PIO pio = (PIO)jtag->pio;

    /* Disable state machines */
    pio_sm_set_enabled(pio, (uint)jtag->sm0, false);
    if (jtag->sm1 >= 0)
        pio_sm_set_enabled(pio, (uint)jtag->sm1, false);

    /* Remove programs from instruction memory */
    if (jtag->mode == MODE_JTAG) {
        pio_remove_program(pio, &jtag_shift_program, jtag->offset0);
    } else {
        pio_remove_program(pio, &jtag_loopback_program, jtag->offset0);
    }

    /* Release state machines */
    if (jtag->sm1 >= 0)
        pio_sm_unclaim(pio, (uint)jtag->sm1);
    pio_sm_unclaim(pio, (uint)jtag->sm0);

    /* Close PIO instance */
    pio_close(pio);

    free(jtag);
}

int rp1_jtag_set_freq(rp1_jtag_t *jtag, uint32_t freq_hz)
{
    if (!jtag || freq_hz == 0)
        return RP1_JTAG_ERR_PARAM;

    PIO pio = (PIO)jtag->pio;

    /* Clock divider: TCK freq = PIO_CLK / (INSTR_PER_BIT * divider) */
    float div = (float)RP1_PIO_CLK_HZ / ((float)freq_hz * INSTR_PER_BIT);
    if (div < 1.0f)
        div = 1.0f;

    /* Set divider on SM0 */
    pio_sm_set_clkdiv(pio, (uint)jtag->sm0, div);

    /* Set divider on SM1 if in JTAG mode (not loopback) */
    if (jtag->sm1 >= 0)
        pio_sm_set_clkdiv(pio, (uint)jtag->sm1, div);

    jtag->freq_hz = freq_hz;
    return 0;
}

uint32_t rp1_jtag_get_freq(rp1_jtag_t *jtag)
{
    if (!jtag)
        return 0;
    return jtag->freq_hz;
}

int rp1_jtag_shift(rp1_jtag_t *jtag, uint32_t num_bits,
                   const uint8_t *tms, const uint8_t *tdi,
                   uint8_t *tdo)
{
    if (!jtag || !tdi)
        return RP1_JTAG_ERR_PARAM;
    if (num_bits == 0)
        return 0;

    if (jtag->mode == MODE_JTAG) {
        if (!tms)
            return RP1_JTAG_ERR_PARAM;

        /* Split TMS into constant-value runs and dispatch each to
         * jtag_shift_bulk() (SM0-only DMA with TMS via GPIO).
         *
         * TMS is set as a constant GPIO output via SM1's
         * set_pins_with_mask before each SM0 DMA transfer.
         *
         * Fast path: for constant TMS (common case for bitstream
         * programming), the byte-level scan below finds a single
         * run and dispatches one call to jtag_shift_bulk(). */

        /* Byte-level scan for constant TMS (fast path) */
        uint8_t first_byte = (tms[0] & 1) ? 0xFF : 0x00;
        bool all_same = true;
        uint32_t full_bytes = num_bits / 8;
        for (uint32_t i = 0; i < full_bytes; i++) {
            if (tms[i] != first_byte) {
                all_same = false;
                break;
            }
        }
        if (all_same && (num_bits % 8) != 0) {
            uint8_t mask = (1u << (num_bits % 8)) - 1;
            if ((tms[full_bytes] & mask) != (first_byte & mask))
                all_same = false;
        }

        if (all_same) {
            /* Constant TMS: single call */
            return jtag_shift_bulk(jtag, num_bits, first_byte != 0,
                                   tdi, tdo, 0);
        }

        /* Variable TMS: split into constant-value runs.
         * Bit-level scanning is used for the (typically small)
         * variable-TMS case (TAP navigation, < 100 bits). */
        uint32_t pos = 0;
        while (pos < num_bits) {
            bool run_tms = bit_get(tms, pos);
            uint32_t run_start = pos;
            pos++;
            while (pos < num_bits && bit_get(tms, pos) == run_tms)
                pos++;
            uint32_t run_bits = pos - run_start;

            int rc = jtag_shift_bulk(jtag, run_bits, run_tms,
                                     tdi, tdo, run_start);
            if (rc < 0)
                return rc;
        }
        return 0;
    } else {
        return loopback_shift_dma(jtag, num_bits, tdi, tdo);
    }
}

int rp1_jtag_toggle_clk(rp1_jtag_t *jtag, uint32_t num_clocks,
                        bool tms, bool tdi)
{
    if (!jtag)
        return RP1_JTAG_ERR_PARAM;
    if (num_clocks == 0)
        return 0;

    /* Build constant TMS and TDI vectors */
    uint32_t num_bytes = (num_clocks + 7) / 8;
    uint8_t *tms_vec = malloc(num_bytes);
    uint8_t *tdi_vec = malloc(num_bytes);
    if (!tms_vec || !tdi_vec) {
        free(tms_vec);
        free(tdi_vec);
        return RP1_JTAG_ERR_IO;
    }

    memset(tms_vec, tms ? 0xFF : 0x00, num_bytes);
    memset(tdi_vec, tdi ? 0xFF : 0x00, num_bytes);

    int rc = rp1_jtag_shift(jtag, num_clocks, tms_vec, tdi_vec, NULL);

    free(tms_vec);
    free(tdi_vec);
    return rc;
}

int rp1_jtag_reset(rp1_jtag_t *jtag, int srst, int trst)
{
    if (!jtag)
        return RP1_JTAG_ERR_PARAM;

    PIO pio = (PIO)jtag->pio;

    /* Drive SRST pin if configured and requested */
    if (srst >= 0 && jtag->pins.srst >= 0) {
        uint32_t mask = 1u << jtag->pins.srst;
        uint32_t val = srst ? mask : 0;
        pio_sm_set_pins_with_mask(pio, (uint)jtag->sm0, val, mask);
    }

    /* Drive TRST pin if configured and requested */
    if (trst >= 0 && jtag->pins.trst >= 0) {
        uint32_t mask = 1u << jtag->pins.trst;
        uint32_t val = trst ? mask : 0;
        pio_sm_set_pins_with_mask(pio, (uint)jtag->sm0, val, mask);
    }

    return 0;
}

#else /* !HAVE_PIOLIB */

/* ---- Stub implementations when PIOLib is not available ---- */

rp1_jtag_t *rp1_jtag_init(const rp1_jtag_pins_t *pins)
{
    (void)pins;
    fprintf(stderr, "rp1_jtag: compiled without PIOLib support\n");
    return NULL;
}

rp1_jtag_t *rp1_jtag_init_loopback(void)
{
    fprintf(stderr, "rp1_jtag: compiled without PIOLib support\n");
    return NULL;
}

void rp1_jtag_close(rp1_jtag_t *jtag) { (void)jtag; }

int rp1_jtag_set_freq(rp1_jtag_t *jtag, uint32_t freq_hz)
{
    (void)jtag; (void)freq_hz;
    return RP1_JTAG_ERR_NODEV;
}

uint32_t rp1_jtag_get_freq(rp1_jtag_t *jtag)
{
    (void)jtag;
    return 0;
}

int rp1_jtag_shift(rp1_jtag_t *jtag, uint32_t num_bits,
                   const uint8_t *tms, const uint8_t *tdi,
                   uint8_t *tdo)
{
    (void)jtag; (void)num_bits; (void)tms; (void)tdi; (void)tdo;
    return RP1_JTAG_ERR_NODEV;
}

int rp1_jtag_toggle_clk(rp1_jtag_t *jtag, uint32_t num_clocks,
                        bool tms, bool tdi)
{
    (void)jtag; (void)num_clocks; (void)tms; (void)tdi;
    return RP1_JTAG_ERR_NODEV;
}

int rp1_jtag_reset(rp1_jtag_t *jtag, int srst, int trst)
{
    (void)jtag; (void)srst; (void)trst;
    return RP1_JTAG_ERR_NODEV;
}

#endif /* HAVE_PIOLIB */
