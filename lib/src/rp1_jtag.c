/*
 * rp1_jtag.c - Core library implementation (Phase 2 DMA)
 *
 * Two-SM architecture with bulk DMA via three concurrent pthreads:
 *   SM0: TDI/TDO/TCK (jtag_shift program, 4 instr/bit counted loop)
 *   SM1: TMS (jtag_tms program, synchronized to SM0's TCK via GPIO wait)
 *   Thread 1: SM0 TX DMA (count word + TDI data)
 *   Thread 2: SM0 RX DMA (TDO data)
 *   Thread 3: SM1 TX DMA (TMS data)
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
#include "pio/generated/jtag_tms.pio.h"
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

/* ---- DMA thread wrapper ---- */

#ifdef HAVE_PIOLIB

typedef struct {
    PIO pio;
    uint sm;
    enum pio_xfer_dir dir;
    size_t size;
    void *buf;
    int ret;
    int err;    /* errno captured after pio_sm_xfer_data */
} xfer_args_t;

static void *xfer_thread(void *arg)
{
    xfer_args_t *a = (xfer_args_t *)arg;
    errno = 0;
    a->ret = pio_sm_xfer_data(a->pio, a->sm, a->dir, a->size, a->buf);
    a->err = errno;
    return NULL;
}


/* ---- Core JTAG shift via DMA (two SMs) ---- */

/*
 * Shift num_bits through JTAG using DMA with SM0 (TDI/TDO/TCK) and
 * SM1 (TMS). DMA transfers run concurrently:
 *   - SM1 TX: TMS data (background thread, started first)
 *   - SM0 RX: TDO data (background thread)
 *   - SM0 TX: count word + TDI data (calling thread)
 */
static int jtag_shift_dma(rp1_jtag_t *jtag, uint32_t num_bits,
                           const uint8_t *tms, const uint8_t *tdi,
                           uint8_t *tdo)
{
    PIO pio = (PIO)jtag->pio;
    uint sm0 = (uint)jtag->sm0;
    uint sm1 = (uint)jtag->sm1;
    int ret = 0;

    /* Reset both SMs to program start with clean shift counters.
     *
     * After a previous shift, SM1 may be stalled mid-program
     * (at `wait 1 gpio 4` instead of program start). Without
     * resetting, SM1 misses the first TMS output on the next
     * shift, causing TMS to be delayed by one bit.
     *
     * pio_sm_restart() clears shift counters (ISR/OSR count = 0),
     * which triggers autopull on SM1's first `out pins, 1`,
     * correctly loading fresh TMS data from the FIFO.
     *
     * clear_fifos is not needed: FIFOs are empty after a completed
     * DMA transfer (all data consumed by SM / read by DMA).
     * pio_select is not needed: only used by sm_config_set_*
     * functions, not by pio_sm_exec or pio_sm_restart. */
    pio_sm_set_enabled(pio, sm0, false);
    pio_sm_set_enabled(pio, sm1, false);
    pio_sm_restart(pio, sm0);
    pio_sm_restart(pio, sm1);
    pio_sm_exec(pio, sm0, pio_encode_jmp(jtag->offset0));
    pio_sm_exec(pio, sm1, pio_encode_jmp(jtag->offset1));

    /* Calculate buffer sizes.
     *
     * SM0 TX: 1 count word + ceil(N/32) TDI data words.
     * SM0 RX: The PIO program produces floor(N/32) autopush words
     *         plus 1 explicit push word = floor(N/32) + 1 total.
     *         This differs from ceil(N/32) when N is a multiple of 32.
     * SM1 TX: ceil(N/32) TMS data words (no count word).
     *
     * Enforce MIN_DMA_BYTES since PIOLib can't DMA single words. */
    uint32_t num_data_words = bits_to_word_count(num_bits);
    uint32_t sm0_rx_words = num_bits / BITS_PER_WORD + 1;  /* autopush + final push */
    size_t sm0_tx_bytes = (1 + num_data_words) * sizeof(uint32_t);  /* count + TDI */
    size_t sm0_rx_bytes = sm0_rx_words * sizeof(uint32_t);          /* TDO */
    size_t sm1_tx_bytes = num_data_words * sizeof(uint32_t);        /* TMS */

    /* DMA transfer sizes (padded to minimum) */
    size_t sm0_tx_dma = sm0_tx_bytes < MIN_DMA_BYTES ? MIN_DMA_BYTES : sm0_tx_bytes;
    size_t sm0_rx_dma = sm0_rx_bytes < MIN_DMA_BYTES ? MIN_DMA_BYTES : sm0_rx_bytes;
    size_t sm1_tx_dma = sm1_tx_bytes < MIN_DMA_BYTES ? MIN_DMA_BYTES : sm1_tx_bytes;

    /* Allocate aligned buffers for DMA.
     * C11 requires aligned_alloc size to be a multiple of alignment. */
    size_t sm0_tx_alloc = (sm0_tx_dma + 63) & ~(size_t)63;
    size_t sm0_rx_alloc = (sm0_rx_dma + 63) & ~(size_t)63;
    size_t sm1_tx_alloc = (sm1_tx_dma + 63) & ~(size_t)63;
    uint32_t *sm0_tx = (uint32_t *)aligned_alloc(64, sm0_tx_alloc);
    uint32_t *sm0_rx = (uint32_t *)aligned_alloc(64, sm0_rx_alloc);
    uint32_t *sm1_tx = (uint32_t *)aligned_alloc(64, sm1_tx_alloc);
    if (!sm0_tx || !sm0_rx || !sm1_tx) {
        free(sm0_tx);
        free(sm0_rx);
        free(sm1_tx);
        return RP1_JTAG_ERR_IO;
    }

    /* Pack data into word buffers, zero-pad any DMA padding */
    memset(sm0_tx, 0, sm0_tx_dma);
    memset(sm1_tx, 0, sm1_tx_dma);
    sm0_tx[0] = num_bits - 1;                        /* count word */
    bits_to_words(tdi, 0, num_bits, &sm0_tx[1]);     /* TDI data */
    bits_to_words(tms, 0, num_bits, sm1_tx);         /* TMS data */
    memset(sm0_rx, 0, sm0_rx_dma);                   /* clear RX buffer */

    /* NOTE: DMA channels are configured once during init via
     * pio_sm_config_xfer(). Per-shift transfers use xfer_data only. */

    /* Enable both state machines */
    pio_sm_set_enabled(pio, sm0, true);
    pio_sm_set_enabled(pio, sm1, true);

    /* Launch DMA transfers: 2 threads + calling thread.
     *
     * ORDERING: SM1 TX (TMS) must start before SM0 TX (TDI/TCK).
     *
     * At this point both SMs are enabled but stalled:
     *   SM0: stalled on `pull side 0` (TX FIFO empty, TCK held LOW)
     *   SM1: passed `wait 0 gpio 4` (TCK is LOW), stalled on `out pins, 1`
     *        (TX FIFO empty, autopull blocks)
     *
     * SM1 TX and SM0 RX run as background threads. SM0 TX runs on
     * the calling thread. Thread creation gives SM1 TX a head start:
     * by the time SM0 TX calls xfer_data (~50-100 µs later, after
     * creating both threads), SM1's DMA is already being set up in
     * the kernel. This replaces the previous usleep(500) and also
     * eliminates one pthread_create/join pair. */
    xfer_args_t tx1 = {pio, sm1, PIO_DIR_TO_SM,   sm1_tx_dma, sm1_tx, 0, 0};
    xfer_args_t rx0 = {pio, sm0, PIO_DIR_FROM_SM, sm0_rx_dma, sm0_rx, 0, 0};

    pthread_t t_tx1, t_rx0;
    pthread_create(&t_tx1, NULL, xfer_thread, &tx1);  /* SM1 TX first (TMS) */
    pthread_create(&t_rx0, NULL, xfer_thread, &rx0);  /* SM0 RX (TDO) */

    /* SM0 TX on calling thread — SM1 TX thread has head start from
     * the time spent creating both threads above. */
    int tx0_ret, tx0_err;
    errno = 0;
    tx0_ret = pio_sm_xfer_data(pio, sm0, PIO_DIR_TO_SM, sm0_tx_dma, sm0_tx);
    tx0_err = errno;

    pthread_join(t_tx1, NULL);
    pthread_join(t_rx0, NULL);

    /* SMs are left enabled but stalled (SM0 at `pull side 0`,
     * SM1 at `out pins, 1` waiting for autopull). Next call's
     * reset handles disable, and rp1_jtag_close() cleans up. */

    /* Check for DMA errors */
    if (tx0_ret < 0 || rx0.ret < 0 || tx1.ret < 0) {
        fprintf(stderr, "rp1_jtag: DMA transfer failed "
                "(tx0=%d[%s], rx0=%d[%s], tx1=%d[%s])\n",
                tx0_ret, strerror(tx0_err),
                rx0.ret, strerror(rx0.err),
                tx1.ret, strerror(tx1.err));
        ret = RP1_JTAG_ERR_IO;
        goto cleanup;
    }

    /* Fix partial-word alignment from PIO right-shift ISR.
     *
     * With in_shift_right=true, each `in pins, 1` shifts data into the
     * ISR from the MSB end. After N shifts (N < 32), valid data sits in
     * bits [(32-N)..31], not bits [0..(N-1)]. Full 32-bit words from
     * autopush are correctly aligned, but the last partial word from
     * the explicit `push` instruction needs right-shifting. */
    {
        uint32_t remainder = num_bits % BITS_PER_WORD;
        if (remainder != 0) {
            sm0_rx[num_data_words - 1] >>= (BITS_PER_WORD - remainder);
        }
    }

    /* Unpack TDO words into output bit vector */
    if (tdo) {
        words_to_bits(sm0_rx, num_data_words, tdo, 0, num_bits);
    }

    ret = 0;

cleanup:
    free(sm0_tx);
    free(sm0_rx);
    free(sm1_tx);
    return ret;
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

    /* Configure DMA channels */
    ret = pio_sm_config_xfer(pio, sm0, PIO_DIR_TO_SM, dma_bytes, 1);
    if (ret < 0) goto cleanup;
    ret = pio_sm_config_xfer(pio, sm0, PIO_DIR_FROM_SM, dma_bytes, 1);
    if (ret < 0) goto cleanup;

    /* NOTE: Do NOT call pio_sm_set_dmactrl() here. The kernel driver
     * (PR #7190) already sets correct FIFO thresholds in config_xfer.
     * Manually overriding with 0xC0000108 breaks RX DMA on newer kernels. */

    /* Enable state machine */
    pio_sm_set_enabled(pio, sm0, true);

    /* Launch 2 concurrent DMA threads */
    xfer_args_t tx_args = {pio, sm0, PIO_DIR_TO_SM,   dma_bytes, tx_buf, 0, 0};
    xfer_args_t rx_args = {pio, sm0, PIO_DIR_FROM_SM, dma_bytes, rx_buf, 0, 0};

    pthread_t t_tx, t_rx;
    pthread_create(&t_tx, NULL, xfer_thread, &tx_args);
    pthread_create(&t_rx, NULL, xfer_thread, &rx_args);

    pthread_join(t_tx, NULL);
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

    /* Load PIO programs into instruction memory */
    uint offset0 = pio_add_program(pio, &jtag_shift_program);
    if (offset0 == PIO_ORIGIN_INVALID) {
        fprintf(stderr, "rp1_jtag: failed to load jtag_shift program\n");
        pio_sm_unclaim(pio, (uint)sm1);
        pio_sm_unclaim(pio, (uint)sm0);
        pio_close(pio);
        return NULL;
    }

    uint offset1 = pio_add_program(pio, &jtag_tms_program);
    if (offset1 == PIO_ORIGIN_INVALID) {
        fprintf(stderr, "rp1_jtag: failed to load jtag_tms program\n");
        pio_remove_program(pio, &jtag_shift_program, offset0);
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

    /* Configure SM1: TMS (jtag_tms program)
     *
     * LIMITATION: The jtag_tms program has GPIO4 hardcoded in its wait
     * instructions (wait 0 gpio 4 / wait 1 gpio 4) to synchronize with
     * SM0's TCK. If TCK is not on GPIO4, the wait instructions would need
     * to be patched at runtime. For NeTV2 (TCK=GPIO4), this works as-is. */
    pio_sm_config c1 = jtag_tms_program_get_default_config(offset1);
    sm_config_set_out_pins(&c1, pins->tms, 1);
    sm_config_set_out_shift(&c1, true, true, 32);   /* right shift, autopull at 32 */
    sm_config_set_fifo_join(&c1, PIO_FIFO_JOIN_TX); /* TX-only FIFO (8 words) */
    sm_config_set_clkdiv(&c1, div0);                /* same clock as SM0 */
    pio_sm_init(pio, (uint)sm1, offset1, &c1);

    /* Set pin direction for SM1 */
    pio_gpio_init(pio, pins->tms);
    pio_sm_set_consecutive_pindirs(pio, (uint)sm1, pins->tms, 1, true);   /* TMS output */

    /* Configure DMA bounce buffers for all channels (once at init).
     *
     * buf_size=4096 matches the kernel's DMA_BOUNCE_BUFFER_SIZE.
     * Actual transfer sizes in xfer_data can be smaller.
     *
     * NOTE: SM1 TX DMA has a size limit (~88 bytes / 22 words) beyond
     * which it times out. Chunk sizes in rp1_jtag_shift are set to stay
     * within this limit via MAX_CHUNK_BITS. */
    #define DMA_BUF_SIZE 4096
    int dma_ret;
    dma_ret = pio_sm_config_xfer(pio, (uint)sm0, PIO_DIR_TO_SM, DMA_BUF_SIZE, 1);
    if (dma_ret < 0) {
        fprintf(stderr, "rp1_jtag: config_xfer sm0 TX failed: %d\n", dma_ret);
        pio_remove_program(pio, &jtag_tms_program, offset1);
        pio_remove_program(pio, &jtag_shift_program, offset0);
        pio_sm_unclaim(pio, (uint)sm1);
        pio_sm_unclaim(pio, (uint)sm0);
        pio_close(pio);
        return NULL;
    }
    dma_ret = pio_sm_config_xfer(pio, (uint)sm0, PIO_DIR_FROM_SM, DMA_BUF_SIZE, 1);
    if (dma_ret < 0) {
        fprintf(stderr, "rp1_jtag: config_xfer sm0 RX failed: %d\n", dma_ret);
        pio_remove_program(pio, &jtag_tms_program, offset1);
        pio_remove_program(pio, &jtag_shift_program, offset0);
        pio_sm_unclaim(pio, (uint)sm1);
        pio_sm_unclaim(pio, (uint)sm0);
        pio_close(pio);
        return NULL;
    }
    /* SM1 TX DMA: Use 256-byte buffer (enough for MAX_CHUNK_BITS=640
     * which needs 20 words = 80 bytes). Larger buffers cause DMA timeout
     * in the kernel driver's bounce buffer setup. */
    dma_ret = pio_sm_config_xfer(pio, (uint)sm1, PIO_DIR_TO_SM, 256, 1);
    if (dma_ret < 0) {
        fprintf(stderr, "rp1_jtag: config_xfer sm1 TX failed: %d\n", dma_ret);
        pio_remove_program(pio, &jtag_tms_program, offset1);
        pio_remove_program(pio, &jtag_shift_program, offset0);
        pio_sm_unclaim(pio, (uint)sm1);
        pio_sm_unclaim(pio, (uint)sm0);
        pio_close(pio);
        return NULL;
    }

    /* Allocate and fill context */
    rp1_jtag_t *jtag = calloc(1, sizeof(rp1_jtag_t));
    if (!jtag) {
        pio_remove_program(pio, &jtag_tms_program, offset1);
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
    jtag->offset1 = offset1;

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
        pio_remove_program(pio, &jtag_tms_program, jtag->offset1);
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

        /* Chunk large transfers to stay within PIOLib's DMA timeout.
         * Each chunk is a complete PIO transfer with its own count word.
         * Chunks are at 32-bit word boundaries so byte pointer arithmetic
         * stays aligned. The JTAG TAP state is maintained between chunks
         * (TMS is passed through per-bit, not modified by chunking). */
        uint32_t offset = 0;
        while (offset < num_bits) {
            uint32_t chunk = num_bits - offset;
            if (chunk > MAX_CHUNK_BITS)
                chunk = MAX_CHUNK_BITS;

            uint32_t byte_off = offset / 8;
            int rc = jtag_shift_dma(jtag, chunk,
                                     tms + byte_off,
                                     tdi + byte_off,
                                     tdo ? tdo + byte_off : NULL);
            if (rc < 0)
                return rc;
            offset += chunk;
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
