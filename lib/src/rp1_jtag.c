/*
 * rp1_jtag.c - Core library implementation
 *
 * TMS run-length splitting + word-by-word FIFO interleaving.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rp1_jtag_internal.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* Maximum number of TMS runs we support per shift call.
 * In practice, openFPGALoader generates 2-3 runs per call. */
#define MAX_TMS_RUNS 256

/* Maximum words per PIO transfer (TX FIFO data words, excluding count).
 * Each chunk uses two stack arrays of this size (tdi_words + tdo_words),
 * so 8192 words = 32 KB per array = 64 KB total stack usage per chunk.
 * RPi 5 default stack limit is 8 MB, so this is well within bounds. */
#define MAX_TRANSFER_WORDS 8192

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

/* ---- TMS run-length scanning ---- */

int tms_scan_runs(const uint8_t *tms, uint32_t num_bits,
                  tms_run_t *runs, int max_runs)
{
    if (num_bits == 0)
        return 0;

    int count = 0;
    uint32_t run_start = 0;
    bool run_value = bit_get(tms, 0);

    for (uint32_t i = 1; i < num_bits; i++) {
        bool v = bit_get(tms, i);
        if (v != run_value) {
            /* End of current run */
            if (count >= max_runs)
                return -1;
            runs[count].start_bit = run_start;
            runs[count].num_bits = i - run_start;
            runs[count].tms_value = run_value;
            count++;
            run_start = i;
            run_value = v;
        }
    }

    /* Final run */
    if (count >= max_runs)
        return -1;
    runs[count].start_bit = run_start;
    runs[count].num_bits = num_bits - run_start;
    runs[count].tms_value = run_value;
    count++;

    return count;
}

/* ---- Bit-to-word packing ---- */

int bits_to_words(const uint8_t *src, uint32_t start_bit,
                  uint32_t num_bits, uint32_t *words)
{
    if (num_bits == 0)
        return 0;

    int num_words = (num_bits + BITS_PER_WORD - 1) / BITS_PER_WORD;

    for (int w = 0; w < num_words; w++) {
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

    return num_words;
}

void words_to_bits(const uint32_t *words, int num_words,
                   uint8_t *dst, uint32_t start_bit,
                   uint32_t num_bits)
{
    for (int w = 0; w < num_words; w++) {
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

/* ---- PIO shift for a single chunk ---- */

/* Maximum bits per PIO transfer chunk */
#define MAX_CHUNK_BITS (MAX_TRANSFER_WORDS * BITS_PER_WORD)

static int pio_shift_chunk(rp1_jtag_t *jtag,
                           const uint8_t *tdi, uint8_t *tdo,
                           uint32_t start_bit, uint32_t num_bits)
{
    pio_backend_t *be = jtag->backend;
    int rc;

    /* Pack TDI bits into words */
    int num_data_words = (num_bits + BITS_PER_WORD - 1) / BITS_PER_WORD;
    uint32_t tdi_words[MAX_TRANSFER_WORDS];
    bits_to_words(tdi, start_bit, num_bits, tdi_words);

    /* Write count word to TX FIFO */
    rc = be->ops->sm_put(be, num_bits - 1);
    if (rc < 0)
        return RP1_JTAG_ERR_IO;

    /* Word-by-word FIFO interleaving:
     *   - Write TDI words to TX FIFO
     *   - Read TDO words from RX FIFO as they become available
     * The PIO SM processes 32 bits per word, so we alternate
     * between feeding TX and draining RX to avoid FIFO overflow. */
    uint32_t tdo_words[MAX_TRANSFER_WORDS];
    int tx_idx = 0;
    int rx_idx = 0;

    while (rx_idx < num_data_words) {
        /* Feed TX FIFO */
        while (tx_idx < num_data_words &&
               be->ops->tx_fifo_has_space(be)) {
            rc = be->ops->sm_put(be, tdi_words[tx_idx]);
            if (rc < 0)
                return RP1_JTAG_ERR_IO;
            tx_idx++;
        }

        /* Drain RX FIFO */
        while (rx_idx < num_data_words &&
               be->ops->rx_fifo_has_data(be)) {
            rc = be->ops->sm_get(be, &tdo_words[rx_idx]);
            if (rc < 0)
                return RP1_JTAG_ERR_IO;
            rx_idx++;
        }

        /* If neither FIFO is ready, try blocking read to make progress */
        if (rx_idx < num_data_words && tx_idx >= num_data_words) {
            rc = be->ops->sm_get(be, &tdo_words[rx_idx]);
            if (rc < 0)
                return RP1_JTAG_ERR_IO;
            rx_idx++;
        }
    }

    /* Drain spurious words from RX FIFO.
     * RP1 PIO (v1) fires autopush before `push` instructions. For transfers
     * that are exact multiples of 32 bits, autopush sends the real data word,
     * then the explicit `push` sends a zeroed ISR as a spurious extra word.
     * We must discard these to prevent them accumulating across transfers. */
    {
        uint32_t discard;
        while (be->ops->rx_fifo_has_data(be))
            be->ops->sm_get(be, &discard);
    }

    /* Fix partial-word alignment from PIO right-shift ISR.
     *
     * With in_shift_right=true, each `in pins, 1` shifts data into the
     * ISR from the MSB end. After N shifts (N < 32), valid data sits in
     * bits [(32-N)..31], not bits [0..(N-1)]. Full 32-bit words from
     * autopush are correctly aligned, but the last partial word from
     * the explicit `push` instruction needs right-shifting. */
    {
        uint32_t partial_bits = num_bits % BITS_PER_WORD;
        if (partial_bits != 0)
            tdo_words[num_data_words - 1] >>= (BITS_PER_WORD - partial_bits);
    }

    /* Unpack TDO words into output bit vector */
    if (tdo) {
        words_to_bits(tdo_words, num_data_words, tdo, start_bit, num_bits);
    }

    return 0;
}

/* ---- Program switching ---- */

/*
 * Minimum bits to justify switching to the fast program.
 * Must amortize program-switch overhead (~50us on RP1 PIOLib).
 * 256 bits = 8 words = 1 PIO transfer's worth of data.
 */
#define FAST_PROGRAM_THRESHOLD  256

/*
 * Switch to a different PIO program and update the clock divider.
 * Disables the SM before switching, does NOT re-enable (caller decides).
 */
static int switch_program(rp1_jtag_t *jtag, pio_program_id_t prog,
                          int new_instr_per_bit)
{
    pio_backend_t *be = jtag->backend;
    int rc;

    if (jtag->current_program == prog)
        return 0;

    be->ops->sm_set_enabled(be, false);

    rc = be->ops->load_program(be, prog);
    if (rc < 0)
        return RP1_JTAG_ERR_IO;

    jtag->current_program = prog;
    jtag->instr_per_bit = new_instr_per_bit;

    /* Update divider for new program's instructions-per-bit */
    float div = (float)RP1_PIO_CLK_HZ /
                (jtag->freq_hz * jtag->instr_per_bit);
    if (div < 1.0f)
        div = 1.0f;
    be->ops->set_clk_div(be, div);

    return 0;
}

/* ---- Fast PIO shift (no count word, SM managed by caller) ---- */

/*
 * DMA buffer size for config_xfer: must be large enough for the largest
 * single xfer_data_bidi call. MAX_TRANSFER_WORDS * 4 = 32768 bytes.
 */
#define DMA_BUF_SIZE  (MAX_TRANSFER_WORDS * 4)

/*
 * DMA ping-pong chunk size: 8 words = 32 bytes (matches RP1 FIFO depth).
 * Used only in the single-direction fallback path where larger transfers
 * deadlock because the blocking TX DMA fills the TX FIFO but nobody
 * drains the RX FIFO, stalling the SM.
 */
#define DMA_CHUNK_WORDS  8
#define DMA_CHUNK_BYTES  (DMA_CHUNK_WORDS * 4)

/*
 * Ensure DMA channels are configured (once, lazily).
 */
static int ensure_dma_configured(rp1_jtag_t *jtag)
{
    pio_backend_t *be = jtag->backend;
    int rc;

    if (jtag->dma_configured)
        return 0;

    if (!be->ops->config_xfer)
        return -1;

    rc = be->ops->config_xfer(be, PIO_BE_DIR_TX, DMA_BUF_SIZE, 1);
    if (rc < 0)
        return rc;

    rc = be->ops->config_xfer(be, PIO_BE_DIR_RX, DMA_BUF_SIZE, 1);
    if (rc < 0)
        return rc;

    jtag->dma_configured = true;
    return 0;
}

/*
 * Shift one chunk using jtag_shift_fast (2 instr/bit, no count word).
 * Only handles exact multiples of 32 bits.
 *
 * Caller must enable the SM before the first call and disable it after
 * the last call. Between calls, the SM stalls safely with TCK low
 * (the `out pins, 1 side 0` instruction stalls on autopull).
 *
 * Three-tier DMA strategy:
 *  1. xfer_data_bidi: threaded concurrent TX+RX (entire chunk, ~2 ioctls)
 *  2. xfer_data ping-pong: TX 32B → RX 32B → repeat (~N/4 ioctls)
 *  3. word-by-word FIFO interleaving (no DMA)
 */
static int pio_shift_fast_chunk(rp1_jtag_t *jtag,
                                const uint8_t *tdi, uint8_t *tdo,
                                uint32_t start_bit, uint32_t num_bits)
{
    pio_backend_t *be = jtag->backend;
    int rc;

    /* num_bits must be a multiple of 32 */
    int num_words = num_bits / BITS_PER_WORD;
    uint32_t tdi_words[MAX_TRANSFER_WORDS];
    bits_to_words(tdi, start_bit, num_bits, tdi_words);

    uint32_t tdo_words[MAX_TRANSFER_WORDS];

    /* Try threaded bidi DMA first (entire chunk in one call) */
    if (be->ops->xfer_data_bidi && ensure_dma_configured(jtag) == 0) {
        int data_bytes = num_words * 4;
        rc = be->ops->xfer_data_bidi(be, data_bytes, tdi_words,
                                          data_bytes, tdo_words);
        if (rc < 0)
            return RP1_JTAG_ERR_IO;
    } else if (be->ops->xfer_data && ensure_dma_configured(jtag) == 0) {
        /*
         * Fallback: DMA ping-pong (TX 32 bytes, RX 32 bytes, repeat).
         * Each DMA call transfers 8 words (FIFO depth). This reduces
         * the ioctl count from 2N (word-by-word) to N/4 (DMA chunks).
         */
        int word_idx = 0;
        while (word_idx < num_words) {
            int chunk_words = num_words - word_idx;
            if (chunk_words > DMA_CHUNK_WORDS)
                chunk_words = DMA_CHUNK_WORDS;
            int chunk_bytes = chunk_words * 4;

            rc = be->ops->xfer_data(be, PIO_BE_DIR_TX,
                                    chunk_bytes, &tdi_words[word_idx]);
            if (rc < 0)
                return RP1_JTAG_ERR_IO;

            rc = be->ops->xfer_data(be, PIO_BE_DIR_RX,
                                    chunk_bytes, &tdo_words[word_idx]);
            if (rc < 0)
                return RP1_JTAG_ERR_IO;

            word_idx += chunk_words;
        }
    } else {
        /* Fallback: word-by-word FIFO interleaving */
        int tx_idx = 0;
        int rx_idx = 0;

        while (rx_idx < num_words) {
            while (tx_idx < num_words &&
                   be->ops->tx_fifo_has_space(be)) {
                rc = be->ops->sm_put(be, tdi_words[tx_idx]);
                if (rc < 0)
                    return RP1_JTAG_ERR_IO;
                tx_idx++;
            }

            while (rx_idx < num_words &&
                   be->ops->rx_fifo_has_data(be)) {
                rc = be->ops->sm_get(be, &tdo_words[rx_idx]);
                if (rc < 0)
                    return RP1_JTAG_ERR_IO;
                rx_idx++;
            }

            if (rx_idx < num_words && tx_idx >= num_words) {
                rc = be->ops->sm_get(be, &tdo_words[rx_idx]);
                if (rc < 0)
                    return RP1_JTAG_ERR_IO;
                rx_idx++;
            }
        }
    }

    /* Unpack TDO (no partial-word fixup needed — all words are full 32-bit) */
    if (tdo)
        words_to_bits(tdo_words, num_words, tdo, start_bit, num_bits);

    return 0;
}

/* ---- PIO shift for a single TMS run ---- */

int pio_shift_run(rp1_jtag_t *jtag, bool tms_value,
                  const uint8_t *tdi, uint8_t *tdo,
                  uint32_t start_bit, uint32_t num_bits)
{
    int rc;

    if (num_bits == 0)
        return 0;

    /* Set TMS via GPIO (only in JTAG mode, not loopback) */
    if (jtag->mode == MODE_JTAG && jtag->pins.tms >= 0) {
        rc = jtag->backend->ops->gpio_set(jtag->backend,
                                           jtag->pins.tms, tms_value);
        if (rc < 0)
            return RP1_JTAG_ERR_IO;
    }

    uint32_t bit_offset = start_bit;
    uint32_t remaining = num_bits;

    /*
     * Fast path: use jtag_shift_fast for bulk data (≥256 bits, JTAG mode).
     * The fast program is 2 instructions/bit (vs 5), but only handles
     * exact multiples of 32 bits. Remainder goes through counted program.
     *
     * Not used in loopback mode (fast program uses real pins, loopback
     * program uses internal y-register copy).
     */
    if (remaining >= FAST_PROGRAM_THRESHOLD && jtag->mode == MODE_JTAG) {
        uint32_t fast_bits = (remaining / BITS_PER_WORD) * BITS_PER_WORD;
        uint32_t remainder_bits = remaining - fast_bits;

        /* Switch to fast program (disables SM) */
        rc = switch_program(jtag, PIO_PROG_JTAG_SHIFT_FAST,
                            INSTR_PER_BIT_FAST);
        if (rc < 0)
            return rc;

        /* Enable SM once — fast program stalls safely on autopull
         * when TX FIFO empties, holding TCK low between chunks. */
        jtag->backend->ops->sm_set_enabled(jtag->backend, true);

        /* Process in MAX_CHUNK_BITS-sized pieces */
        while (fast_bits > 0) {
            uint32_t chunk_bits = fast_bits;
            if (chunk_bits > MAX_CHUNK_BITS)
                chunk_bits = MAX_CHUNK_BITS;

            rc = pio_shift_fast_chunk(jtag, tdi, tdo,
                                      bit_offset, chunk_bits);
            if (rc < 0)
                break;

            bit_offset += chunk_bits;
            fast_bits -= chunk_bits;
        }

        /* Stop sequence (once): disable SM, force TCK low, drain FIFO */
        {
            pio_backend_t *be = jtag->backend;
            be->ops->sm_set_enabled(be, false);
            if (jtag->mode == MODE_JTAG && jtag->pins.tck >= 0)
                be->ops->gpio_set(be, jtag->pins.tck, false);
            uint32_t discard;
            while (be->ops->rx_fifo_has_data(be))
                be->ops->sm_get(be, &discard);
        }

        if (rc < 0)
            return rc;

        remaining = remainder_bits;

        if (remaining > 0) {
            /* Switch back to counted program for remainder */
            rc = switch_program(jtag, PIO_PROG_JTAG_SHIFT,
                                INSTR_PER_BIT_COUNTED);
            if (rc < 0)
                return rc;
            jtag->backend->ops->sm_set_enabled(jtag->backend, true);
        }
    }

    /*
     * Counted path: use jtag_shift for remaining bits (or all bits
     * if below threshold). Handles any bit count including partial words.
     */
    if (remaining > 0) {
        /* Ensure counted program is loaded and SM is running */
        if (jtag->current_program != PIO_PROG_JTAG_SHIFT &&
            jtag->current_program != PIO_PROG_JTAG_LOOPBACK) {
            pio_program_id_t counted_prog =
                (jtag->mode == MODE_LOOPBACK) ? PIO_PROG_JTAG_LOOPBACK
                                               : PIO_PROG_JTAG_SHIFT;
            rc = switch_program(jtag, counted_prog, INSTR_PER_BIT_COUNTED);
            if (rc < 0)
                return rc;
            jtag->backend->ops->sm_set_enabled(jtag->backend, true);
        }

        while (remaining > 0) {
            uint32_t chunk_bits = remaining;
            if (chunk_bits > MAX_CHUNK_BITS)
                chunk_bits = MAX_CHUNK_BITS;

            rc = pio_shift_chunk(jtag, tdi, tdo, bit_offset, chunk_bits);
            if (rc < 0)
                return rc;

            bit_offset += chunk_bits;
            remaining -= chunk_bits;
        }
    }

    return 0;
}

/* ---- Public API ---- */

static rp1_jtag_t *init_common(pio_backend_t *backend,
                                const rp1_jtag_pins_t *pins,
                                rp1_jtag_mode_t mode,
                                pio_program_id_t prog)
{
    rp1_jtag_t *jtag = calloc(1, sizeof(rp1_jtag_t));
    if (!jtag) {
        pio_backend_destroy(backend);
        return NULL;
    }

    jtag->backend = backend;
    jtag->mode = mode;
    jtag->freq_hz = DEFAULT_FREQ_HZ;
    jtag->instr_per_bit = INSTR_PER_BIT_COUNTED;
    jtag->current_program = prog;

    if (pins)
        jtag->pins = *pins;
    else
        memset(&jtag->pins, -1, sizeof(jtag->pins));

    /* Configure PIO SM pins */
    pio_sm_pins_t sm_pins = {
        .sideset_base = pins ? pins->tck : -1,
        .out_base     = pins ? pins->tdi : -1,
        .in_base      = pins ? pins->tdo : -1,
        .tms_base     = pins ? pins->tms : -1,
        .clk_div      = (float)RP1_PIO_CLK_HZ /
                         (jtag->freq_hz * jtag->instr_per_bit),
    };

    int rc = backend->ops->init(backend, prog, &sm_pins);
    if (rc < 0) {
        free(jtag);
        pio_backend_destroy(backend);
        return NULL;
    }

    /* Enable the state machine */
    backend->ops->sm_set_enabled(backend, true);

    return jtag;
}

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

    pio_backend_t *be = pio_backend_rp1_create();
    if (!be)
        return NULL;

    return init_common(be, pins, MODE_JTAG, PIO_PROG_JTAG_SHIFT);
}

rp1_jtag_t *rp1_jtag_init_loopback(void)
{
    pio_backend_t *be = pio_backend_rp1_create();
    if (!be)
        return NULL;

    return init_common(be, NULL, MODE_LOOPBACK, PIO_PROG_JTAG_LOOPBACK);
}

void rp1_jtag_close(rp1_jtag_t *jtag)
{
    if (!jtag)
        return;
    pio_backend_destroy(jtag->backend);
    free(jtag);
}

int rp1_jtag_set_freq(rp1_jtag_t *jtag, uint32_t freq_hz)
{
    if (!jtag || freq_hz == 0)
        return RP1_JTAG_ERR_PARAM;

    /*
     * Clock divider: TCK freq = PIO_CLK / (instr_per_bit * divider).
     * jtag_shift uses 5 instr/bit, jtag_shift_fast uses 2 instr/bit.
     */
    float div = (float)RP1_PIO_CLK_HZ / (freq_hz * jtag->instr_per_bit);
    if (div < 1.0f)
        div = 1.0f;

    int rc = jtag->backend->ops->set_clk_div(jtag->backend, div);
    if (rc < 0)
        return RP1_JTAG_ERR_IO;

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
    if (!jtag || !tms || !tdi)
        return RP1_JTAG_ERR_PARAM;
    if (num_bits == 0)
        return 0;

    /* Scan TMS vector into contiguous runs of constant value */
    tms_run_t runs[MAX_TMS_RUNS];
    int num_runs = tms_scan_runs(tms, num_bits, runs, MAX_TMS_RUNS);
    if (num_runs < 0)
        return RP1_JTAG_ERR_PARAM;

    /* Execute each run as a separate PIO transfer */
    for (int i = 0; i < num_runs; i++) {
        int rc = pio_shift_run(jtag, runs[i].tms_value,
                               tdi, tdo,
                               runs[i].start_bit,
                               runs[i].num_bits);
        if (rc < 0)
            return rc;
    }

    return 0;
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

    pio_backend_t *be = jtag->backend;
    int rc;

    if (srst >= 0 && jtag->pins.srst >= 0) {
        rc = be->ops->gpio_set(be, jtag->pins.srst, srst ? true : false);
        if (rc < 0)
            return RP1_JTAG_ERR_IO;
    }

    if (trst >= 0 && jtag->pins.trst >= 0) {
        rc = be->ops->gpio_set(be, jtag->pins.trst, trst ? true : false);
        if (rc < 0)
            return RP1_JTAG_ERR_IO;
    }

    return 0;
}

/*
 * Internal: create a context with a specific backend (for unit tests).
 * The backend is not initialized -- caller must call init() on it.
 */
rp1_jtag_t *rp1_jtag_init_with_backend(pio_backend_t *backend,
                                        const rp1_jtag_pins_t *pins)
{
    return init_common(backend, pins, MODE_JTAG, PIO_PROG_JTAG_SHIFT);
}
