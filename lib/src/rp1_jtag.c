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

/* Maximum words per PIO transfer (TX FIFO data words, excluding count) */
#define MAX_TRANSFER_WORDS 1024

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

    /*
     * Split large runs into MAX_CHUNK_BITS-sized PIO transfers.
     * Each chunk is a separate PIO transfer (count word + data words).
     * TMS stays constant since it's set once per run via GPIO.
     */
    uint32_t bit_offset = start_bit;
    uint32_t remaining = num_bits;

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
        .clk_div      = (float)RP1_PIO_CLK_HZ / (jtag->freq_hz * 5),
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

    /* Clock divider: PIO runs 5 instructions per bit (jtag_shift),
     * so TCK frequency = PIO_CLK / (5 * divider) */
    float div = (float)RP1_PIO_CLK_HZ / (freq_hz * 5);
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
