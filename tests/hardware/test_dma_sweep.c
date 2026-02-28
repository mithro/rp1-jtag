/*
 * test_dma_sweep.c - Find exact DMA size threshold for jtag_shift program
 *
 * Tests PIO DMA with both loopback and jtag_shift programs at ascending
 * transfer sizes. Stops at first failure (DMA failure corrupts PIO state).
 *
 * Usage: test_dma_sweep [--loopback-only] [--jtag-only] [--clkdiv N]
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <stdbool.h>
#include <time.h>

#ifdef HAVE_PIOLIB
#include "piolib.h"
#include "pio/generated/jtag_loopback.pio.h"
#include "pio/generated/jtag_shift.pio.h"

/* jtag_shift_nocount: same timing as jtag_shift (4 instructions/bit,
 * sideset TCK) but wraps forever using autopull/autopush only.
 * No explicit pull or push. Tests whether explicit pull/push
 * instructions interact differently with DMA than autopull/autopush.
 *
 * Equivalent PIO assembly:
 *   .program jtag_shift_nocount
 *   .side_set 1
 *   .wrap_target
 *       out pins, 1     side 0    ; Drive TDI, TCK LOW
 *       nop             side 0    ; Extra cycle (TMS setup time)
 *       in pins, 1      side 1    ; Sample TDO, TCK HIGH
 *       nop             side 0    ; TCK LOW
 *   .wrap
 */
#define jtag_shift_nocount_wrap_target 0
#define jtag_shift_nocount_wrap 3

static const uint16_t jtag_shift_nocount_program_instructions[] = {
            //     .wrap_target
    0x6001, //  0: out    pins, 1         side 0
    0xa042, //  1: nop                    side 0
    0x5001, //  2: in     pins, 1         side 1
    0xa042, //  3: nop                    side 0
            //     .wrap
};

static const struct pio_program jtag_shift_nocount_program = {
    .instructions = jtag_shift_nocount_program_instructions,
    .length = 4,
    .origin = -1,
    .pio_version = 0,
#if PICO_PIO_VERSION > 0
    .used_gpio_ranges = 0x0
#endif
};

/* DMA thread wrapper (same as library uses) */
typedef struct {
    PIO pio;
    uint sm;
    enum pio_xfer_dir dir;
    size_t size;
    void *buf;
    int ret;
    int err;
} xfer_args_t;

static void *xfer_thread(void *arg)
{
    xfer_args_t *a = (xfer_args_t *)arg;
    errno = 0;
    a->ret = pio_sm_xfer_data(a->pio, a->sm, a->dir, a->size, a->buf);
    a->err = errno;
    return NULL;
}

/* Test loopback DMA at a given word count and clock divider.
 * Returns 0 on success, -1 on failure. */
static int test_loopback(PIO pio, uint sm, uint offset, float clkdiv,
                         uint32_t num_words)
{
    size_t bytes = num_words * sizeof(uint32_t);
    if (bytes < 8) bytes = 8;  /* MIN_DMA_BYTES */

    /* Allocate and fill TX buffer with test pattern */
    uint32_t *tx = (uint32_t *)aligned_alloc(64, (bytes + 63) & ~(size_t)63);
    uint32_t *rx = (uint32_t *)aligned_alloc(64, (bytes + 63) & ~(size_t)63);
    if (!tx || !rx) { free(tx); free(rx); return -1; }
    memset(tx, 0, bytes);
    memset(rx, 0, bytes);
    for (uint32_t i = 0; i < num_words; i++)
        tx[i] = 0xA5000000 | i;

    /* Configure SM */
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_restart(pio, sm);

    pio_select(pio);
    pio_sm_config c = jtag_loopback_program_get_default_config(offset);
    sm_config_set_out_shift(&c, true, true, 32);
    sm_config_set_in_shift(&c, true, true, 32);
    sm_config_set_clkdiv(&c, clkdiv);
    pio_sm_init(pio, sm, offset, &c);

    /* Configure DMA */
    int ret;
    ret = pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, bytes, 1);
    if (ret < 0) { free(tx); free(rx); return -1; }
    ret = pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, bytes, 1);
    if (ret < 0) { free(tx); free(rx); return -1; }

    pio_sm_set_enabled(pio, sm, true);

    /* Launch concurrent TX/RX DMA */
    xfer_args_t tx_args = {pio, sm, PIO_DIR_TO_SM,   bytes, tx, 0, 0};
    xfer_args_t rx_args = {pio, sm, PIO_DIR_FROM_SM, bytes, rx, 0, 0};

    pthread_t t_tx, t_rx;
    pthread_create(&t_tx, NULL, xfer_thread, &tx_args);
    pthread_create(&t_rx, NULL, xfer_thread, &rx_args);
    pthread_join(t_tx, NULL);
    pthread_join(t_rx, NULL);

    pio_sm_set_enabled(pio, sm, false);

    int result = 0;
    if (tx_args.ret < 0 || rx_args.ret < 0) {
        printf("    DMA FAIL: tx=%d[%s] rx=%d[%s]\n",
               tx_args.ret, strerror(tx_args.err),
               rx_args.ret, strerror(rx_args.err));
        result = -1;
    } else {
        /* Verify data */
        int mismatches = 0;
        for (uint32_t i = 0; i < num_words && mismatches < 3; i++) {
            if (rx[i] != tx[i]) {
                printf("    DATA MISMATCH word[%u]: tx=0x%08x rx=0x%08x\n",
                       i, tx[i], rx[i]);
                mismatches++;
            }
        }
        if (mismatches > 0)
            result = -1;
    }

    free(tx);
    free(rx);
    return result;
}

/* Modes for testing jtag_shift DMA */
typedef enum {
    JS_MODE_COUNT_IN_DMA,    /* Count word included in TX DMA (library default) */
    JS_MODE_COUNT_VIA_PUT,   /* Count word via put_blocking, DMA for data only */
} js_mode_t;

static const char *js_mode_name(js_mode_t mode)
{
    switch (mode) {
    case JS_MODE_COUNT_IN_DMA: return "count-in-DMA";
    case JS_MODE_COUNT_VIA_PUT: return "count-via-put";
    default: return "unknown";
    }
}

/* Test jtag_shift DMA at a given bit count and clock divider.
 * Sends count word + zero TDI data. TDO is whatever GPIO reads.
 * Only checks DMA success, not data correctness.
 * Returns 0 on success, -1 on failure. */
static int test_jtag_shift(PIO pio, uint sm, uint offset, float clkdiv,
                           uint32_t num_bits, int tdi_pin, int tdo_pin,
                           int tck_pin, js_mode_t mode)
{
    /* Calculate sizes */
    uint32_t num_data_words = (num_bits + 31) / 32;
    uint32_t sm0_rx_words = num_bits / 32 + 1;  /* autopush + explicit push */

    /* TX bytes depends on mode */
    size_t tx_bytes;
    if (mode == JS_MODE_COUNT_VIA_PUT) {
        tx_bytes = num_data_words * sizeof(uint32_t);  /* data only, no count */
    } else {
        tx_bytes = (1 + num_data_words) * sizeof(uint32_t);  /* count + data */
    }
    size_t rx_bytes = sm0_rx_words * sizeof(uint32_t);
    if (tx_bytes < 8) tx_bytes = 8;
    if (rx_bytes < 8) rx_bytes = 8;

    /* Allocate buffers */
    uint32_t *tx = (uint32_t *)aligned_alloc(64, (tx_bytes + 63) & ~(size_t)63);
    uint32_t *rx = (uint32_t *)aligned_alloc(64, (rx_bytes + 63) & ~(size_t)63);
    if (!tx || !rx) { free(tx); free(rx); return -1; }
    memset(tx, 0, tx_bytes);
    memset(rx, 0, rx_bytes);

    /* Pack TX data */
    if (mode == JS_MODE_COUNT_VIA_PUT) {
        /* Data only (count fed via put_blocking) */
    } else {
        tx[0] = num_bits - 1;  /* count word */
    }

    /* Configure SM */
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_restart(pio, sm);

    pio_select(pio);
    pio_sm_config c = jtag_shift_program_get_default_config(offset);
    sm_config_set_out_pins(&c, (uint)tdi_pin, 1);
    sm_config_set_in_pins(&c, (uint)tdo_pin);
    sm_config_set_sideset_pins(&c, (uint)tck_pin);
    sm_config_set_out_shift(&c, true, true, 32);
    sm_config_set_in_shift(&c, true, true, 32);
    sm_config_set_clkdiv(&c, clkdiv);
    pio_sm_init(pio, sm, offset, &c);

    /* Set pin directions */
    pio_gpio_init(pio, (uint)tck_pin);
    pio_gpio_init(pio, (uint)tdi_pin);
    pio_gpio_init(pio, (uint)tdo_pin);
    pio_sm_set_consecutive_pindirs(pio, sm, (uint)tck_pin, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, (uint)tdi_pin, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, (uint)tdo_pin, 1, false);

    /* Configure DMA with large buffer */
    size_t config_size = 256 * 1024;

    int ret;
    ret = pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, config_size, 1);
    if (ret < 0) {
        printf("    config_xfer TX failed: %d (errno=%d: %s)\n",
               ret, errno, strerror(errno));
        free(tx); free(rx);
        return -1;
    }
    ret = pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, config_size, 1);
    if (ret < 0) {
        printf("    config_xfer RX failed: %d (errno=%d: %s)\n",
               ret, errno, strerror(errno));
        free(tx); free(rx);
        return -1;
    }

    /* Enable SM */
    pio_sm_set_enabled(pio, sm, true);

    /* In COUNT_VIA_PUT mode, feed count word before starting DMA.
     * SM is at `pull side 0`, waiting for TX FIFO. put_blocking feeds
     * the count word. SM pulls it, loads X, and enters the loop.
     * SM then stalls at first `out pins, 1` autopull (FIFO empty).
     * DMA then provides the data words. */
    if (mode == JS_MODE_COUNT_VIA_PUT) {
        pio_sm_put_blocking(pio, sm, num_bits - 1);
    }

    /* Launch concurrent TX/RX DMA */
    xfer_args_t tx_args = {pio, sm, PIO_DIR_TO_SM,   tx_bytes, tx, 0, 0};
    xfer_args_t rx_args = {pio, sm, PIO_DIR_FROM_SM, rx_bytes, rx, 0, 0};

    pthread_t t_tx, t_rx;
    pthread_create(&t_tx, NULL, xfer_thread, &tx_args);
    pthread_create(&t_rx, NULL, xfer_thread, &rx_args);
    pthread_join(t_tx, NULL);
    pthread_join(t_rx, NULL);

    pio_sm_set_enabled(pio, sm, false);

    int result = 0;
    if (tx_args.ret < 0 || rx_args.ret < 0) {
        printf("    DMA FAIL: tx=%d[%s] rx=%d[%s]\n",
               tx_args.ret, strerror(tx_args.err),
               rx_args.ret, strerror(rx_args.err));
        result = -1;
    }

    free(tx);
    free(rx);
    return result;
}

/* Test nocount program: same timing as jtag_shift but wraps forever
 * with autopull/autopush only (no explicit pull/push).
 * TX DMA = ceil(num_bits/32) words of TDI data (no count word).
 * RX DMA = floor(num_bits/32) words of TDO data (no explicit push).
 * num_bits MUST be a multiple of 32 for clean autopush behavior.
 * Returns 0 on success, -1 on failure. */
static int test_jtag_nocount(PIO pio, uint sm, uint offset, float clkdiv,
                              uint32_t num_bits, int tdi_pin, int tdo_pin,
                              int tck_pin)
{
    /* num_bits must be multiple of 32 */
    if (num_bits % 32 != 0) {
        printf("    ERROR: num_bits=%u not multiple of 32\n", num_bits);
        return -1;
    }

    uint32_t num_words = num_bits / 32;
    size_t tx_bytes = num_words * sizeof(uint32_t);
    size_t rx_bytes = num_words * sizeof(uint32_t);
    if (tx_bytes < 8) tx_bytes = 8;
    if (rx_bytes < 8) rx_bytes = 8;

    /* Allocate buffers */
    uint32_t *tx = (uint32_t *)aligned_alloc(64, (tx_bytes + 63) & ~(size_t)63);
    uint32_t *rx = (uint32_t *)aligned_alloc(64, (rx_bytes + 63) & ~(size_t)63);
    if (!tx || !rx) { free(tx); free(rx); return -1; }
    memset(tx, 0, tx_bytes);
    memset(rx, 0, rx_bytes);

    /* Configure SM */
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_restart(pio, sm);

    pio_select(pio);
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + jtag_shift_nocount_wrap_target,
                       offset + jtag_shift_nocount_wrap);
    sm_config_set_sideset(&c, 1, false, false);
    sm_config_set_out_pins(&c, (uint)tdi_pin, 1);
    sm_config_set_in_pins(&c, (uint)tdo_pin);
    sm_config_set_sideset_pins(&c, (uint)tck_pin);
    sm_config_set_out_shift(&c, true, true, 32);   /* LSB-first, autopull at 32 */
    sm_config_set_in_shift(&c, true, true, 32);    /* LSB-first, autopush at 32 */
    sm_config_set_clkdiv(&c, clkdiv);
    pio_sm_init(pio, sm, offset, &c);

    /* Set pin directions */
    pio_gpio_init(pio, (uint)tck_pin);
    pio_gpio_init(pio, (uint)tdi_pin);
    pio_gpio_init(pio, (uint)tdo_pin);
    pio_sm_set_consecutive_pindirs(pio, sm, (uint)tck_pin, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, (uint)tdi_pin, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, (uint)tdo_pin, 1, false);

    /* Configure DMA with large buffer */
    size_t config_size = 256 * 1024;

    int ret;
    ret = pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, config_size, 1);
    if (ret < 0) {
        printf("    config_xfer TX failed: %d (errno=%d: %s)\n",
               ret, errno, strerror(errno));
        free(tx); free(rx);
        return -1;
    }
    ret = pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, config_size, 1);
    if (ret < 0) {
        printf("    config_xfer RX failed: %d (errno=%d: %s)\n",
               ret, errno, strerror(errno));
        free(tx); free(rx);
        return -1;
    }

    /* Enable SM */
    pio_sm_set_enabled(pio, sm, true);

    /* Launch concurrent TX/RX DMA */
    xfer_args_t tx_args = {pio, sm, PIO_DIR_TO_SM,   tx_bytes, tx, 0, 0};
    xfer_args_t rx_args = {pio, sm, PIO_DIR_FROM_SM, rx_bytes, rx, 0, 0};

    pthread_t t_tx, t_rx;
    pthread_create(&t_tx, NULL, xfer_thread, &tx_args);
    pthread_create(&t_rx, NULL, xfer_thread, &rx_args);
    pthread_join(t_tx, NULL);
    pthread_join(t_rx, NULL);

    pio_sm_set_enabled(pio, sm, false);

    int result = 0;
    if (tx_args.ret < 0 || rx_args.ret < 0) {
        printf("    DMA FAIL: tx=%d[%s] rx=%d[%s]\n",
               tx_args.ret, strerror(tx_args.err),
               rx_args.ret, strerror(rx_args.err));
        result = -1;
    }

    free(tx);
    free(rx);
    return result;
}

/* Test sequential jtag_shift DMA: do N consecutive 1024-bit transfers
 * on the same SM WITHOUT restarting between them. The SM wraps from
 * push back to pull, so it should be ready for the next chunk.
 * Returns 0 on success, -1 on failure. */
static int test_jtag_sequential(PIO pio, uint sm, uint offset, float clkdiv,
                                 int num_chunks, int tdi_pin, int tdo_pin,
                                 int tck_pin, bool reconfig_per_chunk,
                                 bool restart_only)
{
    const uint32_t bits_per_chunk = 1024;
    const uint32_t data_words = bits_per_chunk / 32;  /* 32 */
    const size_t tx_bytes = (1 + data_words) * sizeof(uint32_t);  /* 132 */
    const size_t rx_bytes = (data_words + 1) * sizeof(uint32_t);  /* 132 */
    /* +1 for explicit push (even though 1024%32==0, push still fires) */

    /* Allocate buffers */
    uint32_t *tx = (uint32_t *)aligned_alloc(64, (tx_bytes + 63) & ~(size_t)63);
    uint32_t *rx = (uint32_t *)aligned_alloc(64, (rx_bytes + 63) & ~(size_t)63);
    if (!tx || !rx) { free(tx); free(rx); return -1; }

    /* Configure SM once */
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_restart(pio, sm);

    pio_select(pio);
    pio_sm_config c = jtag_shift_program_get_default_config(offset);
    sm_config_set_out_pins(&c, (uint)tdi_pin, 1);
    sm_config_set_in_pins(&c, (uint)tdo_pin);
    sm_config_set_sideset_pins(&c, (uint)tck_pin);
    sm_config_set_out_shift(&c, true, true, 32);
    sm_config_set_in_shift(&c, true, true, 32);
    sm_config_set_clkdiv(&c, clkdiv);
    pio_sm_init(pio, sm, offset, &c);

    /* Set pin directions */
    pio_gpio_init(pio, (uint)tck_pin);
    pio_gpio_init(pio, (uint)tdi_pin);
    pio_gpio_init(pio, (uint)tdo_pin);
    pio_sm_set_consecutive_pindirs(pio, sm, (uint)tck_pin, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, (uint)tdi_pin, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, (uint)tdo_pin, 1, false);

    /* Configure DMA once with buffer large enough for any chunk.
     * Use 4096 to match the library's DMA_BUF_SIZE default. */
    size_t config_size = restart_only ? 4096 : 256;
    int ret;
    ret = pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, config_size, 1);
    if (ret < 0) {
        printf("    config_xfer TX failed: %d (errno=%d: %s)\n",
               ret, errno, strerror(errno));
        free(tx); free(rx); return -1;
    }
    ret = pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, config_size, 1);
    if (ret < 0) {
        printf("    config_xfer RX failed: %d (errno=%d: %s)\n",
               ret, errno, strerror(errno));
        free(tx); free(rx); return -1;
    }

    /* Enable SM once — it starts at pull, waiting for count word */
    pio_sm_set_enabled(pio, sm, true);

    int result = 0;
    for (int chunk = 0; chunk < num_chunks; chunk++) {
        /* Optionally reset SM and/or reconfigure DMA before each chunk.
         * restart_only: SM restart only (matches existing library pattern).
         * reconfig_per_chunk: SM restart + config_xfer (full reinit). */
        if (reconfig_per_chunk || restart_only) {
            pio_sm_set_enabled(pio, sm, false);
            pio_sm_restart(pio, sm);
            pio_sm_exec(pio, sm, pio_encode_jmp(offset));

            if (reconfig_per_chunk) {
                ret = pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM,
                                          config_size, 1);
                if (ret < 0) {
                    printf("    Chunk %d config_xfer TX: %d (%s)\n",
                           chunk + 1, ret, strerror(errno));
                    result = -1; break;
                }
                ret = pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM,
                                          config_size, 1);
                if (ret < 0) {
                    printf("    Chunk %d config_xfer RX: %d (%s)\n",
                           chunk + 1, ret, strerror(errno));
                    result = -1; break;
                }
            }

            pio_sm_set_enabled(pio, sm, true);
        }

        /* Build TX: count word + data words (all zeros) */
        memset(tx, 0, tx_bytes);
        tx[0] = bits_per_chunk - 1;
        memset(rx, 0, rx_bytes);

        /* Launch concurrent TX/RX DMA */
        xfer_args_t tx_args = {pio, sm, PIO_DIR_TO_SM,   tx_bytes, tx, 0, 0};
        xfer_args_t rx_args = {pio, sm, PIO_DIR_FROM_SM, rx_bytes, rx, 0, 0};

        pthread_t t_tx, t_rx;
        pthread_create(&t_tx, NULL, xfer_thread, &tx_args);
        pthread_create(&t_rx, NULL, xfer_thread, &rx_args);
        pthread_join(t_tx, NULL);
        pthread_join(t_rx, NULL);

        if (tx_args.ret < 0 || rx_args.ret < 0) {
            printf("    Chunk %d/%d FAIL: tx=%d[%s] rx=%d[%s]\n",
                   chunk + 1, num_chunks,
                   tx_args.ret, strerror(tx_args.err),
                   rx_args.ret, strerror(rx_args.err));
            result = -1;
            break;
        }
    }

    pio_sm_set_enabled(pio, sm, false);
    free(tx);
    free(rx);
    return result;
}

int main(int argc, char *argv[])
{
    bool do_loopback = true;
    bool do_jtag = true;
    bool do_nocount = true;
    bool do_sequential = false;
    bool seq_reconfig = false;
    bool seq_restart_only = false;
    int seq_chunks = 10;
    float clkdiv = 5.0f;  /* default: 200 MHz / 5 / 4 = 10 MHz TCK */

    /* Parse args */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--loopback-only") == 0) {
            do_jtag = false; do_nocount = false;
        } else if (strcmp(argv[i], "--jtag-only") == 0) {
            do_loopback = false; do_nocount = false;
        } else if (strcmp(argv[i], "--nocount-only") == 0) {
            do_loopback = false; do_jtag = false;
        } else if (strcmp(argv[i], "--sequential") == 0) {
            do_loopback = false; do_jtag = false; do_nocount = false;
            do_sequential = true;
            if (i + 1 < argc && argv[i+1][0] != '-')
                seq_chunks = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--reconfig") == 0) {
            seq_reconfig = true;
        } else if (strcmp(argv[i], "--restart-only") == 0) {
            seq_restart_only = true;
        } else if (strcmp(argv[i], "--clkdiv") == 0 && i + 1 < argc) {
            clkdiv = strtof(argv[++i], NULL);
        } else {
            fprintf(stderr, "Usage: %s [--loopback-only] [--jtag-only] "
                    "[--nocount-only] [--sequential [N]] [--clkdiv N]\n",
                    argv[0]);
            return 2;
        }
    }

    printf("DMA Sweep Test\n");
    printf("==============\n");
    printf("Clock divider: %.1f\n\n", clkdiv);

    /* Init PIO */
    if (pio_init() < 0) { printf("FAIL: pio_init\n"); return 1; }
    PIO pio = pio_open(0);
    if (PIO_IS_ERR(pio)) { printf("FAIL: pio_open\n"); return 1; }
    int sm = pio_claim_unused_sm(pio, true);
    if (sm < 0) { printf("FAIL: claim SM\n"); pio_close(pio); return 1; }

    /* === Loopback sweep === */
    if (do_loopback) {
        uint lb_offset = pio_add_program(pio, &jtag_loopback_program);
        if (lb_offset == PIO_ORIGIN_INVALID) {
            printf("FAIL: load loopback program\n");
            goto cleanup;
        }

        printf("=== Loopback DMA (clkdiv=%.1f) ===\n", clkdiv);
        /* Test ascending word counts with fine granularity around threshold.
         * Known: 56 words (224 bytes) passes, 60 words (240 bytes) fails. */
        uint32_t lb_sizes[] = {2, 8, 16, 32, 48, 56, 57, 58, 59, 60, 61,
                                62, 63, 64, 65, 96, 128, 256, 512, 1024};
        int lb_count = sizeof(lb_sizes) / sizeof(lb_sizes[0]);

        for (int i = 0; i < lb_count; i++) {
            uint32_t words = lb_sizes[i];
            size_t bytes = words * 4;
            printf("  %4u words (%5zu bytes): ", words, bytes);
            fflush(stdout);

            int rc = test_loopback(pio, (uint)sm, lb_offset, clkdiv, words);
            if (rc == 0) {
                printf("PASS\n");
            } else {
                printf("FAIL\n");
                printf("  Stopping loopback tests (PIO may be corrupted)\n");
                break;
            }
        }

        pio_sm_set_enabled(pio, (uint)sm, false);
        pio_remove_program(pio, &jtag_loopback_program, lb_offset);
        printf("\n");
    }

    /* === nocount sweep (autopull/autopush only, same timing as jtag_shift) === */
    if (do_nocount) {
        uint nc_offset = pio_add_program(pio, &jtag_shift_nocount_program);
        if (nc_offset == PIO_ORIGIN_INVALID) {
            printf("FAIL: load nocount program\n");
            goto cleanup;
        }

        /* NeTV2 pins */
        int nc_tdi = 27, nc_tdo = 22, nc_tck = 4;

        printf("=== nocount DMA (autopull/autopush, 4 instr/bit, clkdiv=%.1f) ===\n",
               clkdiv);
        printf("  Pins: TDI=%d TDO=%d TCK=%d\n", nc_tdi, nc_tdo, nc_tck);
        printf("  Same timing as jtag_shift but NO explicit pull/push\n\n");

        /* All sizes must be multiples of 32 bits */
        uint32_t nc_bits[] = {
            64, 128, 256, 512, 1024, 1536, 2048, 4096, 8192, 32768,
        };
        int nc_count = sizeof(nc_bits) / sizeof(nc_bits[0]);

        for (int i = 0; i < nc_count; i++) {
            uint32_t bits = nc_bits[i];
            uint32_t words = bits / 32;
            size_t bytes = words * 4;

            printf("  %6u bits (%5zu bytes / %4u words TX=RX): ",
                   bits, bytes, words);
            fflush(stdout);

            int rc = test_jtag_nocount(pio, (uint)sm, nc_offset, clkdiv,
                                        bits, nc_tdi, nc_tdo, nc_tck);
            if (rc == 0) {
                printf("PASS\n");
            } else {
                printf("FAIL\n");
                printf("  Stopping nocount tests (PIO may be corrupted)\n");
                break;
            }
        }

        pio_sm_set_enabled(pio, (uint)sm, false);
        pio_remove_program(pio, &jtag_shift_nocount_program, nc_offset);
        printf("\n");
    }

    /* === jtag_shift sweep === */
    if (do_jtag) {
        uint js_offset = pio_add_program(pio, &jtag_shift_program);
        if (js_offset == PIO_ORIGIN_INVALID) {
            printf("FAIL: load jtag_shift program\n");
            goto cleanup;
        }

        /* NeTV2 pins */
        int tdi_pin = 27, tdo_pin = 22, tck_pin = 4;

        /* Test count-via-put FIRST (experimental), then count-in-DMA.
         * DMA failures corrupt PIO, so first mode to fail ends that mode.
         * Sizes start at 1024+ to avoid MIN_DMA_BYTES padding issues
         * with count-via-put mode (padding adds fake count word). */
        js_mode_t modes[] = {JS_MODE_COUNT_VIA_PUT, JS_MODE_COUNT_IN_DMA};
        int mode_count = sizeof(modes) / sizeof(modes[0]);

        uint32_t js_bits[] = {
            1024, 2048, 4096, 8192, 32768, 131072,
        };
        int js_count = sizeof(js_bits) / sizeof(js_bits[0]);

        for (int m = 0; m < mode_count; m++) {
            js_mode_t mode = modes[m];
            printf("=== jtag_shift DMA [%s] (clkdiv=%.1f) ===\n",
                   js_mode_name(mode), clkdiv);
            printf("  Pins: TDI=%d TDO=%d TCK=%d\n\n",
                   tdi_pin, tdo_pin, tck_pin);

            for (int i = 0; i < js_count; i++) {
                uint32_t bits = js_bits[i];
                uint32_t data_words = (bits + 31) / 32;
                uint32_t tx_words = (mode == JS_MODE_COUNT_VIA_PUT)
                    ? data_words : 1 + data_words;
                uint32_t rx_words = bits / 32 + 1;
                size_t tx_b = tx_words * 4;
                size_t rx_b = rx_words * 4;
                if (tx_b < 8) tx_b = 8;
                if (rx_b < 8) rx_b = 8;

                printf("  %6u bits (TX=%5zu B/%4u W, RX=%5zu B/%4u W): ",
                       bits, tx_b, tx_words, rx_b, rx_words);
                fflush(stdout);

                int rc = test_jtag_shift(pio, (uint)sm, js_offset, clkdiv,
                                         bits, tdi_pin, tdo_pin, tck_pin,
                                         mode);
                if (rc == 0) {
                    printf("PASS\n");
                } else {
                    printf("FAIL\n");
                    printf("  Stopping %s tests "
                           "(PIO may be corrupted)\n",
                           js_mode_name(mode));
                    /* Skip to next mode by breaking inner loop */
                    break;
                }
            }
            printf("\n");
        }

        pio_sm_set_enabled(pio, (uint)sm, false);
        pio_remove_program(pio, &jtag_shift_program, js_offset);
    }

    /* === Sequential jtag_shift test (no SM restart between chunks) === */
    if (do_sequential) {
        uint sq_offset = pio_add_program(pio, &jtag_shift_program);
        if (sq_offset == PIO_ORIGIN_INVALID) {
            printf("FAIL: load jtag_shift program\n");
            goto cleanup;
        }

        int tdi_pin = 27, tdo_pin = 22, tck_pin = 4;

        printf("=== Sequential jtag_shift DMA (clkdiv=%.1f) ===\n", clkdiv);
        printf("  %d consecutive 1024-bit chunks, NO SM restart between\n",
               seq_chunks);
        printf("  Pins: TDI=%d TDO=%d TCK=%d\n", tdi_pin, tdo_pin, tck_pin);
        const char *mode_str = seq_reconfig ? "per-chunk (256B + SM restart)"
                             : seq_restart_only ? "once (4096B), SM restart per chunk"
                             : "once (256B), no restart";
        printf("  config_xfer: %s\n\n", mode_str);

        struct timespec t_start, t_end;
        clock_gettime(CLOCK_MONOTONIC, &t_start);

        int rc = test_jtag_sequential(pio, (uint)sm, sq_offset, clkdiv,
                                       seq_chunks, tdi_pin, tdo_pin, tck_pin,
                                       seq_reconfig, seq_restart_only);

        clock_gettime(CLOCK_MONOTONIC, &t_end);
        double elapsed = (t_end.tv_sec - t_start.tv_sec) +
                          (t_end.tv_nsec - t_start.tv_nsec) / 1e9;
        uint64_t total_bits = (uint64_t)seq_chunks * 1024;

        if (rc == 0) {
            printf("  PASS: %d chunks x 1024 bits = %lu bits in %.3f s\n",
                   seq_chunks, (unsigned long)total_bits, elapsed);
            printf("  Throughput: %.1f kbit/s = %.1f kB/s\n",
                   total_bits / elapsed / 1000.0,
                   total_bits / elapsed / 8000.0);
            double ioctl_time = elapsed - (total_bits / (200e6 / clkdiv / 4));
            printf("  Estimated ioctl overhead: %.3f s (%.1f us/chunk)\n",
                   ioctl_time > 0 ? ioctl_time : 0,
                   ioctl_time > 0 ? ioctl_time / seq_chunks * 1e6 : 0);
        } else {
            printf("  FAIL after %.3f s\n", elapsed);
        }

        pio_sm_set_enabled(pio, (uint)sm, false);
        pio_remove_program(pio, &jtag_shift_program, sq_offset);
        printf("\n");
    }

cleanup:
    pio_sm_unclaim(pio, (uint)sm);
    pio_close(pio);
    return 0;
}

#else

int main(void)
{
    printf("Compiled without PIOLib support\n");
    return 1;
}

#endif
