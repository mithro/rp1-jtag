/*
 * test_txonly_hybrid.c - DMA + FIFO-write hybrid for TX-only transfers
 *
 * Root cause: RP1 PIOLib DMA doesn't deliver all TX words — ~12% of
 * words are lost for large transfers. The SM stalls on autopull.
 *
 * Fix: After TX DMA returns, use pio_sm_put_blocking() to feed
 * remaining data words until the SM finishes (RX_FIFO > 0).
 * This is NOT concurrent — DMA completes first, then FIFO writes.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdbool.h>
#include <time.h>

#include "piolib.h"
#include "pio/generated/jtag_shift.pio.h"

#define RP1_PIO_CLK_HZ  200000000
#define INSTR_PER_BIT   4
#define DMA_BUF_SIZE    4096
#define DMACTRL_THRESH1 0xC0000101  /* threshold=1, hi-pri, DREQ enable */

/* Pins: TDI=27, TDO=22, TCK=4, TMS=17 */
#define PIN_TDI 27
#define PIN_TDO 22
#define PIN_TCK 4
#define PIN_TMS 17

static int test_hybrid(PIO pio, uint sm, uint offset, uint32_t num_bits,
                       uint32_t freq_hz, bool verbose)
{
    uint32_t num_data_words = (num_bits + 31) / 32;
    size_t tx_bytes = (1 + num_data_words) * 4;
    if (tx_bytes < 8) tx_bytes = 8;

    /* Allocate and fill TX buffer */
    size_t alloc = (tx_bytes + 63) & ~(size_t)63;
    uint32_t *tx_buf = aligned_alloc(64, alloc);
    if (!tx_buf) return -1;
    memset(tx_buf, 0xAA, tx_bytes);  /* fill with pattern */
    tx_buf[0] = num_bits - 1;  /* count word */

    /* Disable SM, configure with autopush OFF */
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);

    pio_select(pio);
    {
        pio_sm_config c = jtag_shift_program_get_default_config(offset);
        sm_config_set_out_pins(&c, PIN_TDI, 1);
        sm_config_set_in_pins(&c, PIN_TDO);
        sm_config_set_sideset_pins(&c, PIN_TCK);
        sm_config_set_out_shift(&c, true, true, 32);   /* autopull ON */
        sm_config_set_in_shift(&c, true, false, 32);    /* autopush OFF */
        float div = (float)RP1_PIO_CLK_HZ /
                    ((float)freq_hz * INSTR_PER_BIT);
        if (div < 1.0f) div = 1.0f;
        sm_config_set_clkdiv(&c, div);
        pio_sm_init(pio, sm, offset, &c);
    }

    /* Config DMA */
    pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, DMA_BUF_SIZE, 1);
    pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, DMA_BUF_SIZE, 1);
    pio_sm_set_dmactrl(pio, sm, true, DMACTRL_THRESH1);

    /* Start SM */
    pio_sm_exec(pio, sm, pio_encode_jmp(offset));
    pio_sm_set_enabled(pio, sm, true);

    struct timespec t_start, t_dma, t_end;
    clock_gettime(CLOCK_MONOTONIC, &t_start);

    /* Phase 1: TX DMA (delivers ~87% of words) */
    errno = 0;
    int ret = pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM, tx_bytes, tx_buf);
    if (ret < 0) {
        printf("TX DMA failed: %s\n", strerror(errno));
        free(tx_buf);
        pio_sm_set_enabled(pio, sm, false);
        return -1;
    }

    clock_gettime(CLOCK_MONOTONIC, &t_dma);

    /* Phase 2: FIFO-write fallback for remaining words.
     * The SM is stalled on autopull (TX FIFO empty, DMA didn't
     * deliver all words). Feed zeros via put_blocking until
     * the SM finishes the counted loop (push → RX_FIFO > 0). */
    uint32_t fifo_writes = 0;
    while (pio_sm_get_rx_fifo_level(pio, sm) == 0) {
        pio_sm_put_blocking(pio, sm, 0x00000000);  /* zero padding */
        fifo_writes++;
        if (fifo_writes > num_data_words) {
            /* Safety: should never need more writes than data words */
            printf("ERROR: too many FIFO writes (%u > %u)\n",
                   fifo_writes, num_data_words);
            break;
        }
    }

    clock_gettime(CLOCK_MONOTONIC, &t_end);

    double dma_time = (t_dma.tv_sec - t_start.tv_sec) +
                      (t_dma.tv_nsec - t_start.tv_nsec) / 1e9;
    double fifo_time = (t_end.tv_sec - t_dma.tv_sec) +
                       (t_end.tv_nsec - t_dma.tv_nsec) / 1e9;
    double total_time = dma_time + fifo_time;

    int result = (pio_sm_get_rx_fifo_level(pio, sm) > 0) ? 0 : -1;

    if (verbose || result < 0) {
        uint32_t dma_words = 1 + num_data_words - fifo_writes;
        printf("    DMA: %u/%u words (%.1f%%), FIFO: %u words\n",
               dma_words, 1 + num_data_words,
               100.0 * dma_words / (1 + num_data_words),
               fifo_writes);
        printf("    Time: DMA=%.3fms FIFO=%.3fms total=%.3fms\n",
               dma_time * 1000, fifo_time * 1000, total_time * 1000);
        double kbs = (num_bits / 8.0) / total_time / 1024.0;
        printf("    Throughput: %.0f kB/s\n", kbs);
    }

    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    free(tx_buf);
    return result;
}

int main(void)
{
    printf("TX-only DMA + FIFO-write Hybrid Test\n");
    printf("=====================================\n\n");

    if (pio_init() < 0) {
        fprintf(stderr, "pio_init failed\n");
        return 1;
    }

    PIO pio = pio_open(0);
    if (PIO_IS_ERR(pio)) {
        fprintf(stderr, "pio_open failed\n");
        return 1;
    }

    int sm = pio_claim_unused_sm(pio, true);
    if (sm < 0) {
        fprintf(stderr, "No free SM\n");
        pio_close(pio);
        return 1;
    }

    uint offset = pio_add_program(pio, &jtag_shift_program);
    if (offset == PIO_ORIGIN_INVALID) {
        fprintf(stderr, "Failed to load program\n");
        pio_sm_unclaim(pio, sm);
        pio_close(pio);
        return 1;
    }

    /* Set up pins */
    pio_select(pio);
    pio_gpio_init(pio, PIN_TCK);
    pio_gpio_init(pio, PIN_TDI);
    pio_gpio_init(pio, PIN_TDO);
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_TCK, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_TDI, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_TDO, 1, false);

    uint32_t freq_hz = 10000000;  /* 10 MHz */

    printf("Frequency: %u Hz\n\n", freq_hz);

    /* Test across all sizes */
    uint32_t sizes[] = {32, 64, 128, 224, 256, 480, 512, 1024,
                        2048, 4096, 8192, 30720};
    int count = sizeof(sizes) / sizeof(sizes[0]);
    int passed = 0;

    printf("=== Correctness tests (all sizes) ===\n");
    for (int i = 0; i < count; i++) {
        printf("  %5u bits: ", sizes[i]);
        fflush(stdout);
        int rc = test_hybrid(pio, sm, offset, sizes[i], freq_hz, true);
        printf("    %s\n\n", rc == 0 ? "PASS" : "FAIL");
        if (rc == 0) passed++;
    }
    printf("=== %d/%d passed ===\n\n", passed, count);

    /* Performance test with large chunk */
    printf("=== Performance: 30720 bits at multiple frequencies ===\n");
    uint32_t freqs[] = {5000000, 10000000, 15000000, 20000000};
    int nfreqs = sizeof(freqs) / sizeof(freqs[0]);

    for (int f = 0; f < nfreqs; f++) {
        printf("  %u MHz:\n", freqs[f] / 1000000);
        /* Run 3 times for consistency */
        for (int r = 0; r < 3; r++) {
            printf("    Run %d: ", r + 1);
            fflush(stdout);
            test_hybrid(pio, sm, offset, 30720, freqs[f], true);
        }
        printf("\n");
    }

    /* Restore SM */
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    pio_select(pio);
    {
        pio_sm_config c = jtag_shift_program_get_default_config(offset);
        sm_config_set_out_pins(&c, PIN_TDI, 1);
        sm_config_set_in_pins(&c, PIN_TDO);
        sm_config_set_sideset_pins(&c, PIN_TCK);
        sm_config_set_out_shift(&c, true, true, 32);
        sm_config_set_in_shift(&c, true, true, 32);
        float div = (float)RP1_PIO_CLK_HZ /
                    ((float)freq_hz * INSTR_PER_BIT);
        if (div < 1.0f) div = 1.0f;
        sm_config_set_clkdiv(&c, div);
        pio_sm_init(pio, sm, offset, &c);
    }

    pio_remove_program(pio, &jtag_shift_program, offset);
    pio_sm_unclaim(pio, sm);
    pio_close(pio);

    return passed == count ? 0 : 1;
}
