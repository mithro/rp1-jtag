/*
 * test_txonly_minimal.c - Minimal TX-only DMA test with padding
 *
 * Tests the exact same approach as jtag_shift_txonly:
 * - autopush OFF
 * - TX DMA with count word + data + padding
 * - SM completion detected by RX FIFO level (explicit push)
 * - halt instruction prevents padding consumption
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdbool.h>

#include "piolib.h"
#include "pio/generated/jtag_shift.pio.h"

#define RP1_PIO_CLK_HZ  200000000
#define INSTR_PER_BIT   4
#define DMA_BUF_SIZE    4096

#define PIN_TDI 27
#define PIN_TDO 22
#define PIN_TCK 4
#define PIN_TMS 17

static int test_txonly(PIO pio, uint sm, uint offset,
                       uint32_t num_bits, uint32_t freq_hz,
                       bool use_padding)
{
    uint32_t num_data_words = (num_bits + 31) / 32;
    uint32_t sm_needs = 1 + num_data_words;
    uint32_t padding = use_padding ? (sm_needs / 3 + 2) : 0;
    uint32_t total_words = sm_needs + padding;
    uint32_t max_tx_words = DMA_BUF_SIZE / 4;
    if (total_words > max_tx_words - 1)
        total_words = max_tx_words - 1;

    size_t tx_bytes = total_words * 4;
    if (tx_bytes < 8) tx_bytes = 8;

    size_t alloc = (tx_bytes + 63) & ~(size_t)63;
    uint32_t *tx_buf = aligned_alloc(64, alloc);
    if (!tx_buf) return -1;
    memset(tx_buf, 0, tx_bytes);

    tx_buf[0] = num_bits - 1;
    for (uint32_t i = 1; i <= num_data_words; i++)
        tx_buf[i] = 0xAAAAAAAA;
    /* padding words are already zero from memset */

    printf("  %u bits: sm_needs=%u pad=%u total=%u tx_bytes=%zu\n",
           num_bits, sm_needs, padding, total_words, tx_bytes);

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

    /* Config DMA - both directions needed (kernel requires balanced setup) */
    int cfg_tx = pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, DMA_BUF_SIZE, 1);
    int cfg_rx = pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, DMA_BUF_SIZE, 1);
    printf("    config_xfer TX=%d RX=%d\n", cfg_tx, cfg_rx);
    if (cfg_tx < 0 || cfg_rx < 0) { free(tx_buf); return -1; }

    pio_sm_set_dmactrl(pio, sm, true, 0xC0000101);  /* threshold=1 */

    /* Start SM */
    pio_sm_exec(pio, sm, pio_encode_jmp(offset));
    pio_sm_set_enabled(pio, sm, true);

    /* TX DMA */
    errno = 0;
    int ret = pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM, tx_bytes, tx_buf);
    if (ret < 0) {
        printf("    TX DMA FAILED: %s (ret=%d)\n", strerror(errno), ret);
        pio_sm_set_enabled(pio, sm, false);
        free(tx_buf);
        return -1;
    }
    printf("    TX DMA OK (ret=%d)\n", ret);

    /* Wait for SM completion — check RX FIFO for the push word */
    int timeout_us = 0;
    int wait_limit = use_padding ? 100000 : 2000000;  /* 2 sec for no-pad */
    while (pio_sm_get_rx_fifo_level(pio, sm) == 0) {
        usleep(100);
        timeout_us += 100;
        if (timeout_us > wait_limit) {
            printf("    SM did not complete (no push word after %d us)\n",
                   timeout_us);
            printf("    TX_FIFO=%u RX_FIFO=%u\n",
                   pio_sm_get_tx_fifo_level(pio, sm),
                   pio_sm_get_rx_fifo_level(pio, sm));
            pio_sm_set_enabled(pio, sm, false);
            free(tx_buf);
            return use_padding ? -1 : 0;  /* Expected for no-pad */
        }
    }

    printf("    SM completed! (wait=%d us, tx_bytes=%zu)\n",
           timeout_us, tx_bytes);
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    free(tx_buf);
    return 0;
}

int main(void)
{
    printf("TX-only DMA with padding (minimal test)\n");
    printf("========================================\n\n");

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

    printf("Program loaded at offset %u (length=%d)\n\n", offset,
           jtag_shift_program.length);

    /* Set up pins */
    pio_select(pio);
    pio_gpio_init(pio, PIN_TCK);
    pio_gpio_init(pio, PIN_TDI);
    pio_gpio_init(pio, PIN_TDO);
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_TCK, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_TDI, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_TDO, 1, false);

    uint32_t freq_hz = 10000000;
    printf("Frequency: %u Hz\n\n", freq_hz);

    uint32_t sizes[] = {32, 128, 224, 512, 1024, 1536, 2048, 3072, 4096, 7680};
    int count = sizeof(sizes) / sizeof(sizes[0]);

    printf("=== Without padding (like test_txonly_hybrid2) ===\n");
    int passed_nopad = 0;
    for (int i = 0; i < count; i++) {
        int rc = test_txonly(pio, sm, offset, sizes[i], freq_hz, false);
        printf("    %s\n\n", rc == 0 ? "PASS" : "FAIL");
        if (rc == 0) passed_nopad++;
        /* Don't break — continue even after failure */
    }
    printf("=== No-pad: %d/%d passed ===\n\n", passed_nopad, count);

    printf("=== With padding ===\n");
    int passed_pad = 0;
    for (int i = 0; i < count; i++) {
        int rc = test_txonly(pio, sm, offset, sizes[i], freq_hz, true);
        printf("    %s\n\n", rc == 0 ? "PASS" : "FAIL");
        if (rc == 0) passed_pad++;
    }
    printf("=== Padded: %d/%d passed ===\n", passed_pad, count);

    pio_sm_set_enabled(pio, sm, false);
    pio_remove_program(pio, &jtag_shift_program, offset);
    pio_sm_unclaim(pio, sm);
    pio_close(pio);

    return (passed_nopad + passed_pad) == count * 2 ? 0 : 1;
}
