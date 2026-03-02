/*
 * test_txonly_dma.c - Test TX-only DMA with autopush OFF
 *
 * Tests that the jtag_shift PIO program works with:
 * - Autopush OFF (ISR wraps silently)
 * - TX DMA only (no RX DMA)
 * - Count-word protocol
 * - SM completion via RX FIFO level (explicit push)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "piolib.h"
#include "pio/generated/jtag_shift.pio.h"

#define RP1_PIO_CLK_HZ  200000000
#define INSTR_PER_BIT   4
#define FREQ_HZ         10000000
#define DMA_BUF_SIZE    4096
#define DMACTRL_TX      0xC0000104

/* Pins: TDI=27, TDO=22, TCK=4, TMS=17 */
#define PIN_TDI 27
#define PIN_TDO 22
#define PIN_TCK 4
#define PIN_TMS 17

static int test_txonly(PIO pio, uint sm, uint offset, uint32_t num_bits)
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
                    ((float)FREQ_HZ * INSTR_PER_BIT);
        if (div < 1.0f) div = 1.0f;
        sm_config_set_clkdiv(&c, div);
        pio_sm_init(pio, sm, offset, &c);
    }

    /* Config DMA */
    pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, DMA_BUF_SIZE, 1);
    pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, DMA_BUF_SIZE, 1);
    pio_sm_set_dmactrl(pio, sm, true, DMACTRL_TX);

    /* Start SM */
    pio_sm_exec(pio, sm, pio_encode_jmp(offset));
    pio_sm_set_enabled(pio, sm, true);

    /* TX DMA */
    printf("  TX DMA: %zu bytes (%u words)...", tx_bytes, (uint32_t)(tx_bytes/4));
    fflush(stdout);
    errno = 0;
    int ret = pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM, tx_bytes, tx_buf);
    int saved_errno = errno;
    printf(" ret=%d", ret);
    if (ret < 0) printf(" errno=%s", strerror(saved_errno));
    printf("\n");

    if (ret < 0) {
        printf("  TX DMA failed!\n");
        free(tx_buf);
        pio_sm_set_enabled(pio, sm, false);
        return -1;
    }

    /* Check FIFO levels right after TX DMA */
    uint32_t tx_level = pio_sm_get_tx_fifo_level(pio, sm);
    uint32_t rx_level = pio_sm_get_rx_fifo_level(pio, sm);
    printf("  After TX DMA: TX_FIFO=%u, RX_FIFO=%u\n", tx_level, rx_level);

    /* Wait for SM completion */
    int timeout_us = 0;
    while (pio_sm_get_rx_fifo_level(pio, sm) == 0) {
        usleep(10);
        timeout_us += 10;
        if (timeout_us > 100000) {  /* 100ms timeout */
            tx_level = pio_sm_get_tx_fifo_level(pio, sm);
            rx_level = pio_sm_get_rx_fifo_level(pio, sm);
            printf("  TIMEOUT after %d us: TX_FIFO=%u, RX_FIFO=%u\n",
                   timeout_us, tx_level, rx_level);
            free(tx_buf);
            pio_sm_set_enabled(pio, sm, false);
            return -1;
        }
    }

    printf("  SM completed in %d us, RX_FIFO=%u\n",
           timeout_us, pio_sm_get_rx_fifo_level(pio, sm));

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
                    ((float)FREQ_HZ * INSTR_PER_BIT);
        if (div < 1.0f) div = 1.0f;
        sm_config_set_clkdiv(&c, div);
        pio_sm_init(pio, sm, offset, &c);
    }

    free(tx_buf);
    return 0;
}

int main(void)
{
    printf("TX-only DMA Test (autopush OFF)\n");
    printf("================================\n\n");

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

    uint32_t test_sizes[] = {32, 64, 128, 224, 256, 512, 1024, 4096, 30720};
    int num_tests = sizeof(test_sizes) / sizeof(test_sizes[0]);
    int passed = 0;

    for (int i = 0; i < num_tests; i++) {
        printf("Test %d: %u bits\n", i+1, test_sizes[i]);
        int ret = test_txonly(pio, sm, offset, test_sizes[i]);
        if (ret == 0) {
            printf("  PASS\n\n");
            passed++;
        } else {
            printf("  FAIL\n\n");
        }
    }

    printf("=== %d/%d tests passed ===\n", passed, num_tests);

    pio_remove_program(pio, &jtag_shift_program, offset);
    pio_sm_unclaim(pio, sm);
    pio_close(pio);

    return passed == num_tests ? 0 : 1;
}
