/*
 * test_txonly_padded.c - Test TX-only DMA with threshold=1 + padding word
 *
 * Hypothesis: DMA with DREQ threshold=N fails to deliver the last N words.
 * Fix: Use threshold=1 and add 1 padding word to TX buffer.
 * DMA loses the padding word, delivers all real data.
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
#define FREQ_HZ         10000000
#define DMA_BUF_SIZE    4096
#define DMACTRL_THRESH1 0xC0000101  /* threshold=1, hi-pri, DREQ enable */

/* Pins: TDI=27, TDO=22, TCK=4, TMS=17 */
#define PIN_TDI 27
#define PIN_TDO 22
#define PIN_TCK 4
#define PIN_TMS 17

/* Read SM's X register */
static uint32_t read_x_register(PIO pio, uint sm)
{
    pio_sm_exec(pio, sm, pio_encode_mov(pio_isr, pio_null));
    pio_sm_exec(pio, sm, pio_encode_mov(pio_isr, pio_x));
    pio_sm_exec(pio, sm, pio_encode_push(false, false));
    return pio_sm_get(pio, sm);
}

static int test_padded(PIO pio, uint sm, uint offset, uint32_t num_bits,
                       bool use_padding)
{
    uint32_t num_data_words = (num_bits + 31) / 32;
    uint32_t tx_words = 1 + num_data_words;  /* count + data */
    if (use_padding) tx_words += 1;  /* extra padding word */
    size_t tx_bytes = tx_words * 4;
    if (tx_bytes < 8) tx_bytes = 8;

    /* Allocate and fill TX buffer */
    size_t alloc = (tx_bytes + 63) & ~(size_t)63;
    uint32_t *tx_buf = aligned_alloc(64, alloc);
    if (!tx_buf) return -1;
    memset(tx_buf, 0, tx_bytes);  /* zeros for data + padding */
    tx_buf[0] = num_bits - 1;  /* count word */
    /* Fill data with 0xAA pattern */
    for (uint32_t i = 1; i <= num_data_words; i++)
        tx_buf[i] = 0xAAAAAAAA;
    /* Padding word (if present) is already 0 from memset */

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

    /* Config DMA — threshold=1 */
    pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, DMA_BUF_SIZE, 1);
    pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, DMA_BUF_SIZE, 1);
    pio_sm_set_dmactrl(pio, sm, true, DMACTRL_THRESH1);

    /* Start SM */
    pio_sm_exec(pio, sm, pio_encode_jmp(offset));
    pio_sm_set_enabled(pio, sm, true);

    /* TX DMA */
    errno = 0;
    int ret = pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM, tx_bytes, tx_buf);
    if (ret < 0) {
        printf("TX DMA failed: %s\n", strerror(errno));
        free(tx_buf);
        pio_sm_set_enabled(pio, sm, false);
        return -1;
    }

    /* Wait for SM completion */
    int timeout_us = 0;
    while (pio_sm_get_rx_fifo_level(pio, sm) == 0) {
        usleep(10);
        timeout_us += 10;
        if (timeout_us > 100000) {
            /* Timeout — read diagnostics */
            pio_sm_set_enabled(pio, sm, false);
            uint32_t x_val = read_x_register(pio, sm);
            uint32_t bits_done = (num_bits - 1) - x_val;
            printf("TIMEOUT: X=%u (%u bits done, %u remaining)\n",
                   x_val, bits_done, x_val + 1);
            pio_sm_clear_fifos(pio, sm);
            free(tx_buf);
            return -1;
        }
    }

    /* Success */
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    free(tx_buf);
    return 0;
}

int main(void)
{
    printf("TX-only DMA Padded Test (threshold=1 + padding)\n");
    printf("================================================\n\n");

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

    /* Test 1: Verify baseline failure (no padding, threshold=1) */
    printf("=== Baseline: threshold=1, NO padding ===\n");
    uint32_t baseline_sizes[] = {256, 480, 512, 1024, 4096, 30720};
    int bl_count = sizeof(baseline_sizes) / sizeof(baseline_sizes[0]);

    for (int i = 0; i < bl_count; i++) {
        printf("  %5u bits (no pad): ", baseline_sizes[i]);
        fflush(stdout);
        int rc = test_padded(pio, sm, offset, baseline_sizes[i], false);
        printf("%s\n", rc == 0 ? "PASS" : "FAIL");
    }
    printf("\n");

    /* Test 2: Padded (threshold=1 + 1 padding word) */
    printf("=== Fix: threshold=1, WITH padding word ===\n");
    uint32_t padded_sizes[] = {32, 64, 128, 224, 256, 288, 320, 448, 480,
                               512, 544, 1024, 2048, 4096, 8192, 30720};
    int pd_count = sizeof(padded_sizes) / sizeof(padded_sizes[0]);

    int passed = 0;
    for (int i = 0; i < pd_count; i++) {
        printf("  %5u bits (padded): ", padded_sizes[i]);
        fflush(stdout);
        int rc = test_padded(pio, sm, offset, padded_sizes[i], true);
        printf("%s\n", rc == 0 ? "PASS" : "FAIL");
        if (rc == 0) passed++;
    }
    printf("\n=== Padded: %d/%d passed ===\n", passed, pd_count);

    /* Restore SM config */
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

    pio_remove_program(pio, &jtag_shift_program, offset);
    pio_sm_unclaim(pio, sm);
    pio_close(pio);

    return passed == pd_count ? 0 : 1;
}
