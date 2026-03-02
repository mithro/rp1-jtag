/*
 * test_streaming.c - Test streaming mode (no count word) with concurrent DMA
 *
 * Streaming mode: exec mov x,!null + exec jmp offset+2 to skip the count
 * word protocol. Autopush ON for 1:1 TX:RX word ratio.
 * Concurrent TX+RX DMA via pthreads (matches rpi5-rp1-pio-bench).
 *
 * This tests the plan's core approach: continuous SM with concurrent DMA.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>

#include "piolib.h"
#include "pio/generated/jtag_shift.pio.h"

#define RP1_PIO_CLK_HZ  200000000
#define INSTR_PER_BIT   4
#define DMA_BUF_SIZE    (256 * 1024)  /* 256 KB like benchmark */

#define PIN_TDI 27
#define PIN_TDO 22
#define PIN_TCK 4
#define PIN_TMS 17

typedef struct {
    PIO pio;
    uint sm;
    uint dir;
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

static double elapsed_ms(struct timespec *start, struct timespec *end)
{
    return (end->tv_sec - start->tv_sec) * 1000.0 +
           (end->tv_nsec - start->tv_nsec) / 1e6;
}

static int test_streaming(PIO pio, uint sm, uint offset,
                          uint32_t num_bits, uint32_t freq_hz)
{
    /* Round up to multiple of 32 bits (autopush boundary) */
    uint32_t padded_bits = (num_bits + 31) & ~31u;
    uint32_t num_words = padded_bits / 32;
    size_t xfer_bytes = num_words * 4;

    printf("  %u bits (padded=%u, %u words, %zu bytes):\n",
           num_bits, padded_bits, num_words, xfer_bytes);

    /* Allocate TX and RX buffers */
    size_t alloc = (xfer_bytes + 63) & ~(size_t)63;
    uint32_t *tx_buf = aligned_alloc(64, alloc);
    uint32_t *rx_buf = aligned_alloc(64, alloc);
    if (!tx_buf || !rx_buf) {
        free(tx_buf); free(rx_buf);
        return -1;
    }
    memset(tx_buf, 0, xfer_bytes);
    memset(rx_buf, 0, xfer_bytes);

    /* Fill TX with test pattern */
    for (uint32_t i = 0; i < num_words; i++)
        tx_buf[i] = 0xAAAAAAAA;

    /* Configure SM: autopull ON, autopush ON, 32-bit threshold */
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
        sm_config_set_in_shift(&c, true, true, 32);    /* autopush ON */
        float div = (float)RP1_PIO_CLK_HZ /
                    ((float)freq_hz * INSTR_PER_BIT);
        if (div < 1.0f) div = 1.0f;
        sm_config_set_clkdiv(&c, div);
        pio_sm_init(pio, sm, offset, &c);
    }

    /* Config DMA for both directions */
    size_t dma_buf = xfer_bytes > DMA_BUF_SIZE ? DMA_BUF_SIZE : xfer_bytes;
    if (dma_buf < 8) dma_buf = 8;
    int cfg_tx = pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, dma_buf, 1);
    int cfg_rx = pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, dma_buf, 1);
    printf("    config_xfer TX=%d RX=%d (buf=%zu)\n", cfg_tx, cfg_rx, dma_buf);
    if (cfg_tx < 0 || cfg_rx < 0) {
        free(tx_buf); free(rx_buf);
        return -1;
    }

    /* DMACTRL: threshold=4, high priority */
    pio_sm_set_dmactrl(pio, sm, true,  0xC0000104);
    pio_sm_set_dmactrl(pio, sm, false, 0xC0000104);

    /* Streaming mode: set X = 0xFFFFFFFF, jump to loop body */
    pio_sm_exec(pio, sm, pio_encode_mov_not(pio_x, pio_null));
    pio_sm_exec(pio, sm, pio_encode_jmp(offset + 2));

    pio_sm_set_enabled(pio, sm, true);

    struct timespec t_start, t_end;
    clock_gettime(CLOCK_MONOTONIC, &t_start);

    /* Concurrent TX + RX DMA */
    xfer_args_t tx_args = {pio, sm, PIO_DIR_TO_SM,
                           xfer_bytes, tx_buf, 0, 0};
    xfer_args_t rx_args = {pio, sm, PIO_DIR_FROM_SM,
                           xfer_bytes, rx_buf, 0, 0};

    pthread_t t_tx, t_rx;
    pthread_create(&t_tx, NULL, xfer_thread, &tx_args);
    pthread_create(&t_rx, NULL, xfer_thread, &rx_args);
    pthread_join(t_tx, NULL);
    pthread_join(t_rx, NULL);

    clock_gettime(CLOCK_MONOTONIC, &t_end);

    pio_sm_set_enabled(pio, sm, false);

    double ms = elapsed_ms(&t_start, &t_end);
    double kbs = (num_bits / 8.0) / (ms / 1000.0) / 1024.0;

    printf("    TX: ret=%d err=%s\n", tx_args.ret,
           tx_args.ret < 0 ? strerror(tx_args.err) : "OK");
    printf("    RX: ret=%d err=%s\n", rx_args.ret,
           rx_args.ret < 0 ? strerror(rx_args.err) : "OK");
    printf("    Time: %.2f ms, %.0f kB/s\n", ms, kbs);

    int result = (tx_args.ret == 0 && rx_args.ret == 0) ? 0 : -1;
    printf("    %s\n", result == 0 ? "PASS" : "FAIL");

    free(tx_buf);
    free(rx_buf);
    return result;
}

int main(void)
{
    printf("Streaming Mode + Concurrent DMA Test\n");
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

    printf("Program loaded at offset %u\n\n", offset);

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

    /* Test sizes from small to large */
    uint32_t sizes[] = {32, 224, 1024, 7680, 30720, 245760, 3840000};
    int count = sizeof(sizes) / sizeof(sizes[0]);
    int passed = 0;

    for (int i = 0; i < count; i++) {
        int rc = test_streaming(pio, sm, offset, sizes[i], freq_hz);
        if (rc == 0) passed++;
        printf("\n");
    }
    printf("=== %d/%d passed ===\n", passed, count);

    pio_sm_set_enabled(pio, sm, false);
    pio_remove_program(pio, &jtag_shift_program, offset);
    pio_sm_unclaim(pio, sm);
    pio_close(pio);

    return passed == count ? 0 : 1;
}
