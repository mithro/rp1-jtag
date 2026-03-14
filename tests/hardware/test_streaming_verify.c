/*
 * test_streaming_verify.c - Verify streaming DMA data integrity
 *
 * Sends data through the JTAG SM with autopush ON, capturing TDO
 * via RX DMA.  Uses sequential (per-chunk) TX then RX to avoid
 * the concurrent DMA timeout issue with medium-sized transfers.
 *
 * Tests multiple chunk sizes to see which lose data.
 *
 * No FPGA target needed — this test drives TDI/TCK into the void
 * and captures whatever appears on TDO (floating or tied).
 * The key metric is: does RX DMA return the expected number of words?
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include "piolib.h"
#include "pio/generated/jtag_shift.pio.h"

#define DMACTRL_DEFAULT 0xC0000108  /* DREQ_EN | high_pri | threshold=8 */

static int test_streaming_rx(PIO pio, uint sm, uint offset,
                              uint32_t num_words, float clkdiv,
                              uint32_t chunk_bytes)
{
    size_t xfer_bytes = (size_t)num_words * 4;
    uint32_t chunk_words = chunk_bytes / 4;

    printf("  words=%-6u div=%-4.1f chunk=%-5u: ",
           num_words, clkdiv, chunk_bytes);
    fflush(stdout);

    /* Allocate TX and RX buffers */
    size_t alloc = (xfer_bytes + 63) & ~(size_t)63;
    uint32_t *tx_buf = aligned_alloc(64, alloc);
    uint32_t *rx_buf = aligned_alloc(64, alloc);
    if (!tx_buf || !rx_buf) {
        printf("ALLOC FAIL\n");
        free(tx_buf); free(rx_buf);
        return -1;
    }

    /* Fill TX with recognizable pattern */
    for (uint32_t i = 0; i < num_words; i++)
        tx_buf[i] = 0xA5000000 | i;
    memset(rx_buf, 0xDD, alloc);  /* poison */

    /* SM init: autopull ON, autopush ON */
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);

    pio_select(pio);
    pio_sm_config c = jtag_shift_program_get_default_config(offset);
    sm_config_set_out_shift(&c, true, true, 32);   /* autopull ON */
    sm_config_set_in_shift(&c, true, true, 32);    /* autopush ON */
    sm_config_set_clkdiv(&c, clkdiv);
    pio_sm_init(pio, sm, offset, &c);

    /* Configure TX and RX DMA with chunk-sized bounce buffers */
    if (pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, chunk_bytes, 1) < 0 ||
        pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, chunk_bytes, 1) < 0) {
        printf("CONFIG_XFER FAIL\n");
        free(tx_buf); free(rx_buf);
        return -1;
    }
    pio_sm_set_dmactrl(pio, sm, true, DMACTRL_DEFAULT);
    pio_sm_set_dmactrl(pio, sm, false, DMACTRL_DEFAULT);

    /* Streaming mode: X=0xFFFFFFFF, jump to loop body (offset+2) */
    pio_sm_exec(pio, sm, pio_encode_mov_not(pio_x, pio_null));
    pio_sm_exec(pio, sm, pio_encode_jmp(offset + 2));
    pio_sm_set_enabled(pio, sm, true);

    /* Sequential per-chunk TX then RX.
     * For each chunk: send TX data, wait for SM, read RX data. */
    int tx_fail = 0, rx_fail = 0;
    uint32_t words_sent = 0, words_rcvd = 0;

    uint8_t *tx_ptr = (uint8_t *)tx_buf;
    uint8_t *rx_ptr = (uint8_t *)rx_buf;
    size_t remaining = xfer_bytes;

    while (remaining > 0) {
        size_t chunk = remaining > chunk_bytes ? chunk_bytes : remaining;

        /* TX: send chunk to SM */
        errno = 0;
        int rc = pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM, chunk, tx_ptr);
        if (rc < 0) {
            if (!tx_fail)
                printf("TX FAIL at word %u (%s) ", words_sent, strerror(errno));
            tx_fail++;
            break;
        }
        words_sent += chunk / 4;

        /* Wait for SM to process this chunk.
         * At div=5.0: 7 words × 32 bits × 4 cycles × 5ns × 5 = 22.4 µs
         * Use 2× safety margin. */
        uint32_t wait_us = (uint32_t)((double)(chunk / 4) * 32.0 * 4.0 *
                                       clkdiv / 200.0) * 2 + 10;
        usleep(wait_us);

        /* RX: read chunk from SM */
        errno = 0;
        rc = pio_sm_xfer_data(pio, sm, PIO_DIR_FROM_SM, chunk, rx_ptr);
        if (rc < 0) {
            if (!rx_fail)
                printf("RX FAIL at word %u (%s) ", words_rcvd, strerror(errno));
            rx_fail++;
            break;
        }
        words_rcvd += chunk / 4;

        tx_ptr += chunk;
        rx_ptr += chunk;
        remaining -= chunk;
    }

    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);

    if (tx_fail || rx_fail) {
        printf("sent=%u rcvd=%u FAIL\n", words_sent, words_rcvd);
        free(tx_buf); free(rx_buf);
        return -1;
    }

    /* Analyze RX data */
    int poison = 0, zero = 0, nonzero = 0;
    for (uint32_t i = 0; i < num_words; i++) {
        if (rx_buf[i] == 0xDDDDDDDD)
            poison++;
        else if (rx_buf[i] == 0)
            zero++;
        else
            nonzero++;
    }

    printf("sent=%u rcvd=%u (nonzero=%d zero=%d poison=%d)",
           words_sent, words_rcvd, nonzero, zero, poison);

    if (words_rcvd == num_words && poison == 0) {
        printf(" PASS\n");
        free(tx_buf); free(rx_buf);
        return 0;
    } else {
        printf(" FAIL\n");
        /* Print first/last few words for debugging */
        printf("    TX[0..3]: %08x %08x %08x %08x\n",
               tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3]);
        printf("    RX[0..3]: %08x %08x %08x %08x\n",
               rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
        free(tx_buf); free(rx_buf);
        return -1;
    }
}

int main(void)
{
    printf("Streaming DMA TX+RX Verification\n");
    printf("=================================\n\n");

    PIO pio = pio_open(0);
    if (!pio) {
        fprintf(stderr, "pio_open failed\n");
        return 1;
    }

    int sm = pio_claim_unused_sm(pio, true);
    if (sm < 0) {
        fprintf(stderr, "no free SM\n");
        pio_close(pio);
        return 1;
    }

    pio_select(pio);
    uint offset = pio_add_program(pio, &jtag_shift_program);

    /* Test at div=5.0 (10 MHz requested, ~20 MHz actual) */
    float div = 5.0f;
    int pass = 0, fail = 0;

    printf("Test 1: chunk=28 (7 words, < FIFO depth)\n");
    struct { uint32_t words; uint32_t chunk; } tests[] = {
        {    16,  28 },    /*     64 B */
        {   256,  28 },    /*   1024 B */
        {  1024,  28 },    /*   4096 B */
        {  4096,  28 },    /*  16384 B */
        {  9564,  28 },    /*  38256 B (bitstream segment) */
    };

    for (size_t t = 0; t < sizeof(tests)/sizeof(tests[0]); t++) {
        int rc = test_streaming_rx(pio, (uint)sm, offset,
                                    tests[t].words, div, tests[t].chunk);
        if (rc == 0) pass++; else fail++;
    }

    printf("\nTest 2: various chunk sizes (9564 words = 38256 bytes)\n");
    uint32_t chunk_sizes[] = {
        28, 32, 64, 128, 256, 512, 1024, 2048, 4096
    };

    for (size_t t = 0; t < sizeof(chunk_sizes)/sizeof(chunk_sizes[0]); t++) {
        int rc = test_streaming_rx(pio, (uint)sm, offset,
                                    9564, div, chunk_sizes[t]);
        if (rc == 0) pass++; else fail++;
    }

    printf("\nTest 3: large single DMA transfer\n");
    struct { uint32_t words; uint32_t chunk; } tests3[] = {
        {  9564, 9564 * 4 },   /* 38256 B in one shot */
    };

    for (size_t t = 0; t < sizeof(tests3)/sizeof(tests3[0]); t++) {
        int rc = test_streaming_rx(pio, (uint)sm, offset,
                                    tests3[t].words, div, tests3[t].chunk);
        if (rc == 0) pass++; else fail++;
    }

    printf("\n=================================\n");
    printf("Results: %d pass, %d fail\n", pass, fail);

    pio_close(pio);
    return fail > 0 ? 1 : 0;
}
