/*
 * test_concurrent_dma.c - TX-only DMA with autopush OFF
 *
 * Root cause confirmed: concurrent DREQ-paced DMA on the same SM times out
 * when the SM runs slowly (any clkdiv > 1.0). The benchmark only works
 * because it runs at div=1.0 with a 3-cycle loopback (effectively a wire).
 *
 * For bitstream programming (tdo==NULL), we only need TX DMA. With autopush
 * OFF, the SM never stalls on the RX FIFO, so TX DMA can run at full speed.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>

#include "piolib.h"
#include "pio/generated/jtag_shift.pio.h"

static double time_ms(struct timespec *start, struct timespec *end)
{
    return (end->tv_sec - start->tv_sec) * 1000.0 +
           (end->tv_nsec - start->tv_nsec) / 1e6;
}

/*
 * Test TX-only DMA with jtag_shift in streaming mode.
 * Autopush OFF — SM shifts TDI out and samples TDO into ISR,
 * but ISR wraps around silently (no RX FIFO push, no stall).
 */
static int test_txonly(PIO pio, uint sm, uint offset,
                       uint32_t num_words, float clkdiv, uint32_t buf_size)
{
    size_t xfer_bytes = (size_t)num_words * 4;

    printf("  TX-only words=%-6u div=%-5.1f buf=%u: ", num_words, clkdiv, buf_size);
    fflush(stdout);

    uint32_t *tx_buf = aligned_alloc(64, (xfer_bytes + 63) & ~63u);
    if (!tx_buf) {
        printf("ALLOC FAIL\n");
        return -1;
    }
    for (uint32_t i = 0; i < num_words; i++)
        tx_buf[i] = 0xAAAAAAAA;

    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);

    pio_sm_config c = jtag_shift_program_get_default_config(offset);
    sm_config_set_out_shift(&c, true, true, 32);    /* RIGHT shift, autopull ON */
    sm_config_set_in_shift(&c, true, false, 32);    /* RIGHT shift, autopush OFF */
    sm_config_set_clkdiv(&c, clkdiv);
    pio_sm_init(pio, sm, offset, &c);

    /* TX-only DMA config */
    if (pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, buf_size, 1) < 0) {
        printf("CONFIG_XFER FAIL\n");
        free(tx_buf);
        return -1;
    }
    pio_sm_set_dmactrl(pio, sm, true, 0xC0000108);  /* TX threshold=8 */

    /* Streaming mode: X = 0xFFFFFFFF, jump to loop body */
    pio_sm_exec(pio, sm, pio_encode_mov_not(pio_x, pio_null));
    pio_sm_exec(pio, sm, pio_encode_jmp(offset + 2));

    pio_sm_set_enabled(pio, sm, true);

    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    errno = 0;
    int ret = pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM, xfer_bytes, tx_buf);
    int err = errno;

    clock_gettime(CLOCK_MONOTONIC, &t1);

    pio_sm_set_enabled(pio, sm, false);

    double ms = time_ms(&t0, &t1);
    double kbps = (ret == 0) ? (xfer_bytes / 1024.0) / (ms / 1000.0) : 0;

    if (ret == 0) {
        printf("TX=OK %7.1fms %7.0f kB/s PASS\n", ms, kbps);
    } else {
        printf("TX=%d/%s FAIL\n", ret, strerror(err));
    }

    free(tx_buf);
    return ret;
}

/*
 * Test TX-only with xfer_data loop for transfers > buf_size.
 */
static int test_txonly_loop(PIO pio, uint sm, uint offset,
                            uint32_t num_words, float clkdiv, uint32_t buf_size)
{
    size_t total_bytes = (size_t)num_words * 4;

    printf("  TX-loop words=%-6u div=%-5.1f buf=%u: ", num_words, clkdiv, buf_size);
    fflush(stdout);

    uint32_t *tx_buf = aligned_alloc(64, (total_bytes + 63) & ~63u);
    if (!tx_buf) {
        printf("ALLOC FAIL\n");
        return -1;
    }
    for (uint32_t i = 0; i < num_words; i++)
        tx_buf[i] = 0xAAAAAAAA;

    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);

    pio_sm_config c = jtag_shift_program_get_default_config(offset);
    sm_config_set_out_shift(&c, true, true, 32);
    sm_config_set_in_shift(&c, true, false, 32);    /* autopush OFF */
    sm_config_set_clkdiv(&c, clkdiv);
    pio_sm_init(pio, sm, offset, &c);

    if (pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, buf_size, 1) < 0) {
        printf("CONFIG_XFER FAIL\n");
        free(tx_buf);
        return -1;
    }
    pio_sm_set_dmactrl(pio, sm, true, 0xC0000108);

    /* Streaming mode */
    pio_sm_exec(pio, sm, pio_encode_mov_not(pio_x, pio_null));
    pio_sm_exec(pio, sm, pio_encode_jmp(offset + 2));

    pio_sm_set_enabled(pio, sm, true);

    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    uint8_t *ptr = (uint8_t *)tx_buf;
    size_t remaining = total_bytes;
    int ret = 0;
    int err = 0;

    while (remaining > 0) {
        size_t chunk = remaining > buf_size ? buf_size : remaining;
        errno = 0;
        ret = pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM, chunk, ptr);
        err = errno;
        if (ret < 0)
            break;
        ptr += chunk;
        remaining -= chunk;
    }

    clock_gettime(CLOCK_MONOTONIC, &t1);

    pio_sm_set_enabled(pio, sm, false);

    double ms = time_ms(&t0, &t1);
    double kbps = (ret == 0) ? (total_bytes / 1024.0) / (ms / 1000.0) : 0;

    if (ret == 0) {
        printf("TX=OK %7.1fms %7.0f kB/s PASS\n", ms, kbps);
    } else {
        printf("TX=%d/%s after %zu/%zu bytes FAIL\n",
               ret, strerror(err), total_bytes - remaining, total_bytes);
    }

    free(tx_buf);
    return ret;
}

int main(void)
{
    printf("TX-only DMA with autopush OFF\n");
    printf("=============================\n\n");

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
        fprintf(stderr, "Failed to load jtag_shift program\n");
        pio_close(pio);
        return 1;
    }
    printf("jtag_shift loaded at offset %u\n\n", offset);

    int passed = 0, total = 0;

    /* --- Single xfer_data call (total <= buf_size) --- */
    printf("--- TX-only single call: buf_size=256KB ---\n");
    uint32_t sizes[] = {8, 32, 128, 1024, 4096, 16384, 65536};
    for (int i = 0; i < (int)(sizeof(sizes)/sizeof(sizes[0])); i++) {
        total++; if (test_txonly(pio, sm, offset, sizes[i], 1.0f, 262144) == 0) passed++;
    }

    printf("\n--- TX-only single call: div=5.0 (10 MHz TCK) ---\n");
    for (int i = 0; i < (int)(sizeof(sizes)/sizeof(sizes[0])); i++) {
        total++; if (test_txonly(pio, sm, offset, sizes[i], 5.0f, 262144) == 0) passed++;
    }

    printf("\n--- TX-only single call: div=2.5 (20 MHz TCK) ---\n");
    for (int i = 0; i < (int)(sizeof(sizes)/sizeof(sizes[0])); i++) {
        total++; if (test_txonly(pio, sm, offset, sizes[i], 2.5f, 262144) == 0) passed++;
    }

    /* --- Loop xfer_data for large transfers (simulating 3.8 MB bitstream) --- */
    printf("\n--- TX-only loop: 256K words (1 MB), various divs, buf=256KB ---\n");
    float divs[] = {1.0f, 2.5f, 5.0f, 10.0f};
    for (int i = 0; i < (int)(sizeof(divs)/sizeof(divs[0])); i++) {
        total++; if (test_txonly_loop(pio, sm, offset, 262144, divs[i], 262144) == 0) passed++;
    }

    printf("\n--- TX-only loop: 1M words (4 MB ≈ bitstream), div=5.0 ---\n");
    total++; if (test_txonly_loop(pio, sm, offset, 1048576, 5.0f, 262144) == 0) passed++;

    printf("\n--- TX-only loop: 1M words, div=2.5 ---\n");
    total++; if (test_txonly_loop(pio, sm, offset, 1048576, 2.5f, 262144) == 0) passed++;

    pio_remove_program(pio, &jtag_shift_program, offset);

    printf("\n=== %d/%d passed ===\n", passed, total);

    pio_sm_set_enabled(pio, sm, false);
    pio_sm_unclaim(pio, sm);
    pio_close(pio);

    return passed == total ? 0 : 1;
}
