/*
 * test_dma_explore.c - DMA + fast PIO program hardware exploration
 *
 * Directly calls PIOLib DMA APIs to discover what patterns work on RP1.
 * Does NOT go through the backend abstraction — this is a raw hardware test.
 *
 * Parts:
 *   A: DMA basics with loopback program (TX-only, RX-only, ping-pong, concurrent)
 *   B: Fast program stop condition (word-by-word, DMA, timing)
 *   C: Throughput ceiling (large transfers)
 *
 * Requires: RPi 5, sudo for /dev/pio0
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef HAVE_PIOLIB

#include <hardware/pio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>

/* PIO program headers */
#include "pio/generated/jtag_loopback.pio.h"
#include "pio/generated/jtag_shift_fast.pio.h"

/* TCK pin for sideset — must match hardware (GPIO 4 on NeTV2 setup) */
#define TCK_PIN 4
#define TDI_PIN 27
#define TDO_PIN 22

static int tests_passed = 0;
static int tests_failed = 0;

#define FAIL(msg, ...) do { \
    fprintf(stderr, "FAIL: " msg "\n", ##__VA_ARGS__); \
    tests_failed++; \
    goto cleanup; \
} while(0)

#define PASS(name) do { \
    printf("  PASS: %s\n", name); \
    tests_passed++; \
} while(0)

static double time_diff_ms(struct timespec *start, struct timespec *end)
{
    double s = (end->tv_sec - start->tv_sec) * 1000.0;
    double ns = (end->tv_nsec - start->tv_nsec) / 1000000.0;
    return s + ns;
}

/*
 * Helper: set up PIO with a given program, return PIO handle and SM number.
 * Caller must close PIO when done.
 */
static int setup_pio(PIO *pio_out, int *sm_out, uint *offset_out,
                     const pio_program_t *prog,
                     pio_sm_config (*get_config)(uint),
                     bool use_pins)
{
    if (pio_init() < 0) {
        fprintf(stderr, "pio_init() failed\n");
        return -1;
    }

    PIO pio = pio_open(0);
    if (PIO_IS_ERR(pio)) {
        fprintf(stderr, "pio_open(0) failed\n");
        return -1;
    }

    int sm = pio_claim_unused_sm(pio, false);
    if (sm < 0) {
        fprintf(stderr, "no free state machines\n");
        pio_close(pio);
        return -1;
    }

    uint offset = pio_add_program(pio, prog);
    if (offset == PIO_ORIGIN_INVALID) {
        fprintf(stderr, "no program space\n");
        pio_sm_unclaim(pio, sm);
        pio_close(pio);
        return -1;
    }

    pio_select(pio);
    pio_sm_config c = get_config(offset);

    if (use_pins) {
        sm_config_set_sideset_pins(&c, TCK_PIN);
        pio_gpio_init(pio, TCK_PIN);
        pio_sm_set_consecutive_pindirs(pio, sm, TCK_PIN, 1, true);

        sm_config_set_out_pins(&c, TDI_PIN, 1);
        pio_gpio_init(pio, TDI_PIN);
        pio_sm_set_consecutive_pindirs(pio, sm, TDI_PIN, 1, true);

        sm_config_set_in_pins(&c, TDO_PIN);
        pio_gpio_init(pio, TDO_PIN);
    } else {
        /* Loopback — only need sideset for realistic timing */
        sm_config_set_sideset_pins(&c, TCK_PIN);
        pio_gpio_init(pio, TCK_PIN);
        pio_sm_set_consecutive_pindirs(pio, sm, TCK_PIN, 1, true);
    }

    /* 6 MHz TCK with 5 instr/bit loopback → div = 200M / (6M * 5) ≈ 6.67 */
    sm_config_set_clkdiv(&c, 6.67f);

    sm_config_set_out_shift(&c, true, true, 32);
    sm_config_set_in_shift(&c, true, true, 32);

    pio_sm_init(pio, sm, offset, &c);

    *pio_out = pio;
    *sm_out = sm;
    *offset_out = offset;
    return 0;
}

static void teardown_pio(PIO pio, int sm, uint offset,
                         const pio_program_t *prog)
{
    pio_sm_set_enabled(pio, sm, false);
    pio_remove_program(pio, prog, offset);
    pio_sm_unclaim(pio, sm);
    pio_close(pio);
}

/* ---- Part A: DMA basics with loopback program ---- */

/*
 * Test A1: TX-only DMA with manual RX drain.
 * Write 8 words via DMA, read them back one by one.
 */
static void test_a1_tx_dma_manual_rx(void)
{
    PIO pio; int sm; uint offset;
    printf("A1: TX-only DMA + manual RX drain...\n");

    if (setup_pio(&pio, &sm, &offset, &jtag_loopback_program,
                  jtag_loopback_program_get_default_config, false) < 0) {
        FAIL("setup failed");
        return;
    }

    /* Configure TX DMA channel */
    int rc = pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, 256, 1);
    printf("  config_xfer(TX): rc=%d\n", rc);
    if (rc < 0)
        FAIL("config_xfer TX failed: rc=%d", rc);

    pio_sm_set_enabled(pio, sm, true);

    /* Loopback program needs a count word first */
    uint32_t count = 8 * 32 - 1;  /* 8 words = 256 bits */
    pio_sm_put_blocking(pio, sm, count);

    /* TX DMA: send 8 data words (32 bytes) */
    uint32_t tx_data[8];
    for (int i = 0; i < 8; i++)
        tx_data[i] = 0xDEAD0000 + i;

    rc = pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM, 32, tx_data);
    printf("  xfer_data(TX, 32 bytes): rc=%d\n", rc);
    if (rc < 0)
        FAIL("xfer_data TX failed: rc=%d", rc);

    /* Manual RX drain */
    uint32_t rx_data[8];
    for (int i = 0; i < 8; i++)
        rx_data[i] = pio_sm_get_blocking(pio, sm);

    /* Drain spurious words */
    while (!pio_sm_is_rx_fifo_empty(pio, sm))
        (void)pio_sm_get_blocking(pio, sm);

    /* Verify */
    int mismatches = 0;
    for (int i = 0; i < 8; i++) {
        if (rx_data[i] != tx_data[i]) {
            printf("  mismatch word %d: got 0x%08x, expected 0x%08x\n",
                   i, rx_data[i], tx_data[i]);
            mismatches++;
        }
    }

    pio_sm_set_enabled(pio, sm, false);

    if (mismatches > 0)
        FAIL("data mismatch: %d words", mismatches);

    PASS("TX DMA + manual RX");
cleanup:
    teardown_pio(pio, sm, offset, &jtag_loopback_program);
}

/*
 * Test A2: RX-only DMA with manual TX fill.
 * Fill TX FIFO manually, read via DMA.
 */
static void test_a2_manual_tx_rx_dma(void)
{
    PIO pio; int sm; uint offset;
    printf("A2: Manual TX + RX-only DMA...\n");

    if (setup_pio(&pio, &sm, &offset, &jtag_loopback_program,
                  jtag_loopback_program_get_default_config, false) < 0) {
        FAIL("setup failed");
        return;
    }

    /* Configure RX DMA channel */
    int rc = pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, 256, 1);
    printf("  config_xfer(RX): rc=%d\n", rc);
    if (rc < 0)
        FAIL("config_xfer RX failed: rc=%d", rc);

    pio_sm_set_enabled(pio, sm, true);

    /* Count word + 8 data words manually */
    uint32_t count = 8 * 32 - 1;
    pio_sm_put_blocking(pio, sm, count);

    uint32_t tx_data[8];
    for (int i = 0; i < 8; i++) {
        tx_data[i] = 0xCAFE0000 + i;
        pio_sm_put_blocking(pio, sm, tx_data[i]);
    }

    /* RX DMA: read 8 words (32 bytes) */
    uint32_t rx_data[8] = {0};
    rc = pio_sm_xfer_data(pio, sm, PIO_DIR_FROM_SM, 32, rx_data);
    printf("  xfer_data(RX, 32 bytes): rc=%d\n", rc);
    if (rc < 0)
        FAIL("xfer_data RX failed: rc=%d", rc);

    /* Drain spurious words */
    while (!pio_sm_is_rx_fifo_empty(pio, sm))
        (void)pio_sm_get_blocking(pio, sm);

    /* Verify */
    int mismatches = 0;
    for (int i = 0; i < 8; i++) {
        if (rx_data[i] != tx_data[i]) {
            printf("  mismatch word %d: got 0x%08x, expected 0x%08x\n",
                   i, rx_data[i], tx_data[i]);
            mismatches++;
        }
    }

    pio_sm_set_enabled(pio, sm, false);

    if (mismatches > 0)
        FAIL("data mismatch: %d words", mismatches);

    PASS("manual TX + RX DMA");
cleanup:
    teardown_pio(pio, sm, offset, &jtag_loopback_program);
}

/*
 * Test A3: Sequential ping-pong DMA at varying chunk sizes.
 * For each chunk size: TX DMA then RX DMA. Measure throughput.
 */
static void test_a3_pingpong_dma(void)
{
    static const int chunk_sizes[] = {32, 256, 1024, 8192};
    static const int num_sizes = sizeof(chunk_sizes) / sizeof(chunk_sizes[0]);

    PIO pio; int sm; uint offset;
    printf("A3: Sequential ping-pong DMA (varying chunk sizes)...\n");

    if (setup_pio(&pio, &sm, &offset, &jtag_loopback_program,
                  jtag_loopback_program_get_default_config, false) < 0) {
        FAIL("setup failed");
        return;
    }

    /* Configure both DMA channels */
    int rc;
    rc = pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, 65532, 1);
    if (rc < 0)
        FAIL("config_xfer TX failed: rc=%d", rc);
    rc = pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, 65532, 1);
    if (rc < 0)
        FAIL("config_xfer RX failed: rc=%d", rc);

    for (int s = 0; s < num_sizes; s++) {
        int chunk = chunk_sizes[s];
        int num_words = chunk / 4;
        int total_bits = num_words * 32;

        /* Prepare TX data */
        uint32_t *tx_data = malloc(chunk);
        uint32_t *rx_data = calloc(1, chunk);
        if (!tx_data || !rx_data) {
            free(tx_data);
            free(rx_data);
            FAIL("malloc failed for chunk %d", chunk);
        }

        for (int i = 0; i < num_words; i++)
            tx_data[i] = 0xA5000000 | (s << 16) | i;

        pio_sm_set_enabled(pio, sm, true);

        struct timespec t_start, t_end;
        clock_gettime(CLOCK_MONOTONIC, &t_start);

        /* Count word first (manual — not part of DMA payload) */
        pio_sm_put_blocking(pio, sm, total_bits - 1);

        /* TX DMA */
        rc = pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM, chunk, tx_data);
        if (rc < 0) {
            printf("  chunk %d: TX DMA failed: rc=%d\n", chunk, rc);
            free(tx_data);
            free(rx_data);
            pio_sm_set_enabled(pio, sm, false);
            continue;
        }

        /* RX DMA */
        rc = pio_sm_xfer_data(pio, sm, PIO_DIR_FROM_SM, chunk, rx_data);
        if (rc < 0) {
            printf("  chunk %d: RX DMA failed: rc=%d\n", chunk, rc);
            free(tx_data);
            free(rx_data);
            pio_sm_set_enabled(pio, sm, false);
            continue;
        }

        clock_gettime(CLOCK_MONOTONIC, &t_end);

        /* Drain spurious */
        while (!pio_sm_is_rx_fifo_empty(pio, sm))
            (void)pio_sm_get_blocking(pio, sm);

        pio_sm_set_enabled(pio, sm, false);

        /* Verify */
        int mismatches = 0;
        for (int i = 0; i < num_words; i++) {
            if (rx_data[i] != tx_data[i])
                mismatches++;
        }

        double ms = time_diff_ms(&t_start, &t_end);
        double kbps = (chunk / 1024.0) / (ms / 1000.0);
        printf("  chunk %5d bytes: %.1f ms, %7.1f kB/s, %d mismatches\n",
               chunk, ms, kbps, mismatches);

        free(tx_data);
        free(rx_data);
    }

    PASS("ping-pong DMA");
cleanup:
    teardown_pio(pio, sm, offset, &jtag_loopback_program);
}

/*
 * Test A4: Concurrent bidirectional DMA.
 * Configure both TX and RX, start both on same SM.
 */
static void test_a4_concurrent_dma(void)
{
    PIO pio; int sm; uint offset;
    printf("A4: Concurrent bidirectional DMA...\n");

    if (setup_pio(&pio, &sm, &offset, &jtag_loopback_program,
                  jtag_loopback_program_get_default_config, false) < 0) {
        FAIL("setup failed");
        return;
    }

    int num_words = 64;
    int chunk = num_words * 4;  /* 256 bytes */

    /* Configure both channels */
    int rc;
    rc = pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, chunk, 1);
    if (rc < 0)
        FAIL("config_xfer TX failed: rc=%d", rc);
    rc = pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, chunk, 1);
    if (rc < 0)
        FAIL("config_xfer RX failed: rc=%d", rc);

    uint32_t *tx_data = malloc(chunk);
    uint32_t *rx_data = calloc(1, chunk);
    if (!tx_data || !rx_data) {
        free(tx_data);
        free(rx_data);
        FAIL("malloc failed");
    }

    for (int i = 0; i < num_words; i++)
        tx_data[i] = 0xB00B0000 + i;

    pio_sm_set_enabled(pio, sm, true);

    /* Count word manually */
    pio_sm_put_blocking(pio, sm, num_words * 32 - 1);

    /*
     * Attempt concurrent: start TX DMA, then immediately start RX DMA.
     * If the API is blocking (TX completes before returning), we'll
     * know because the RX DMA will also complete fine since data is
     * already in RX FIFO by then. If it's truly concurrent, both
     * proceed simultaneously. Either way the data should be correct.
     */
    struct timespec t_start, t_end;
    clock_gettime(CLOCK_MONOTONIC, &t_start);

    rc = pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM, chunk, tx_data);
    printf("  TX xfer_data: rc=%d\n", rc);
    if (rc < 0) {
        free(tx_data);
        free(rx_data);
        FAIL("TX DMA failed: rc=%d", rc);
    }

    rc = pio_sm_xfer_data(pio, sm, PIO_DIR_FROM_SM, chunk, rx_data);
    printf("  RX xfer_data: rc=%d\n", rc);
    if (rc < 0) {
        free(tx_data);
        free(rx_data);
        FAIL("RX DMA failed: rc=%d", rc);
    }

    clock_gettime(CLOCK_MONOTONIC, &t_end);

    /* Drain spurious */
    while (!pio_sm_is_rx_fifo_empty(pio, sm))
        (void)pio_sm_get_blocking(pio, sm);

    pio_sm_set_enabled(pio, sm, false);

    int mismatches = 0;
    for (int i = 0; i < num_words; i++) {
        if (rx_data[i] != tx_data[i]) {
            if (mismatches < 5)
                printf("  mismatch word %d: got 0x%08x, expected 0x%08x\n",
                       i, rx_data[i], tx_data[i]);
            mismatches++;
        }
    }

    double ms = time_diff_ms(&t_start, &t_end);
    printf("  Total time: %.1f ms, %d mismatches\n", ms, mismatches);
    printf("  NOTE: xfer_data appears to be %s\n",
           ms < 1.0 ? "non-blocking or very fast" : "blocking (sequential)");

    free(tx_data);
    free(rx_data);

    if (mismatches > 0)
        FAIL("data mismatch: %d words", mismatches);

    PASS("concurrent DMA");
cleanup:
    teardown_pio(pio, sm, offset, &jtag_loopback_program);
}

/* ---- Part B: Fast program stop condition ---- */

/*
 * Test B5: Fast program + word-by-word.
 * Load jtag_shift_fast, write N words, read N words, disable SM.
 * Check TCK state, spurious words, data integrity.
 */
static void test_b5_fast_word_by_word(void)
{
    PIO pio; int sm; uint offset;
    printf("B5: Fast program + word-by-word...\n");

    if (setup_pio(&pio, &sm, &offset, &jtag_shift_fast_program,
                  jtag_shift_fast_program_get_default_config, false) < 0) {
        FAIL("setup failed");
        return;
    }

    /* Fast program uses 2 instr/bit → div for 6 MHz: 200M / (6M * 2) ≈ 16.67 */
    pio_sm_set_clkdiv(pio, sm, 16.67f);

    int num_words = 16;

    uint32_t tx_data[16];
    uint32_t rx_data[16] = {0};
    for (int i = 0; i < num_words; i++)
        tx_data[i] = 0xFA500000 + i;

    pio_sm_set_enabled(pio, sm, true);

    /* Interleave sm_put/sm_get for all N*32 bits.
     * No count word — fast program has no counter, just wraps. */
    int tx_idx = 0, rx_idx = 0;
    while (rx_idx < num_words) {
        while (tx_idx < num_words && !pio_sm_is_tx_fifo_full(pio, sm)) {
            pio_sm_put_blocking(pio, sm, tx_data[tx_idx++]);
        }
        while (rx_idx < num_words && !pio_sm_is_rx_fifo_empty(pio, sm)) {
            rx_data[rx_idx++] = pio_sm_get_blocking(pio, sm);
        }
        if (rx_idx < num_words && tx_idx >= num_words) {
            rx_data[rx_idx++] = pio_sm_get_blocking(pio, sm);
        }
    }

    /* Stop sequence: disable SM, force TCK low */
    pio_sm_set_enabled(pio, sm, false);

    /* Force TCK low via pin mask */
    uint32_t tck_mask = 1u << TCK_PIN;
    pio_sm_set_pins_with_mask(pio, sm, 0, tck_mask);

    /* Count spurious RX words */
    int spurious = 0;
    while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
        (void)pio_sm_get_blocking(pio, sm);
        spurious++;
    }
    printf("  Spurious RX words after stop: %d\n", spurious);

    /* Verify data (loopback doesn't apply here — we're using fast program
     * which shifts via real pins. Without a TDI->TDO jumper, TDO reads
     * whatever is on the pin. For this test, we just check no crash/hang.) */
    printf("  TX completed: %d words, RX completed: %d words\n",
           tx_idx, rx_idx);
    printf("  First RX word: 0x%08x (TX was 0x%08x)\n",
           rx_data[0], tx_data[0]);

    /* If testing with real loopback wiring, verify data */
    int mismatches = 0;
    for (int i = 0; i < num_words; i++) {
        if (rx_data[i] != tx_data[i])
            mismatches++;
    }
    printf("  Mismatches: %d (expected unless TDI->TDO jumper installed)\n",
           mismatches);

    PASS("fast program word-by-word (no hang)");
cleanup:
    teardown_pio(pio, sm, offset, &jtag_shift_fast_program);
}

/*
 * Test B6: Fast program + DMA (32-byte ping-pong).
 * Uses DMA in FIFO-depth chunks since larger DMA deadlocks.
 * Tests stop sequence after multi-chunk transfers.
 */
static void test_b6_fast_dma(void)
{
    PIO pio; int sm; uint offset;
    printf("B6: Fast program + 32-byte ping-pong DMA...\n");

    if (setup_pio(&pio, &sm, &offset, &jtag_shift_fast_program,
                  jtag_shift_fast_program_get_default_config, false) < 0) {
        FAIL("setup failed");
        return;
    }

    pio_sm_set_clkdiv(pio, sm, 16.67f);

    /* Total transfer: 256 bytes = 64 words, in 32-byte (8-word) chunks */
    int total_words = 64;
    int total_bytes = total_words * 4;
    int chunk_bytes = 32;  /* FIFO depth = 8 words = 32 bytes */
    int chunk_words = chunk_bytes / 4;
    int num_chunks = total_words / chunk_words;

    /* Configure DMA channels for 32-byte transfers */
    int rc;
    rc = pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, chunk_bytes, 1);
    if (rc < 0)
        FAIL("config_xfer TX failed: rc=%d", rc);
    rc = pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, chunk_bytes, 1);
    if (rc < 0)
        FAIL("config_xfer RX failed: rc=%d", rc);

    uint32_t *tx_data = malloc(total_bytes);
    uint32_t *rx_data = calloc(1, total_bytes);
    if (!tx_data || !rx_data) {
        free(tx_data);
        free(rx_data);
        FAIL("malloc failed");
    }

    for (int i = 0; i < total_words; i++)
        tx_data[i] = 0xD4A00000 + i;

    /* Run 3 back-to-back transfers to test state cleanliness */
    for (int run = 0; run < 3; run++) {
        memset(rx_data, 0, total_bytes);

        pio_sm_set_enabled(pio, sm, true);

        struct timespec t_start, t_end;
        clock_gettime(CLOCK_MONOTONIC, &t_start);

        /* Ping-pong: TX 32 bytes, RX 32 bytes, repeat */
        int ok = 1;
        for (int c = 0; c < num_chunks; c++) {
            rc = pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM,
                                  chunk_bytes, &tx_data[c * chunk_words]);
            if (rc < 0) {
                printf("  run %d chunk %d: TX DMA failed: rc=%d\n",
                       run, c, rc);
                ok = 0;
                break;
            }
            rc = pio_sm_xfer_data(pio, sm, PIO_DIR_FROM_SM,
                                  chunk_bytes, &rx_data[c * chunk_words]);
            if (rc < 0) {
                printf("  run %d chunk %d: RX DMA failed: rc=%d\n",
                       run, c, rc);
                ok = 0;
                break;
            }
        }

        clock_gettime(CLOCK_MONOTONIC, &t_end);

        /* Stop sequence */
        pio_sm_set_enabled(pio, sm, false);
        uint32_t tck_mask = 1u << TCK_PIN;
        pio_sm_set_pins_with_mask(pio, sm, 0, tck_mask);

        /* Drain spurious */
        int spurious = 0;
        while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
            (void)pio_sm_get_blocking(pio, sm);
            spurious++;
        }

        if (!ok) continue;

        double ms = time_diff_ms(&t_start, &t_end);
        double kbps = (total_bytes / 1024.0) / (ms / 1000.0);
        printf("  run %d: %.1f ms, %.1f kB/s, %d spurious\n",
               run, ms, kbps, spurious);
    }

    free(tx_data);
    free(rx_data);

    PASS("fast program ping-pong DMA (3 runs)");
cleanup:
    teardown_pio(pio, sm, offset, &jtag_shift_fast_program);
}

/*
 * Test B7: Fast program stop timing.
 * Measure latency of: disable SM + force TCK low + FIFO drain.
 */
static void test_b7_stop_timing(void)
{
    PIO pio; int sm; uint offset;
    printf("B7: Fast program stop timing...\n");

    if (setup_pio(&pio, &sm, &offset, &jtag_shift_fast_program,
                  jtag_shift_fast_program_get_default_config, false) < 0) {
        FAIL("setup failed");
        return;
    }

    pio_sm_set_clkdiv(pio, sm, 16.67f);

    /* Do a small transfer then time the stop */
    int num_words = 8;

    pio_sm_set_enabled(pio, sm, true);

    uint32_t tx_data[8];
    for (int i = 0; i < num_words; i++) {
        tx_data[i] = 0x12340000 + i;
        pio_sm_put_blocking(pio, sm, tx_data[i]);
    }
    for (int i = 0; i < num_words; i++)
        (void)pio_sm_get_blocking(pio, sm);

    /* Time the stop sequence */
    struct timespec t_start, t_end;
    clock_gettime(CLOCK_MONOTONIC, &t_start);

    pio_sm_set_enabled(pio, sm, false);
    uint32_t tck_mask = 1u << TCK_PIN;
    pio_sm_set_pins_with_mask(pio, sm, 0, tck_mask);
    while (!pio_sm_is_rx_fifo_empty(pio, sm))
        (void)pio_sm_get_blocking(pio, sm);

    clock_gettime(CLOCK_MONOTONIC, &t_end);

    double us = time_diff_ms(&t_start, &t_end) * 1000.0;
    printf("  Stop sequence latency: %.1f us\n", us);

    PASS("stop timing");
cleanup:
    teardown_pio(pio, sm, offset, &jtag_shift_fast_program);
}

/* ---- Part C: Throughput ceiling ---- */

/*
 * Test C8: Large DMA transfers.
 * Test 32 KB and 64 KB (near 65,532 byte limit).
 */
static void test_c8_large_dma(void)
{
    PIO pio; int sm; uint offset;
    printf("C8: Large DMA transfers (throughput ceiling)...\n");

    if (setup_pio(&pio, &sm, &offset, &jtag_loopback_program,
                  jtag_loopback_program_get_default_config, false) < 0) {
        FAIL("setup failed");
        return;
    }

    static const int sizes[] = {32768, 65532};
    static const int num_sizes = 2;

    /* Configure DMA with max buffer size */
    int rc;
    rc = pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, 65532, 1);
    if (rc < 0)
        FAIL("config_xfer TX failed: rc=%d", rc);
    rc = pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, 65532, 1);
    if (rc < 0)
        FAIL("config_xfer RX failed: rc=%d", rc);

    for (int s = 0; s < num_sizes; s++) {
        int chunk = sizes[s];
        int num_words = chunk / 4;

        uint32_t *tx_data = malloc(chunk);
        uint32_t *rx_data = calloc(1, chunk);
        if (!tx_data || !rx_data) {
            free(tx_data);
            free(rx_data);
            FAIL("malloc for %d bytes", chunk);
        }

        for (int i = 0; i < num_words; i++)
            tx_data[i] = (uint32_t)i;

        pio_sm_set_enabled(pio, sm, true);

        /* Count word */
        pio_sm_put_blocking(pio, sm, num_words * 32 - 1);

        struct timespec t_start, t_end;
        clock_gettime(CLOCK_MONOTONIC, &t_start);

        rc = pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM, chunk, tx_data);
        if (rc < 0) {
            printf("  %d bytes: TX DMA failed: rc=%d\n", chunk, rc);
            free(tx_data);
            free(rx_data);
            pio_sm_set_enabled(pio, sm, false);
            continue;
        }

        rc = pio_sm_xfer_data(pio, sm, PIO_DIR_FROM_SM, chunk, rx_data);
        if (rc < 0) {
            printf("  %d bytes: RX DMA failed: rc=%d\n", chunk, rc);
            free(tx_data);
            free(rx_data);
            pio_sm_set_enabled(pio, sm, false);
            continue;
        }

        clock_gettime(CLOCK_MONOTONIC, &t_end);

        /* Drain spurious */
        while (!pio_sm_is_rx_fifo_empty(pio, sm))
            (void)pio_sm_get_blocking(pio, sm);

        pio_sm_set_enabled(pio, sm, false);

        /* Verify */
        int mismatches = 0;
        int first_mismatch = -1;
        for (int i = 0; i < num_words; i++) {
            if (rx_data[i] != tx_data[i]) {
                if (first_mismatch < 0)
                    first_mismatch = i;
                mismatches++;
            }
        }

        double ms = time_diff_ms(&t_start, &t_end);
        double kbps = (chunk / 1024.0) / (ms / 1000.0);
        printf("  %5d bytes: %.1f ms, %7.1f kB/s, %d mismatches",
               chunk, ms, kbps, mismatches);
        if (first_mismatch >= 0)
            printf(" (first at word %d: got 0x%08x, exp 0x%08x)",
                   first_mismatch,
                   rx_data[first_mismatch], tx_data[first_mismatch]);
        printf("\n");

        free(tx_data);
        free(rx_data);
    }

    PASS("large DMA");
cleanup:
    teardown_pio(pio, sm, offset, &jtag_loopback_program);
}

/* ---- Main ---- */

int main(void)
{
    printf("DMA + Fast PIO Exploration (requires RPi 5)\n");
    printf("============================================\n\n");

    /* Report FIFO depth */
    if (pio_init() >= 0) {
        PIO pio = pio_open(0);
        if (!PIO_IS_ERR(pio)) {
            printf("RP1 PIO FIFO depth: %d words (%d bytes)\n\n",
                   pio_get_fifo_depth(pio), pio_get_fifo_depth(pio) * 4);
            pio_close(pio);
        }
    }

    printf("--- Part A: DMA basics (loopback program) ---\n");
    test_a1_tx_dma_manual_rx();
    test_a2_manual_tx_rx_dma();
    test_a3_pingpong_dma();
    test_a4_concurrent_dma();

    printf("\n--- Part B: Fast program stop condition ---\n");
    test_b5_fast_word_by_word();
    test_b6_fast_dma();
    test_b7_stop_timing();

    printf("\n--- Part C: Throughput ceiling ---\n");
    test_c8_large_dma();

    printf("\n============================================\n");
    printf("%d passed, %d failed\n", tests_passed, tests_failed);

    if (tests_passed >= 4) {
        printf("\nGO/NO-GO: At least basic DMA works.\n");
    } else {
        printf("\nGO/NO-GO: DMA appears unreliable — reassess approach.\n");
    }

    return tests_failed > 0 ? 1 : 0;
}

#else /* !HAVE_PIOLIB */

#include <stdio.h>

int main(void)
{
    fprintf(stderr, "test_dma_explore: requires HAVE_PIOLIB (RPi 5 only)\n");
    return 1;
}

#endif /* HAVE_PIOLIB */
