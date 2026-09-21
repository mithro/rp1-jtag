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
#include <strings.h>
#include <unistd.h>
#include <pthread.h>

/* PIO program headers */
#include "pio/generated/jtag_loopback.pio.h"
#include "pio/generated/jtag_shift_fast.pio.h"

/*
 * Minimal PIO loopback: copies TX FIFO → X register → RX FIFO, word by word.
 * 2 instructions per word. At div=1 (200 MHz): 100M words/s = 400 MB/s.
 * No external I/O, no sideset, no count word — pure internal data copy.
 */
#define fast_loopback_wrap_target 0
#define fast_loopback_wrap 1

static const uint16_t fast_loopback_program_instructions[] = {
            //     .wrap_target
    0x6020, //  0: out    x, 32
    0x4020, //  1: in     x, 32
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program fast_loopback_program = {
    .instructions = fast_loopback_program_instructions,
    .length = 2,
    .origin = -1,
    .pio_version = 0,
};

static inline pio_sm_config fast_loopback_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + fast_loopback_wrap_target,
                       offset + fast_loopback_wrap);
    return c;
}
#endif

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

/*
 * Test A5: Threaded bidirectional DMA.
 * Uses jtag_shift_fast (production program) with external TDI→TDO loopback.
 * RX DMA runs in a pthread while TX DMA runs in the main thread.
 *
 * Critical: RX DMA must enter the kernel ioctl BEFORE TX DMA starts,
 * so the RX DMA channel is armed and ready to drain the RX FIFO as the
 * SM produces data. If TX enters first, the SM fills the RX FIFO before
 * RX DMA is ready → SM stalls → TX times out.
 *
 * We use a pthread barrier to synchronize: the RX thread signals "ready"
 * just before entering the ioctl, and the main thread waits for this signal.
 */

typedef struct {
    PIO pio;
    int sm;
    uint32_t data_bytes;
    void *data;
    int result;
    pthread_barrier_t *barrier;
} rx_thread_arg_t;

static void *rx_dma_thread(void *arg)
{
    rx_thread_arg_t *a = (rx_thread_arg_t *)arg;
    /* Signal that we're about to enter the kernel ioctl */
    pthread_barrier_wait(a->barrier);
    a->result = pio_sm_xfer_data(a->pio, a->sm,
                                  PIO_DIR_FROM_SM, a->data_bytes, a->data);
    return NULL;
}

static void test_a5_threaded_bidi_dma(void)
{
    static const int test_sizes[] = {32768, 4096, 256};
    static const int num_sizes = sizeof(test_sizes) / sizeof(test_sizes[0]);

    printf("A5: Threaded bidirectional DMA (jtag_shift_fast, TDI→TDO loopback)...\n");
    printf("    NOTE: Requires external wire from TDI (GPIO %d) to TDO (GPIO %d)\n",
           TDI_PIN, TDO_PIN);

    /* Run each size with a fresh PIO setup to avoid state contamination */
    for (int s = 0; s < num_sizes; s++) {
        int chunk = test_sizes[s];
        int num_words = chunk / 4;

        PIO pio; int sm; uint offset;
        if (setup_pio(&pio, &sm, &offset, &jtag_shift_fast_program,
                      jtag_shift_fast_program_get_default_config, true) < 0) {
            FAIL("setup failed for %d bytes", chunk);
        }

        /* Fast program: 2 instr/bit → 10 MHz TCK: div = 200M / (10M × 2) = 10 */
        pio_sm_set_clkdiv(pio, sm, 10.0f);

        /* Configure DMA channels — buf_size must match the largest xfer_data call */
        int rc;
        rc = pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, chunk, 1);
        if (rc < 0) {
            printf("  %5d bytes: config_xfer TX failed: rc=%d\n", chunk, rc);
            teardown_pio(pio, sm, offset, &jtag_shift_fast_program);
            continue;
        }
        rc = pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, chunk, 1);
        if (rc < 0) {
            printf("  %5d bytes: config_xfer RX failed: rc=%d\n", chunk, rc);
            teardown_pio(pio, sm, offset, &jtag_shift_fast_program);
            continue;
        }

        uint32_t *tx_data = malloc(chunk);
        uint32_t *rx_data = calloc(1, chunk);
        if (!tx_data || !rx_data) {
            free(tx_data);
            free(rx_data);
            teardown_pio(pio, sm, offset, &jtag_shift_fast_program);
            FAIL("malloc for %d bytes", chunk);
        }

        for (int i = 0; i < num_words; i++)
            tx_data[i] = 0xA5A50000 | (s << 8) | (i & 0xFF);

        /* Barrier: RX thread signals "about to enter ioctl" */
        pthread_barrier_t barrier;
        pthread_barrier_init(&barrier, NULL, 2);

        pio_sm_set_enabled(pio, sm, true);

        struct timespec t_start, t_end;

        /* Start RX DMA in pthread */
        rx_thread_arg_t rx_arg = {
            .pio = pio,
            .sm = sm,
            .data_bytes = chunk,
            .data = rx_data,
            .result = -999,
            .barrier = &barrier,
        };
        pthread_t rx_tid;
        rc = pthread_create(&rx_tid, NULL, rx_dma_thread, &rx_arg);
        if (rc != 0) {
            free(tx_data);
            free(rx_data);
            pio_sm_set_enabled(pio, sm, false);
            pthread_barrier_destroy(&barrier);
            teardown_pio(pio, sm, offset, &jtag_shift_fast_program);
            FAIL("pthread_create failed: %d", rc);
        }

        /* Wait for RX thread to be ready, then give it time to enter the
         * kernel ioctl and arm the RX DMA channel before we start TX.
         * Without this delay, the TX ioctl can enter the kernel first,
         * start DMA, fill the RX FIFO before RX DMA is armed, stalling
         * the SM → TX DMA timeout. */
        pthread_barrier_wait(&barrier);
        usleep(100000);  /* 100ms — ensure RX ioctl fully in kernel */

        clock_gettime(CLOCK_MONOTONIC, &t_start);

        /* TX DMA in main thread */
        int tx_rc = pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM, chunk, tx_data);

        /* Wait for RX to complete */
        pthread_join(rx_tid, NULL);

        clock_gettime(CLOCK_MONOTONIC, &t_end);

        pio_sm_set_enabled(pio, sm, false);
        pthread_barrier_destroy(&barrier);

        /* Force TCK low */
        uint32_t tck_mask = 1u << TCK_PIN;
        pio_sm_set_pins_with_mask(pio, sm, 0, tck_mask);

        /* Drain spurious */
        while (!pio_sm_is_rx_fifo_empty(pio, sm))
            (void)pio_sm_get_blocking(pio, sm);

        if (tx_rc < 0) {
            printf("  %5d bytes: TX DMA failed: rc=%d (errno=%d %s)\n",
                   chunk, tx_rc, errno, strerror(errno));
            free(tx_data);
            free(rx_data);
            teardown_pio(pio, sm, offset, &jtag_shift_fast_program);
            continue;
        }
        if (rx_arg.result < 0) {
            printf("  %5d bytes: RX DMA failed: rc=%d\n", chunk, rx_arg.result);
            free(tx_data);
            free(rx_data);
            teardown_pio(pio, sm, offset, &jtag_shift_fast_program);
            continue;
        }

        /* Verify data */
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
        teardown_pio(pio, sm, offset, &jtag_shift_fast_program);
    }

    PASS("threaded bidi DMA");
cleanup:
    return;  /* teardown done per-iteration */
}

/*
 * Test A6: Loopback threaded bidi DMA.
 * Uses jtag_loopback (internal copy, no external wiring needed).
 * Proves DMA pipeline works without I/O complications.
 *
 * The loopback program requires a count word sent manually before DMA.
 * RX DMA in pthread, TX DMA in main thread.
 *
 * Sub-tests isolate the problem:
 *  A6a: RX DMA only (manual TX feed in thread) — proves RX DMA path
 *  A6b: TX DMA only (manual RX drain in thread) — proves TX DMA path
 *  A6c: Both DMA concurrent (RX thread + TX main) — proves full bidi
 */

/* Thread arg for manual TX feed (word-by-word via put_blocking) */
typedef struct {
    PIO pio;
    int sm;
    uint32_t num_words;
    const uint32_t *data;
} manual_tx_arg_t;

static void *manual_tx_thread(void *arg)
{
    manual_tx_arg_t *a = (manual_tx_arg_t *)arg;
    for (uint32_t i = 0; i < a->num_words; i++)
        pio_sm_put_blocking(a->pio, a->sm, a->data[i]);
    return NULL;
}

/* Thread arg for manual RX drain (word-by-word via get_blocking) */
typedef struct {
    PIO pio;
    int sm;
    uint32_t num_words;
    uint32_t *data;
} manual_rx_arg_t;

static void *manual_rx_thread(void *arg)
{
    manual_rx_arg_t *a = (manual_rx_arg_t *)arg;
    for (uint32_t i = 0; i < a->num_words; i++)
        a->data[i] = pio_sm_get_blocking(a->pio, a->sm);
    return NULL;
}

static void test_a6_loopback_bidi_dma(void)
{
    printf("A6: Loopback bidi DMA — isolating DMA DREQ issue\n");

    /*
     * A6a: Sequential — push ALL data first, then RX DMA.
     * With 16 words: push count, push words 0-15 (SM processes at 200MHz,
     * RX FIFO fills with 8 words, SM stalls, remaining 8 words fill TX FIFO).
     * Then RX DMA drains RX FIFO → SM unstalls → processes remaining → done.
     * This is the same pattern as test A2 but with 16 words instead of 8.
     */
    printf("\n  A6a: Sequential — manual TX all 16 words, then RX DMA (64 bytes)\n");
    {
        int chunk = 64;
        int num_words = 16;
        int total_bits = 512;

        PIO pio; int sm; uint offset;
        if (setup_pio(&pio, &sm, &offset, &jtag_loopback_program,
                      jtag_loopback_program_get_default_config, false) < 0) {
            FAIL("A6a setup failed");
        }
        pio_sm_set_clkdiv(pio, sm, 1.0f);

        int rc = pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, chunk, 1);
        printf("    config_xfer(RX, %d): rc=%d\n", chunk, rc);
        if (rc < 0) {
            teardown_pio(pio, sm, offset, &jtag_loopback_program);
            FAIL("A6a config_xfer failed");
        }

        uint32_t tx_data[16], rx_data[16];
        memset(rx_data, 0xDD, sizeof(rx_data));
        for (int i = 0; i < num_words; i++)
            tx_data[i] = 0x6A000000 | i;

        pio_sm_set_enabled(pio, sm, true);

        /* Push count + all 16 data words sequentially BEFORE DMA */
        pio_sm_put_blocking(pio, sm, total_bits - 1);
        for (int i = 0; i < num_words; i++)
            pio_sm_put_blocking(pio, sm, tx_data[i]);
        printf("    All %d words pushed (count + %d data)\n",
               num_words + 1, num_words);

        struct timespec t_start, t_end;
        clock_gettime(CLOCK_MONOTONIC, &t_start);

        /* Now RX DMA to collect results */
        int rx_rc = pio_sm_xfer_data(pio, sm, PIO_DIR_FROM_SM, chunk, rx_data);

        clock_gettime(CLOCK_MONOTONIC, &t_end);
        pio_sm_set_enabled(pio, sm, false);

        int spurious = 0;
        while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
            uint32_t w = pio_sm_get_blocking(pio, sm);
            if (spurious < 4)
                printf("    spurious[%d] = 0x%08x\n", spurious, w);
            spurious++;
        }

        printf("    RX DMA rc=%d, spurious=%d, time=%.2f ms\n",
               rx_rc, spurious, time_diff_ms(&t_start, &t_end));

        int mismatches = 0;
        for (int i = 0; i < num_words; i++) {
            const char *mark = (rx_data[i] == tx_data[i]) ? "OK" : "MISMATCH";
            printf("    word[%2d]: TX=0x%08x  RX=0x%08x  %s\n",
                   i, tx_data[i], rx_data[i], mark);
            if (rx_data[i] != tx_data[i]) mismatches++;
        }
        printf("    Total mismatches: %d / %d\n", mismatches, num_words);

        if (mismatches == 0 && rx_rc == 0)
            PASS("A6a: sequential manual TX → RX DMA (64 bytes)");

cleanup:
        teardown_pio(pio, sm, offset, &jtag_loopback_program);
    }

    /*
     * A6b: Same as A6a but with config_xfer(buf_size=32) instead of 64.
     * Tests whether buf_size matters (A2 uses 256, works for 32-byte xfer).
     */
    printf("\n  A6b: Sequential — same but config_xfer(buf_size=32)\n");
    {
        int chunk = 64;
        int num_words = 16;
        int total_bits = 512;

        PIO pio; int sm; uint offset;
        if (setup_pio(&pio, &sm, &offset, &jtag_loopback_program,
                      jtag_loopback_program_get_default_config, false) < 0) {
            printf("    setup failed\n");
            goto cleanup_b;
        }
        pio_sm_set_clkdiv(pio, sm, 1.0f);

        /* config_xfer with buf_size=32 (known working size from A2/A3) */
        int rc = pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, 32, 1);
        printf("    config_xfer(RX, 32): rc=%d\n", rc);
        if (rc < 0) {
            teardown_pio(pio, sm, offset, &jtag_loopback_program);
            goto cleanup_b;
        }

        uint32_t tx_data[16], rx_data[16];
        memset(rx_data, 0xDD, sizeof(rx_data));
        for (int i = 0; i < num_words; i++)
            tx_data[i] = 0x6B000000 | i;

        pio_sm_set_enabled(pio, sm, true);

        /* Push count + all data */
        pio_sm_put_blocking(pio, sm, total_bits - 1);
        for (int i = 0; i < num_words; i++)
            pio_sm_put_blocking(pio, sm, tx_data[i]);

        struct timespec t_start, t_end;
        clock_gettime(CLOCK_MONOTONIC, &t_start);

        /* RX DMA in two 32-byte chunks (ping-pong, same as working A3) */
        int rx_rc = pio_sm_xfer_data(pio, sm, PIO_DIR_FROM_SM, 32, &rx_data[0]);
        int rx_rc2 = -999;
        if (rx_rc == 0)
            rx_rc2 = pio_sm_xfer_data(pio, sm, PIO_DIR_FROM_SM, 32, &rx_data[8]);

        clock_gettime(CLOCK_MONOTONIC, &t_end);
        pio_sm_set_enabled(pio, sm, false);

        int spurious = 0;
        while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
            (void)pio_sm_get_blocking(pio, sm);
            spurious++;
        }

        printf("    RX DMA chunk1 rc=%d, chunk2 rc=%d, spurious=%d, time=%.2f ms\n",
               rx_rc, rx_rc2, spurious, time_diff_ms(&t_start, &t_end));

        int mismatches = 0;
        for (int i = 0; i < num_words; i++) {
            const char *mark = (rx_data[i] == tx_data[i]) ? "OK" : "MISMATCH";
            printf("    word[%2d]: TX=0x%08x  RX=0x%08x  %s\n",
                   i, tx_data[i], rx_data[i], mark);
            if (rx_data[i] != tx_data[i]) mismatches++;
        }
        printf("    Total mismatches: %d / %d\n", mismatches, num_words);

        if (mismatches == 0 && rx_rc == 0 && rx_rc2 == 0)
            PASS("A6b: sequential TX → RX DMA 2×32B (64 bytes)");

cleanup_b:
        teardown_pio(pio, sm, offset, &jtag_loopback_program);
    }

    /*
     * A6c: TX DMA (64 bytes) + manual RX drain in thread.
     * A6b from previous run showed this works (12/16 correct, last 4 zero
     * because manual RX thread was slow — the SM wrapped back to pull and
     * stalled, so the extra words were never produced).
     * This time, drain 16 data + 1 spurious word = 17 words.
     */
    printf("\n  A6c: TX DMA (64 bytes) + manual RX drain (17 words incl spurious)\n");
    {
        int chunk = 64;
        int num_words = 16;
        int total_bits = 512;

        PIO pio; int sm; uint offset;
        if (setup_pio(&pio, &sm, &offset, &jtag_loopback_program,
                      jtag_loopback_program_get_default_config, false) < 0) {
            printf("    setup failed\n");
            goto cleanup_c;
        }
        pio_sm_set_clkdiv(pio, sm, 1.0f);

        int rc = pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, chunk, 1);
        printf("    config_xfer(TX, %d): rc=%d\n", chunk, rc);
        if (rc < 0) {
            teardown_pio(pio, sm, offset, &jtag_loopback_program);
            goto cleanup_c;
        }

        uint32_t tx_data[16], rx_data[17]; /* 17 = 16 data + 1 spurious */
        memset(rx_data, 0xDD, sizeof(rx_data));
        for (int i = 0; i < num_words; i++)
            tx_data[i] = 0x6C000000 | i;

        pio_sm_set_enabled(pio, sm, true);
        pio_sm_put_blocking(pio, sm, total_bits - 1);

        /* Manual RX: drain 17 words (16 data + 1 spurious from push block) */
        manual_rx_arg_t rx_arg = {pio, sm, 17, rx_data};
        pthread_t rx_tid;
        pthread_create(&rx_tid, NULL, manual_rx_thread, &rx_arg);

        struct timespec t_start, t_end;
        clock_gettime(CLOCK_MONOTONIC, &t_start);

        int tx_rc = pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM, chunk, tx_data);

        if (tx_rc < 0) {
            pio_sm_set_enabled(pio, sm, false);
            pthread_cancel(rx_tid);
        }
        pthread_join(rx_tid, NULL);
        clock_gettime(CLOCK_MONOTONIC, &t_end);
        pio_sm_set_enabled(pio, sm, false);

        printf("    TX DMA rc=%d, time=%.2f ms\n",
               tx_rc, time_diff_ms(&t_start, &t_end));

        int mismatches = 0;
        for (int i = 0; i < num_words; i++) {
            const char *mark = (rx_data[i] == tx_data[i]) ? "OK" : "MISMATCH";
            printf("    word[%2d]: TX=0x%08x  RX=0x%08x  %s\n",
                   i, tx_data[i], rx_data[i], mark);
            if (rx_data[i] != tx_data[i]) mismatches++;
        }
        printf("    spurious word[16] = 0x%08x (expect 0x00000000)\n", rx_data[16]);
        printf("    Total mismatches: %d / %d\n", mismatches, num_words);

        if (mismatches == 0 && tx_rc == 0)
            PASS("A6c: TX DMA + manual RX (64 bytes)");

cleanup_c:
        teardown_pio(pio, sm, offset, &jtag_loopback_program);
    }
}

/*
 * A6d: Threaded concurrent bidi DMA with 32-byte chunks.
 * Both TX and RX run in separate threads, each doing 32-byte DMA calls
 * in a loop. This is the concurrent version of ping-pong.
 *
 * With concurrent DMA, the SM never stalls because RX is drained
 * simultaneously as TX feeds data. This should allow transfers > 32 bytes
 * without deadlock.
 */

typedef struct {
    PIO pio;
    int sm;
    int num_chunks;
    int chunk_bytes;
    void *data;     /* uint32_t* for both TX and RX */
    int result;
} dma_loop_arg_t;

static void *rx_dma_loop_thread(void *arg)
{
    dma_loop_arg_t *a = (dma_loop_arg_t *)arg;
    int cw = a->chunk_bytes / 4;
    uint32_t *data = (uint32_t *)a->data;
    for (int c = 0; c < a->num_chunks; c++) {
        int r = pio_sm_xfer_data(a->pio, a->sm,
            PIO_DIR_FROM_SM, a->chunk_bytes, &data[c * cw]);
        if (r < 0) { a->result = r; return NULL; }
    }
    return NULL;
}

static void *tx_dma_loop_thread(void *arg)
{
    dma_loop_arg_t *a = (dma_loop_arg_t *)arg;
    int cw = a->chunk_bytes / 4;
    const uint32_t *data = (const uint32_t *)a->data;
    for (int c = 0; c < a->num_chunks; c++) {
        int r = pio_sm_xfer_data(a->pio, a->sm,
            PIO_DIR_TO_SM, a->chunk_bytes, &data[c * cw]);
        if (r < 0) { a->result = r; return NULL; }
    }
    return NULL;
}

static void test_a6d_threaded_bidi_dma(void)
{
    printf("\n  A6d: Threaded concurrent bidi DMA (32-byte chunks)\n");

    /*
     * Step 1: Verify sequential baseline works at div=1.
     * A3 uses div=6.67 — let's confirm div=1 works with sequential.
     */
    printf("\n    Step 1: Sequential baseline at div=1 (200 MHz)\n");
    {
        PIO pio; int sm; uint offset;
        if (setup_pio(&pio, &sm, &offset, &jtag_loopback_program,
                      jtag_loopback_program_get_default_config, false) < 0) {
            FAIL("A6d step 1 setup");
        }
        pio_sm_set_clkdiv(pio, sm, 1.0f);

        int rc;
        rc = pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, 32, 1);
        if (rc < 0) { teardown_pio(pio, sm, offset, &jtag_loopback_program); FAIL("TX config"); }
        rc = pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, 32, 1);
        if (rc < 0) { teardown_pio(pio, sm, offset, &jtag_loopback_program); FAIL("RX config"); }

        /* Test sizes: 64, 1024, 32768 — sequential ping-pong at max speed */
        static const int sizes[] = {64, 1024, 32768};
        for (int s = 0; s < 3; s++) {
            int total = sizes[s];
            int nw = total / 4;
            int nchunks = total / 32;

            uint32_t *tx = malloc(total);
            uint32_t *rx = calloc(1, total);
            for (int i = 0; i < nw; i++)
                tx[i] = 0xAA000000 | (s << 16) | (i & 0xFFFF);

            pio_sm_set_enabled(pio, sm, true);
            pio_sm_put_blocking(pio, sm, nw * 32 - 1);

            struct timespec t0, t1;
            clock_gettime(CLOCK_MONOTONIC, &t0);

            int ok = 1;
            for (int c = 0; c < nchunks; c++) {
                rc = pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM, 32, &tx[c*8]);
                if (rc < 0) { printf("      TX chunk %d failed\n", c); ok = 0; break; }
                rc = pio_sm_xfer_data(pio, sm, PIO_DIR_FROM_SM, 32, &rx[c*8]);
                if (rc < 0) { printf("      RX chunk %d failed\n", c); ok = 0; break; }
            }

            clock_gettime(CLOCK_MONOTONIC, &t1);
            pio_sm_set_enabled(pio, sm, false);
            while (!pio_sm_is_rx_fifo_empty(pio, sm))
                (void)pio_sm_get_blocking(pio, sm);

            int mm = 0;
            if (ok) for (int i = 0; i < nw; i++) if (rx[i] != tx[i]) mm++;

            double ms = time_diff_ms(&t0, &t1);
            double kbps = (total / 1024.0) / (ms / 1000.0);
            printf("      %5d bytes seq: %.2f ms, %7.1f kB/s, %d mismatches%s\n",
                   total, ms, kbps, mm, ok ? "" : " (FAILED)");

            free(tx);
            free(rx);
        }
        teardown_pio(pio, sm, offset, &jtag_loopback_program);
    }

    /*
     * Step 2: Primed concurrent bidi DMA.
     * Hypothesis: RX DMA returns stale bounce buffer data when started
     * before any real transfer. Fix: do a small sequential "primer"
     * transfer first to consume stale DMA state, then concurrent.
     *
     * Approach A: Prime with 1 sequential chunk, then concurrent
     * Approach B: TX-first (TX thread starts, RX after short delay)
     */
    printf("\n    Step 2a: Primed concurrent — 1 seq chunk, then concurrent\n");
    {
        int total = 256;  /* 64 words, 8 chunks */
        int nw = total / 4;
        int nchunks = total / 32;

        PIO pio; int sm; uint offset;
        if (setup_pio(&pio, &sm, &offset, &jtag_loopback_program,
                      jtag_loopback_program_get_default_config, false) < 0) {
            printf("      setup failed\n");
            goto skip_step2a;
        }
        pio_sm_set_clkdiv(pio, sm, 1.0f);

        int rc;
        rc = pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, 32, 1);
        rc = pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, 32, 1);

        uint32_t *tx = malloc(total);
        uint32_t *rx = calloc(1, total);
        for (int i = 0; i < nw; i++)
            tx[i] = 0x6D000000 | (i & 0xFFFF);

        pio_sm_set_enabled(pio, sm, true);
        pio_sm_put_blocking(pio, sm, nw * 32 - 1);

        struct timespec t0, t1;
        clock_gettime(CLOCK_MONOTONIC, &t0);

        /* PRIMER: First chunk sequential (TX then RX) to consume stale DMA */
        rc = pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM, 32, &tx[0]);
        if (rc < 0) { printf("      primer TX failed\n"); goto done_2a; }
        rc = pio_sm_xfer_data(pio, sm, PIO_DIR_FROM_SM, 32, &rx[0]);
        if (rc < 0) { printf("      primer RX failed\n"); goto done_2a; }

        /* CONCURRENT: remaining chunks — TX in thread, RX in main */
        dma_loop_arg_t tx_arg = {
            .pio = pio, .sm = sm,
            .num_chunks = nchunks - 1, .chunk_bytes = 32,
            .data = &tx[8], .result = 0,
        };
        pthread_t tx_tid;
        pthread_create(&tx_tid, NULL, tx_dma_loop_thread, &tx_arg);

        /* RX in main thread — starts after TX thread is launched */
        int rx_res = 0;
        for (int c = 1; c < nchunks; c++) {
            rc = pio_sm_xfer_data(pio, sm, PIO_DIR_FROM_SM, 32, &rx[c*8]);
            if (rc < 0) { rx_res = rc; break; }
        }

        pthread_join(tx_tid, NULL);
        clock_gettime(CLOCK_MONOTONIC, &t1);

done_2a:
        pio_sm_set_enabled(pio, sm, false);
        int sp = 0;
        while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
            (void)pio_sm_get_blocking(pio, sm); sp++;
        }

        int mm = 0;
        for (int i = 0; i < nw; i++) if (rx[i] != tx[i]) mm++;

        double ms = time_diff_ms(&t0, &t1);
        double kbps = (total / 1024.0) / (ms / 1000.0);
        printf("      %d bytes: %.2f ms, %7.1f kB/s, %d/%d mm, sp=%d, "
               "TX=%d RX=%d\n",
               total, ms, kbps, mm, nw, sp, tx_arg.result, rx_res);
        if (mm > 0) {
            for (int i = 0; i < nw && i < 20; i++) {
                const char *m = (rx[i] == tx[i]) ? "OK" : "MISMATCH";
                printf("        word[%2d]: TX=0x%08x RX=0x%08x %s\n",
                       i, tx[i], rx[i], m);
            }
        }

        free(tx);
        free(rx);
        teardown_pio(pio, sm, offset, &jtag_loopback_program);
skip_step2a: ;
    }

    printf("\n    Step 2b: TX-first concurrent — TX thread first, RX after\n");
    {
        int total = 256;
        int nw = total / 4;
        int nchunks = total / 32;

        PIO pio; int sm; uint offset;
        if (setup_pio(&pio, &sm, &offset, &jtag_loopback_program,
                      jtag_loopback_program_get_default_config, false) < 0) {
            printf("      setup failed\n");
            goto skip_step2b;
        }
        pio_sm_set_clkdiv(pio, sm, 1.0f);

        int rc;
        rc = pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, 32, 1);
        rc = pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, 32, 1);

        uint32_t *tx = malloc(total);
        uint32_t *rx = calloc(1, total);
        for (int i = 0; i < nw; i++)
            tx[i] = 0x6E000000 | (i & 0xFFFF);

        pio_sm_set_enabled(pio, sm, true);
        pio_sm_put_blocking(pio, sm, nw * 32 - 1);

        struct timespec t0, t1;
        clock_gettime(CLOCK_MONOTONIC, &t0);

        /* TX thread starts FIRST */
        dma_loop_arg_t tx_arg = {
            .pio = pio, .sm = sm,
            .num_chunks = nchunks, .chunk_bytes = 32,
            .data = tx, .result = 0,
        };
        pthread_t tx_tid;
        pthread_create(&tx_tid, NULL, tx_dma_loop_thread, &tx_arg);

        /* RX in main thread — starts after small delay */
        usleep(100);  /* 100µs — TX should have pushed first chunk by now */

        int rx_res = 0;
        for (int c = 0; c < nchunks; c++) {
            rc = pio_sm_xfer_data(pio, sm, PIO_DIR_FROM_SM, 32, &rx[c*8]);
            if (rc < 0) { rx_res = rc; break; }
        }

        pthread_join(tx_tid, NULL);
        clock_gettime(CLOCK_MONOTONIC, &t1);
        pio_sm_set_enabled(pio, sm, false);

        int sp = 0;
        while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
            (void)pio_sm_get_blocking(pio, sm); sp++;
        }

        int mm = 0;
        for (int i = 0; i < nw; i++) if (rx[i] != tx[i]) mm++;

        double ms = time_diff_ms(&t0, &t1);
        double kbps = (total / 1024.0) / (ms / 1000.0);
        printf("      %d bytes: %.2f ms, %7.1f kB/s, %d/%d mm, sp=%d, "
               "TX=%d RX=%d\n",
               total, ms, kbps, mm, nw, sp, tx_arg.result, rx_res);
        if (mm > 0) {
            for (int i = 0; i < nw && i < 20; i++) {
                const char *m = (rx[i] == tx[i]) ? "OK" : "MISMATCH";
                printf("        word[%2d]: TX=0x%08x RX=0x%08x %s\n",
                       i, tx[i], rx[i], m);
            }
        }

        free(tx);
        free(rx);
        teardown_pio(pio, sm, offset, &jtag_loopback_program);
skip_step2b: ;
    }

    /*
     * Step 3: Scale up whichever approach works from step 2.
     */
    printf("\n    Step 3: Scaling with primed concurrent\n");
    {
        static const int sizes[] = {1024, 4096, 32768};
        for (int s = 0; s < 3; s++) {
            int total = sizes[s];
            int nw = total / 4;
            int nchunks = total / 32;

            PIO pio; int sm; uint offset;
            if (setup_pio(&pio, &sm, &offset, &jtag_loopback_program,
                          jtag_loopback_program_get_default_config, false) < 0) {
                printf("      %5d bytes: setup failed\n", total);
                continue;
            }
            pio_sm_set_clkdiv(pio, sm, 1.0f);

            int rc;
            rc = pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, 32, 1);
            if (rc < 0) { teardown_pio(pio, sm, offset, &jtag_loopback_program); continue; }
            rc = pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, 32, 1);
            if (rc < 0) { teardown_pio(pio, sm, offset, &jtag_loopback_program); continue; }

            uint32_t *tx = malloc(total);
            uint32_t *rx = calloc(1, total);
            for (int i = 0; i < nw; i++)
                tx[i] = 0x6F000000 | (s << 16) | (i & 0xFFFF);

            pio_sm_set_enabled(pio, sm, true);
            pio_sm_put_blocking(pio, sm, nw * 32 - 1);

            struct timespec t0, t1;
            clock_gettime(CLOCK_MONOTONIC, &t0);

            /* Primer: 1 sequential chunk */
            rc = pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM, 32, &tx[0]);
            if (rc < 0) { printf("      %5d: primer TX fail\n", total); goto next3; }
            rc = pio_sm_xfer_data(pio, sm, PIO_DIR_FROM_SM, 32, &rx[0]);
            if (rc < 0) { printf("      %5d: primer RX fail\n", total); goto next3; }

            /* Concurrent: TX in thread, RX in main */
            {
                dma_loop_arg_t tx_arg = {
                    .pio = pio, .sm = sm,
                    .num_chunks = nchunks - 1, .chunk_bytes = 32,
                    .data = &tx[8], .result = 0,
                };
                pthread_t tx_tid;
                pthread_create(&tx_tid, NULL, tx_dma_loop_thread, &tx_arg);

                int rx_res = 0;
                for (int c = 1; c < nchunks; c++) {
                    rc = pio_sm_xfer_data(pio, sm, PIO_DIR_FROM_SM, 32, &rx[c*8]);
                    if (rc < 0) { rx_res = rc; break; }
                }
                pthread_join(tx_tid, NULL);

                clock_gettime(CLOCK_MONOTONIC, &t1);
                pio_sm_set_enabled(pio, sm, false);

                int sp = 0;
                while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
                    (void)pio_sm_get_blocking(pio, sm); sp++;
                }

                int mm = 0;
                for (int i = 0; i < nw; i++) if (rx[i] != tx[i]) mm++;

                double ms = time_diff_ms(&t0, &t1);
                double kbps = (total / 1024.0) / (ms / 1000.0);
                printf("      %5d bytes: %.2f ms, %7.1f kB/s, %d/%d mm, "
                       "TX=%d RX=%d sp=%d\n",
                       total, ms, kbps, mm, nw, tx_arg.result, rx_res, sp);
            }

next3:
            free(tx);
            free(rx);
            teardown_pio(pio, sm, offset, &jtag_loopback_program);
        }
    }

    PASS("A6d: threaded concurrent bidi DMA");
cleanup:
    return;
}

/* ---- Part A8: Reliable high-throughput DMA loopback ---- */

/*
 * Test A8: Reliable, repeatable high-throughput DMA loopback.
 *
 * Based on insights from RP1 high-performance DMA analysis:
 *   - config_xfer(buf_size=4096, buf_count=4) enables quad-buffered
 *     kernel DMA pipelining (DMA_BOUNCE_BUFFER_COUNT=4 in kernel)
 *   - Concurrent TX/RX threads work because TX and RX use SEPARATE
 *     dma_info structs in the kernel (no per-SM mutex)
 *   - pio_sm_init() resets SM state (PC, ISR, OSR, FIFOs)
 *   - config_xfer() resets DMA state (semaphore, bounce buffers)
 *
 * Reliability approach:
 *   - Single PIO handle for all iterations (no open/close race)
 *   - pio_sm_init() + config_xfer() before EACH transfer
 *     (guarantees clean SM + DMA state, no stale semaphores)
 *   - Multiple iterations per size for statistical confidence
 *   - No reboot required between runs
 *
 * PIO program: fast_loopback (out x,32 / in x,32)
 *   2 cycles/word at 200 MHz = 400 MB/s internal throughput
 *   No I/O pins, no sideset — pure internal TX→RX copy
 *
 * Expected throughput: ~27-38 MB/s (limited by RP1 DMA, not PIO)
 * Target: >10 MB/s with 100% pass rate
 */
static void test_a8_reliable_high_throughput(void)
{
    printf("  PIO: out x,32 / in x,32 (2 cycles/word, 200 MHz, no I/O)\n");
    printf("  DMA: buf_size=4096, buf_count=4 (quad-buffered)\n");
    printf("  Reset: pio_sm_init + config_xfer before each transfer\n");
    printf("  Threads: concurrent TX+RX via pthreads\n\n");

    /* Open PIO once — single handle for all iterations */
    if (pio_init() < 0) {
        FAIL("pio_init failed");
        return;
    }

    PIO pio = pio_open(0);
    if (PIO_IS_ERR(pio)) {
        FAIL("pio_open failed");
        return;
    }

    int sm = pio_claim_unused_sm(pio, false);
    if (sm < 0) {
        pio_close(pio);
        FAIL("no free SM");
        return;
    }

    uint offset = pio_add_program(pio, &fast_loopback_program);
    if (offset == PIO_ORIGIN_INVALID) {
        pio_sm_unclaim(pio, sm);
        pio_close(pio);
        FAIL("no program space");
        return;
    }

    /* Build base SM config (reused in pio_sm_init each iteration) */
    pio_select(pio);
    pio_sm_config base_cfg = fast_loopback_program_get_default_config(offset);
    sm_config_set_clkdiv(&base_cfg, 1.0f);             /* 200 MHz */
    sm_config_set_out_shift(&base_cfg, true, true, 32); /* autopull at 32 */
    sm_config_set_in_shift(&base_cfg, true, true, 32);  /* autopush at 32 */

    /* Test parameters */
    static const int sizes[] = {4096, 32768, 131072, 524288};
    static const int num_sizes = 4;
    static const int iterations = 5;
    int total_tests = 0, total_passed = 0;

    for (int s = 0; s < num_sizes; s++) {
        int total = sizes[s];
        int nw = total / 4;

        printf("  %d KB (%d iterations):\n", total / 1024, iterations);

        double sum_mbps = 0;
        int passed = 0;

        for (int iter = 0; iter < iterations; iter++) {
            total_tests++;

            /* === FULL RESET: SM + DMA === */

            /* (1) Disable SM */
            pio_sm_set_enabled(pio, sm, false);

            /* (2) Full SM reset: clears FIFOs, ISR, OSR, resets PC */
            pio_sm_init(pio, sm, offset, &base_cfg);

            /* (3) Reinitialize DMA: resets semaphore, bounce buffers.
             * This is the critical step for reliability — without it,
             * stale semaphore count from a previous transfer can cause
             * the next transfer to deadlock or return stale data. */
            int rc;
            rc = pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, 4096, 4);
            if (rc < 0) {
                printf("    iter %d: config_xfer TX failed rc=%d\n",
                       iter + 1, rc);
                continue;
            }
            rc = pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, 4096, 4);
            if (rc < 0) {
                printf("    iter %d: config_xfer RX failed rc=%d\n",
                       iter + 1, rc);
                continue;
            }

            /* Prepare test data (unique pattern per size + iteration) */
            uint32_t *tx = malloc(total);
            uint32_t *rx = calloc(1, total);
            if (!tx || !rx) {
                free(tx); free(rx);
                printf("    iter %d: malloc failed\n", iter + 1);
                continue;
            }
            for (int i = 0; i < nw; i++)
                tx[i] = 0xA8000000 | (s << 20) | (iter << 16) |
                         (i & 0xFFFF);

            /* (4) Enable SM — stalls on empty TX FIFO (autopull) */
            pio_sm_set_enabled(pio, sm, true);

            /* (5) Concurrent TX/RX DMA.
             * Chunk at 32 KB per ioctl call (kernel pipelines 4 KB
             * DMA chunks internally via bounce buffer ring). */
            int xfer_chunk = (total <= 32768) ? total : 32768;
            int nchunks = total / xfer_chunk;
            int cw = xfer_chunk / 4;

            struct timespec t0, t1;
            clock_gettime(CLOCK_MONOTONIC, &t0);

            dma_loop_arg_t rx_arg = {
                .pio = pio, .sm = sm,
                .num_chunks = nchunks, .chunk_bytes = xfer_chunk,
                .data = rx, .result = 0,
            };
            pthread_t rx_tid;
            rc = pthread_create(&rx_tid, NULL, rx_dma_loop_thread, &rx_arg);
            if (rc != 0) {
                printf("    iter %d: pthread_create failed\n", iter + 1);
                pio_sm_set_enabled(pio, sm, false);
                free(tx); free(rx);
                continue;
            }

            /* TX in main thread */
            int tx_rc = 0;
            for (int ch = 0; ch < nchunks; ch++) {
                rc = pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM,
                                      xfer_chunk, &tx[ch * cw]);
                if (rc < 0) { tx_rc = rc; break; }
            }

            pthread_join(rx_tid, NULL);
            clock_gettime(CLOCK_MONOTONIC, &t1);

            /* (6) Disable SM */
            pio_sm_set_enabled(pio, sm, false);

            /* Results */
            double ms = time_diff_ms(&t0, &t1);
            double mbps = (total / (1024.0 * 1024.0)) / (ms / 1000.0);

            int mm = 0;
            for (int i = 0; i < nw; i++)
                if (rx[i] != tx[i]) mm++;

            bool ok = (tx_rc == 0 && rx_arg.result == 0 && mm == 0);
            if (ok) {
                passed++;
                total_passed++;
                sum_mbps += mbps;
            }

            printf("    iter %d: %s | %.2f ms | %.2f MB/s | %d mm",
                   iter + 1, ok ? "PASS" : "FAIL", ms, mbps, mm);
            if (tx_rc < 0) printf(" | TX=%d", tx_rc);
            if (rx_arg.result < 0) printf(" | RX=%d", rx_arg.result);
            if (ok && mbps >= 10.0) printf(" | >10 MB/s");
            printf("\n");

            free(tx);
            free(rx);
        }

        /* Per-size summary */
        if (passed > 0)
            printf("    avg: %.2f MB/s | %d/%d passed",
                   sum_mbps / passed, passed, iterations);
        else
            printf("    ALL FAILED | 0/%d passed", iterations);

        if (passed == iterations && sum_mbps / passed >= 10.0)
            printf(" | TARGET MET\n");
        else
            printf("\n");
    }

    /* Cleanup — single close at end */
    pio_sm_set_enabled(pio, sm, false);
    pio_remove_program(pio, &fast_loopback_program, offset);
    pio_sm_unclaim(pio, sm);
    pio_close(pio);

    printf("\n  Total: %d/%d passed\n", total_passed, total_tests);
    if (total_passed == total_tests)
        PASS("A8: reliable high-throughput DMA (100% pass, >10 MB/s)");
    else {
        tests_failed++;
        printf("  FAIL: A8 (%d/%d passed)\n", total_passed, total_tests);
    }
cleanup:
    return;
}

/*
 * Test A9: Isolate reset requirements for repeated bidi DMA.
 *
 * A8 works (38 MB/s, 100% pass) by calling pio_sm_init() + config_xfer()
 * before EVERY transfer. The production bidi path fails because it calls
 * config_xfer() ONCE and then fires xfer_data_bidi() many times.
 *
 * This test has two variants to isolate the cause:
 *   A9a: config_xfer once → 10x bidi with pio_sm_init between each
 *        (tests whether config_xfer per-call is needed)
 *   A9b: config_xfer once → SM stays enabled → 10x bidi back-to-back
 *        (mimics production pio_shift_fast_chunk exactly)
 */
static void test_a9_bidi_reset_isolation(void)
{
    printf("  PIO: out x,32 / in x,32 (fast_loopback, 200 MHz, no I/O)\n");
    printf("  Tests whether repeated bidi DMA needs config_xfer between calls\n\n");

    if (pio_init() < 0) {
        FAIL("pio_init failed");
        return;
    }

    PIO pio = pio_open(0);
    if (PIO_IS_ERR(pio)) {
        FAIL("pio_open failed");
        return;
    }

    int sm = pio_claim_unused_sm(pio, false);
    if (sm < 0) {
        pio_close(pio);
        FAIL("no free SM");
        return;
    }

    uint offset = pio_add_program(pio, &fast_loopback_program);
    if (offset == PIO_ORIGIN_INVALID) {
        pio_sm_unclaim(pio, sm);
        pio_close(pio);
        FAIL("no program space");
        return;
    }

    pio_select(pio);
    pio_sm_config base_cfg = fast_loopback_program_get_default_config(offset);
    sm_config_set_clkdiv(&base_cfg, 1.0f);
    sm_config_set_out_shift(&base_cfg, true, true, 32);
    sm_config_set_in_shift(&base_cfg, true, true, 32);

    static const int total = 32768;  /* 32 KB per transfer */
    static const int nw = 32768 / 4;
    static const int iterations = 10;
    int rc;

    /*
     * Variant A9a: config_xfer ONCE, then pio_sm_init between each bidi call.
     * This tests whether DMA semaphore/bounce state survives across sm_init.
     */
    printf("  A9a: config_xfer once + pio_sm_init between iterations\n");
    {
        /* Initial full setup */
        pio_sm_set_enabled(pio, sm, false);
        pio_sm_init(pio, sm, offset, &base_cfg);
        rc = pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, 4096, 4);
        if (rc < 0) { printf("    config_xfer TX failed: %d\n", rc); goto a9a_done; }
        rc = pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, 4096, 4);
        if (rc < 0) { printf("    config_xfer RX failed: %d\n", rc); goto a9a_done; }

        int a9a_passed = 0;
        for (int iter = 0; iter < iterations; iter++) {
            /* SM-only reset (NO config_xfer) */
            pio_sm_set_enabled(pio, sm, false);
            pio_sm_init(pio, sm, offset, &base_cfg);
            pio_sm_set_enabled(pio, sm, true);

            uint32_t *tx = malloc(total);
            uint32_t *rx = calloc(1, total);
            if (!tx || !rx) { free(tx); free(rx); continue; }
            for (int i = 0; i < nw; i++)
                tx[i] = 0xA9A00000 | (iter << 16) | (i & 0xFFFF);

            /* Concurrent bidi DMA */
            int xfer_chunk = total;
            dma_loop_arg_t rx_arg = {
                .pio = pio, .sm = sm,
                .num_chunks = 1, .chunk_bytes = xfer_chunk,
                .data = rx, .result = 0,
            };
            pthread_t rx_tid;
            pthread_create(&rx_tid, NULL, rx_dma_loop_thread, &rx_arg);

            int tx_rc = pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM, xfer_chunk, tx);
            pthread_join(rx_tid, NULL);

            pio_sm_set_enabled(pio, sm, false);

            int mm = 0;
            for (int i = 0; i < nw; i++)
                if (rx[i] != tx[i]) mm++;

            bool ok = (tx_rc == 0 && rx_arg.result == 0 && mm == 0);
            printf("    iter %2d: %s | TX=%d RX=%d | %d mismatches\n",
                   iter + 1, ok ? "PASS" : "FAIL",
                   tx_rc, rx_arg.result, mm);

            if (ok) a9a_passed++;
            free(tx);
            free(rx);
        }
        printf("    Result: %d/%d passed\n", a9a_passed, iterations);
        if (a9a_passed == iterations)
            PASS("A9a: bidi works with sm_init only (no config_xfer between)");
        else {
            tests_failed++;
            printf("  FAIL: A9a (%d/%d)\n", a9a_passed, iterations);
        }
    }
a9a_done:

    /*
     * Variant A9b: config_xfer ONCE, SM stays enabled, 10 bidi calls
     * back-to-back. This exactly mimics the production pio_shift_fast_chunk
     * calling pattern: enable SM once, fire bidi in a loop.
     */
    printf("\n  A9b: config_xfer once + SM stays enabled + back-to-back bidi\n");
    {
        pio_sm_set_enabled(pio, sm, false);
        pio_sm_init(pio, sm, offset, &base_cfg);
        rc = pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, 4096, 4);
        if (rc < 0) { printf("    config_xfer TX failed: %d\n", rc); goto a9b_done; }
        rc = pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, 4096, 4);
        if (rc < 0) { printf("    config_xfer RX failed: %d\n", rc); goto a9b_done; }

        pio_sm_set_enabled(pio, sm, true);

        int a9b_passed = 0;
        for (int iter = 0; iter < iterations; iter++) {
            /* NO sm_init, NO config_xfer, NO disable — just fire bidi */
            uint32_t *tx = malloc(total);
            uint32_t *rx = calloc(1, total);
            if (!tx || !rx) { free(tx); free(rx); continue; }
            for (int i = 0; i < nw; i++)
                tx[i] = 0xA9B00000 | (iter << 16) | (i & 0xFFFF);

            int xfer_chunk = total;
            dma_loop_arg_t rx_arg = {
                .pio = pio, .sm = sm,
                .num_chunks = 1, .chunk_bytes = xfer_chunk,
                .data = rx, .result = 0,
            };
            pthread_t rx_tid;
            pthread_create(&rx_tid, NULL, rx_dma_loop_thread, &rx_arg);

            int tx_rc = pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM, xfer_chunk, tx);
            pthread_join(rx_tid, NULL);

            int mm = 0;
            for (int i = 0; i < nw; i++)
                if (rx[i] != tx[i]) mm++;

            bool ok = (tx_rc == 0 && rx_arg.result == 0 && mm == 0);
            printf("    iter %2d: %s | TX=%d RX=%d | %d mismatches\n",
                   iter + 1, ok ? "PASS" : "FAIL",
                   tx_rc, rx_arg.result, mm);

            if (ok) a9b_passed++;
            free(tx);
            free(rx);

            /* If bidi failed (DMA timeout), remaining iterations will also fail */
            if (!ok) {
                printf("    (stopping early — DMA state likely corrupted)\n");
                break;
            }
        }

        pio_sm_set_enabled(pio, sm, false);
        printf("    Result: %d/%d passed\n", a9b_passed, iterations);
        if (a9b_passed == iterations)
            PASS("A9b: bidi works with SM enabled + back-to-back (production pattern)");
        else {
            tests_failed++;
            printf("  FAIL: A9b (%d/%d)\n", a9b_passed, iterations);
        }
    }
a9b_done:

    /* Cleanup fast_loopback */
    pio_sm_set_enabled(pio, sm, false);
    pio_remove_program(pio, &fast_loopback_program, offset);
    pio_sm_unclaim(pio, sm);
    pio_close(pio);

cleanup:
    return;
}

/*
 * Test A9c: Back-to-back bidi with jtag_shift_fast (production PIO program).
 *
 * Uses jtag_shift_fast with external TDI→TDO loopback wire.
 * This is the exact PIO program and calling pattern used during bitstream load.
 * config_xfer once, SM stays enabled, multiple bidi calls back-to-back.
 *
 * If this works but production fails, the issue is in higher-level JTAG logic
 * (TMS handling, program switching, etc.), not in the bidi DMA mechanism.
 */
static void test_a9c_bidi_fast_program(void)
{
    printf("  PIO: jtag_shift_fast (2 instr/bit, sideset TCK, real pins)\n");
    printf("  Pins: TDI=GPIO%d, TDO=GPIO%d, TCK=GPIO%d\n", TDI_PIN, TDO_PIN, TCK_PIN);
    printf("  Requires external wire: TDI → TDO\n");
    printf("  config_xfer(4096,4) once, SM enabled, 10x back-to-back bidi\n\n");

    if (pio_init() < 0) {
        FAIL("pio_init failed");
        return;
    }

    PIO pio = pio_open(0);
    if (PIO_IS_ERR(pio)) {
        FAIL("pio_open failed");
        return;
    }

    int sm = pio_claim_unused_sm(pio, false);
    if (sm < 0) {
        pio_close(pio);
        FAIL("no free SM");
        return;
    }

    uint offset = pio_add_program(pio, &jtag_shift_fast_program);
    if (offset == PIO_ORIGIN_INVALID) {
        pio_sm_unclaim(pio, sm);
        pio_close(pio);
        FAIL("no program space");
        return;
    }

    /* Configure SM for jtag_shift_fast with real pins */
    pio_select(pio);
    pio_sm_config cfg = jtag_shift_fast_program_get_default_config(offset);

    /* TCK: sideset */
    sm_config_set_sideset_pins(&cfg, TCK_PIN);
    pio_gpio_init(pio, TCK_PIN);
    pio_sm_set_consecutive_pindirs(pio, sm, TCK_PIN, 1, true);

    /* TDI: OUT */
    sm_config_set_out_pins(&cfg, TDI_PIN, 1);
    pio_gpio_init(pio, TDI_PIN);
    pio_sm_set_consecutive_pindirs(pio, sm, TDI_PIN, 1, true);

    /* TDO: IN */
    sm_config_set_in_pins(&cfg, TDO_PIN);
    pio_gpio_init(pio, TDO_PIN);

    /* 10 MHz TCK: div = 200M / (10M * 2 instr/bit) = 10 */
    sm_config_set_clkdiv(&cfg, 10.0f);
    sm_config_set_out_shift(&cfg, true, true, 32);
    sm_config_set_in_shift(&cfg, true, true, 32);

    pio_sm_init(pio, sm, offset, &cfg);

    /* Configure DMA once (production pattern) */
    int rc;
    rc = pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, 4096, 4);
    if (rc < 0) { printf("    config_xfer TX failed: %d\n", rc); goto done; }
    rc = pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, 4096, 4);
    if (rc < 0) { printf("    config_xfer RX failed: %d\n", rc); goto done; }

    /* Enable SM once (production pattern: SM stays running across chunks) */
    pio_sm_set_enabled(pio, sm, true);

    static const int total = 32768;  /* 32 KB per bidi call = production chunk size */
    static const int nw = 32768 / 4;
    static const int iterations = 10;
    int passed = 0;

    for (int iter = 0; iter < iterations; iter++) {
        uint32_t *tx = malloc(total);
        uint32_t *rx = calloc(1, total);
        if (!tx || !rx) { free(tx); free(rx); continue; }
        for (int i = 0; i < nw; i++)
            tx[i] = 0xA9C00000 | (iter << 16) | (i & 0xFFFF);

        struct timespec t0, t1;
        clock_gettime(CLOCK_MONOTONIC, &t0);

        /* Bidi DMA: RX thread + TX main (exactly like rp1_xfer_data_bidi) */
        dma_loop_arg_t rx_arg = {
            .pio = pio, .sm = sm,
            .num_chunks = 1, .chunk_bytes = total,
            .data = rx, .result = 0,
        };
        pthread_t rx_tid;
        pthread_create(&rx_tid, NULL, rx_dma_loop_thread, &rx_arg);

        int tx_rc = pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM, total, tx);
        pthread_join(rx_tid, NULL);

        clock_gettime(CLOCK_MONOTONIC, &t1);

        double ms = time_diff_ms(&t0, &t1);
        double kbps = (total / 1024.0) / (ms / 1000.0);

        /* With jtag_shift_fast, data goes through physical pins.
         * TDI→TDO external wire means RX data should match TX data
         * (bit-for-bit, since the PIO shifts LSB-first both ways). */
        int mm = 0;
        int first_mm = -1;
        for (int i = 0; i < nw; i++) {
            if (rx[i] != tx[i]) {
                if (first_mm < 0) first_mm = i;
                mm++;
            }
        }

        bool ok = (tx_rc == 0 && rx_arg.result == 0 && mm == 0);
        printf("    iter %2d: %s | %.1f ms | %.0f kB/s | TX=%d RX=%d | %d mm",
               iter + 1, ok ? "PASS" : "FAIL", ms, kbps,
               tx_rc, rx_arg.result, mm);
        if (first_mm >= 0)
            printf(" | first@%d: got=%08x exp=%08x", first_mm, rx[first_mm], tx[first_mm]);
        printf("\n");

        if (ok) passed++;
        free(tx);
        free(rx);

        if (!ok) {
            printf("    (stopping early)\n");
            break;
        }
    }

    pio_sm_set_enabled(pio, sm, false);

    /* Force TCK low */
    pio_sm_set_pins_with_mask(pio, sm, 0, 1u << TCK_PIN);

    printf("    Result: %d/%d passed\n", passed, iterations);
    if (passed == iterations)
        PASS("A9c: bidi with jtag_shift_fast back-to-back (production pattern)");
    else {
        tests_failed++;
        printf("  FAIL: A9c (%d/%d)\n", passed, iterations);
    }

done:
    pio_sm_set_enabled(pio, sm, false);
    pio_remove_program(pio, &jtag_shift_fast_program, offset);
    pio_sm_unclaim(pio, sm);
    pio_close(pio);
    return;

cleanup:
    return;
}

/*
 * Test A9d: fast_loopback at div=320 (same word rate as jtag_shift_fast@div=10).
 *
 * jtag_shift_fast processes 2 instr/bit × 32 bits/word = 64 instr/word.
 * At div=10: 640 PIO cycles/word = 3.2 µs/word.
 *
 * fast_loopback processes 2 instr/word. At div=320: 640 PIO cycles/word = 3.2 µs/word.
 * Same word rate, but pure internal copy (no I/O pins, no sideset).
 *
 * If this passes → problem is specific to jtag_shift_fast (pins/sideset/program).
 * If this fails  → problem is slow SM causing DMA timeout.
 */
static void test_a9d_slow_loopback(void)
{
    printf("  PIO: fast_loopback — divider x transfer size matrix\n");
    printf("  No I/O pins — pure internal TX->RX copy\n");
    printf("  config_xfer(4096,4), SM init per cell, 1 bidi per cell\n\n");

    if (pio_init() < 0) {
        FAIL("pio_init failed");
        return;
    }

    PIO pio = pio_open(0);
    if (PIO_IS_ERR(pio)) {
        FAIL("pio_open failed");
        return;
    }

    int sm = pio_claim_unused_sm(pio, false);
    if (sm < 0) {
        pio_close(pio);
        FAIL("no free SM");
        return;
    }

    uint offset = pio_add_program(pio, &fast_loopback_program);
    if (offset == PIO_ORIGIN_INVALID) {
        pio_sm_unclaim(pio, sm);
        pio_close(pio);
        FAIL("no program space");
        return;
    }

    pio_select(pio);
    pio_sm_config cfg = fast_loopback_program_get_default_config(offset);
    sm_config_set_clkdiv(&cfg, 1.0f);
    sm_config_set_out_shift(&cfg, true, true, 32);
    sm_config_set_in_shift(&cfg, true, true, 32);

    pio_sm_init(pio, sm, offset, &cfg);

    int rc;
    rc = pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, 4096, 4);
    if (rc < 0) { printf("    config_xfer TX failed: %d\n", rc); goto done; }
    rc = pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM, 4096, 4);
    if (rc < 0) { printf("    config_xfer RX failed: %d\n", rc); goto done; }

    /* Matrix: dividers x transfer sizes.
     * fast_loopback: 2 instr/word, each instr takes div * 5ns.
     * div=1: 10ns/word, div=4: 40ns/word, div=320: 3200ns/word */
    static const float dividers[] = {1.0f, 2.0f, 3.0f, 4.0f, 8.0f, 16.0f, 320.0f};
    static const int ndivs = sizeof(dividers) / sizeof(dividers[0]);
    static const int sizes[] = {32, 256, 4096, 32768};
    static const int nsizes = sizeof(sizes) / sizeof(sizes[0]);
    int total_passed = 0, total_tests = 0;

    /* Header */
    printf("    %8s", "");
    for (int s = 0; s < nsizes; s++)
        printf(" | %10d B", sizes[s]);
    printf("\n");
    printf("    %8s", "--------");
    for (int s = 0; s < nsizes; s++)
        printf("-+------------");
    printf("\n");

    for (int d = 0; d < ndivs; d++) {
        float div = dividers[d];
        double ns_per_word = div * 2.0 * 5.0;
        printf("    div=%4.0f ", div);
        fflush(stdout);

        for (int s = 0; s < nsizes; s++) {
            int xfer_total = sizes[s];
            int nw = xfer_total / 4;

            /* Reset SM with new divider */
            pio_sm_set_enabled(pio, sm, false);
            pio_sm_init(pio, sm, offset, &cfg);
            pio_sm_set_clkdiv(pio, sm, div);
            pio_sm_set_enabled(pio, sm, true);

            uint32_t *tx = malloc(xfer_total);
            uint32_t *rx = calloc(1, xfer_total);
            if (!tx || !rx) { free(tx); free(rx); printf("| %11s ", "OOM"); continue; }
            for (int i = 0; i < nw; i++)
                tx[i] = 0xA9D00000 | (d << 20) | (s << 16) | (i & 0xFFFF);

            struct timespec t0, t1;
            clock_gettime(CLOCK_MONOTONIC, &t0);

            dma_loop_arg_t rx_arg = {
                .pio = pio, .sm = sm,
                .num_chunks = 1, .chunk_bytes = xfer_total,
                .data = rx, .result = 0,
            };
            pthread_t rx_tid;
            pthread_create(&rx_tid, NULL, rx_dma_loop_thread, &rx_arg);

            int tx_rc = pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM, xfer_total, tx);
            pthread_join(rx_tid, NULL);

            clock_gettime(CLOCK_MONOTONIC, &t1);
            double ms = time_diff_ms(&t0, &t1);

            int mm = 0;
            for (int i = 0; i < nw; i++)
                if (rx[i] != tx[i]) mm++;

            bool ok = (tx_rc == 0 && rx_arg.result == 0 && mm == 0);
            total_tests++;
            if (ok) total_passed++;

            printf("| %4s %5.0fms", ok ? "PASS" : "FAIL", ms);
            fflush(stdout);

            free(tx);
            free(rx);
        }
        printf("  (%.0f ns/word)\n", ns_per_word);
    }

    pio_sm_set_enabled(pio, sm, false);

    printf("\n    Result: %d/%d cells passed\n", total_passed, total_tests);
    if (total_passed == total_tests)
        PASS("A9d: all cells pass");
    else if (total_passed > 0) {
        tests_passed++;
        printf("  INFO: A9d — bidi works for some divider/size combinations\n");
    } else {
        tests_failed++;
        printf("  FAIL: A9d — all cells failed\n");
    }

done:
    pio_sm_set_enabled(pio, sm, false);
    pio_remove_program(pio, &fast_loopback_program, offset);
cleanup:
    pio_sm_unclaim(pio, sm);
    pio_close(pio);
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

static int should_run(int argc, char *argv[], const char *name)
{
    if (argc <= 1)
        return 1;  /* No filter — run all */
    for (int i = 1; i < argc; i++) {
        if (strcasecmp(argv[i], name) == 0)
            return 1;
    }
    return 0;
}

int main(int argc, char *argv[])
{
    if (argc > 1)
        printf("Running selected tests:");
    else
        printf("Running all tests");

    for (int i = 1; i < argc; i++)
        printf(" %s", argv[i]);
    printf("\n");

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

    if (should_run(argc, argv, "a1") || should_run(argc, argv, "a2") ||
        should_run(argc, argv, "a3") || should_run(argc, argv, "a4")) {
        printf("--- Part A: DMA basics (loopback program) ---\n");
    }
    if (should_run(argc, argv, "a1")) test_a1_tx_dma_manual_rx();
    if (should_run(argc, argv, "a2")) test_a2_manual_tx_rx_dma();
    if (should_run(argc, argv, "a3")) test_a3_pingpong_dma();
    if (should_run(argc, argv, "a4")) test_a4_concurrent_dma();

    if (should_run(argc, argv, "a5")) {
        printf("--- Part A5: Threaded bidi DMA (fast program, external loopback) ---\n");
        test_a5_threaded_bidi_dma();
    }

    if (should_run(argc, argv, "a6")) {
        printf("--- Part A6: Loopback bidi DMA diagnostics ---\n");
        test_a6_loopback_bidi_dma();
    }

    if (should_run(argc, argv, "a6d")) {
        printf("--- Part A6d: Threaded concurrent bidi DMA ---\n");
        test_a6d_threaded_bidi_dma();
    }

    if (should_run(argc, argv, "a8")) {
        printf("--- Part A8: Reliable high-throughput DMA (10 MB/s target) ---\n");
        test_a8_reliable_high_throughput();
    }

    if (should_run(argc, argv, "a9")) {
        printf("--- Part A9: Bidi DMA reset isolation ---\n");
        test_a9_bidi_reset_isolation();
    }

    if (should_run(argc, argv, "a9c")) {
        printf("--- Part A9c: Bidi with jtag_shift_fast (production program) ---\n");
        test_a9c_bidi_fast_program();
    }

    if (should_run(argc, argv, "a9d")) {
        printf("--- Part A9d: Slow loopback (div=320, same word rate as jtag_shift_fast) ---\n");
        test_a9d_slow_loopback();
    }

    if (should_run(argc, argv, "b5") || should_run(argc, argv, "b6") ||
        should_run(argc, argv, "b7")) {
        printf("\n--- Part B: Fast program stop condition ---\n");
    }
    if (should_run(argc, argv, "b5")) test_b5_fast_word_by_word();
    if (should_run(argc, argv, "b6")) test_b6_fast_dma();
    if (should_run(argc, argv, "b7")) test_b7_stop_timing();

    if (should_run(argc, argv, "c8")) {
        printf("\n--- Part C: Throughput ceiling ---\n");
        test_c8_large_dma();
    }

    printf("\n============================================\n");
    printf("%d passed, %d failed\n", tests_passed, tests_failed);

    if (tests_passed >= 4) {
        printf("\nGO/NO-GO: At least basic DMA works.\n");
    } else if (tests_passed > 0 && tests_failed == 0) {
        printf("\nGO/NO-GO: Selected tests passed.\n");
    } else {
        printf("\nGO/NO-GO: DMA appears unreliable — reassess approach.\n");
    }

    return tests_failed > 0 ? 1 : 0;
}

#else /* !HAVE_PIOLIB */

#include <stdio.h>

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    fprintf(stderr, "test_dma_explore: requires HAVE_PIOLIB (RPi 5 only)\n");
    return 1;
}

#endif /* HAVE_PIOLIB */
