/*
 * test_txonly_diag.c - Diagnose TX-only DMA failure for >256 bits
 *
 * Root cause investigation for: TX-only DMA with autopush OFF works for
 * ≤256 bits but fails for ≥512 bits. SM appears to stall despite
 * TX_FIFO=0 (all data consumed).
 *
 * Hypothesis tests:
 *   A. RX config_xfer interferes with RX FIFO level reads
 *   B. DMA corrupts count word (wrong X value)
 *   C. DMA doesn't deliver all words (partial transfer)
 *   D. DMACTRL threshold affects behavior
 *   E. config_xfer buf_size must match transfer size
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

/* Pins: TDI=27, TDO=22, TCK=4, TMS=17 */
#define PIN_TDI 27
#define PIN_TDO 22
#define PIN_TCK 4
#define PIN_TMS 17

/* Read SM's X register by executing: mov isr, x ; push
 * SM must be disabled before calling this.
 * Returns X value via RX FIFO read. */
static uint32_t read_x_register(PIO pio, uint sm)
{
    /* Clear ISR first */
    pio_sm_exec(pio, sm, pio_encode_mov(pio_isr, pio_null));
    /* mov isr, x: transfer X to ISR */
    pio_sm_exec(pio, sm, pio_encode_mov(pio_isr, pio_x));
    /* push: move ISR to RX FIFO */
    pio_sm_exec(pio, sm, pio_encode_push(false, false));
    /* Read from RX FIFO */
    return pio_sm_get(pio, sm);
}

typedef struct {
    uint32_t num_bits;
    bool rx_config_xfer;     /* Whether to call config_xfer for RX */
    bool count_via_put;      /* Count word via put_blocking vs in DMA */
    uint32_t dmactrl_tx;     /* DMACTRL value for TX */
    uint32_t config_buf_size;/* buf_size for config_xfer */
    const char *label;
} test_config_t;

static int run_test(PIO pio, uint sm, uint offset, test_config_t *cfg)
{
    uint32_t num_bits = cfg->num_bits;
    uint32_t num_data_words = (num_bits + 31) / 32;
    size_t tx_bytes;

    if (cfg->count_via_put) {
        tx_bytes = num_data_words * 4;
    } else {
        tx_bytes = (1 + num_data_words) * 4;
    }
    if (tx_bytes < 8) tx_bytes = 8;

    /* Allocate and fill TX buffer */
    size_t alloc = (tx_bytes + 63) & ~(size_t)63;
    uint32_t *tx_buf = aligned_alloc(64, alloc);
    if (!tx_buf) return -1;
    memset(tx_buf, 0xAA, tx_bytes);

    if (cfg->count_via_put) {
        /* Data only in DMA buffer */
    } else {
        tx_buf[0] = num_bits - 1;  /* count word */
    }

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
    int ret;
    ret = pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM,
                              cfg->config_buf_size, 1);
    if (ret < 0) {
        printf("    config_xfer TX failed: ret=%d errno=%s\n",
               ret, strerror(errno));
        free(tx_buf);
        return -1;
    }

    if (cfg->rx_config_xfer) {
        ret = pio_sm_config_xfer(pio, sm, PIO_DIR_FROM_SM,
                                  cfg->config_buf_size, 1);
        if (ret < 0) {
            printf("    config_xfer RX failed: ret=%d errno=%s\n",
                   ret, strerror(errno));
            free(tx_buf);
            return -1;
        }
    }

    pio_sm_set_dmactrl(pio, sm, true, cfg->dmactrl_tx);

    /* Start SM at instruction 0 (pull) */
    pio_sm_exec(pio, sm, pio_encode_jmp(offset));
    pio_sm_set_enabled(pio, sm, true);

    /* If count_via_put: send count word before DMA */
    if (cfg->count_via_put) {
        pio_sm_put_blocking(pio, sm, num_bits - 1);
    }

    /* TX DMA */
    errno = 0;
    ret = pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM, tx_bytes, tx_buf);
    int saved_errno = errno;

    if (ret < 0) {
        printf("    TX DMA failed: ret=%d errno=%s\n",
               ret, strerror(saved_errno));
        free(tx_buf);
        pio_sm_set_enabled(pio, sm, false);
        return -1;
    }

    /* Check FIFO levels immediately */
    uint32_t tx_level = pio_sm_get_tx_fifo_level(pio, sm);
    uint32_t rx_level = pio_sm_get_rx_fifo_level(pio, sm);

    /* Poll for SM completion (RX_FIFO > 0) */
    int timeout_us = 0;
    while (pio_sm_get_rx_fifo_level(pio, sm) == 0) {
        usleep(10);
        timeout_us += 10;
        if (timeout_us > 50000) {  /* 50ms timeout */
            break;
        }
    }

    uint32_t final_tx = pio_sm_get_tx_fifo_level(pio, sm);
    uint32_t final_rx = pio_sm_get_rx_fifo_level(pio, sm);

    if (final_rx > 0) {
        printf("    OK: completed in %d us (TX=%u RX=%u → TX=%u RX=%u)\n",
               timeout_us, tx_level, rx_level, final_tx, final_rx);
        free(tx_buf);
        pio_sm_set_enabled(pio, sm, false);
        pio_sm_clear_fifos(pio, sm);
        return 0;
    }

    /* FAILURE — SM didn't complete. Gather diagnostics. */
    printf("    TIMEOUT %d us: TX=%u→%u RX=%u→%u\n",
           timeout_us, tx_level, rx_level, final_tx, final_rx);

    /* Stop SM and read X register */
    pio_sm_set_enabled(pio, sm, false);

    uint32_t x_val = read_x_register(pio, sm);
    printf("    X register = %u (0x%08x)", x_val, x_val);
    if (x_val == 0) {
        printf(" — loop counter exhausted, push should have executed!\n");
    } else if (x_val == num_bits - 1) {
        printf(" — SM never started loop (count word was loaded but no data consumed)\n");
    } else {
        uint32_t bits_done = (num_bits - 1) - x_val;
        uint32_t words_consumed = bits_done / 32;
        printf(" — %u bits processed, %u remaining (%u words consumed of %u)\n",
               bits_done, x_val + 1, words_consumed, num_data_words);
    }

    pio_sm_clear_fifos(pio, sm);
    free(tx_buf);
    return -1;
}

int main(void)
{
    printf("TX-only DMA Diagnostic Test\n");
    printf("===========================\n\n");

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

    printf("Program offset: %u\n", offset);
    printf("jtag_shift instructions:\n");
    printf("  %u: pull   side 0  (count word)\n", offset + 0);
    printf("  %u: out x  side 0  (load X)\n", offset + 1);
    printf("  %u: out 1  side 0  (TDI, TCK LOW)\n", offset + 2);
    printf("  %u: nop    side 0  (setup time)\n", offset + 3);
    printf("  %u: in 1   side 1  (TDO, TCK HIGH)\n", offset + 4);
    printf("  %u: jmp x-- side 0 (loop)\n", offset + 5);
    printf("  %u: push   side 0  (flush ISR)\n", offset + 6);
    printf("\n");

    /* ===== Test A: Baseline — reproduce failure with 512 bits ===== */
    printf("=== Test A: Baseline (512 bits, with RX config_xfer) ===\n");
    {
        test_config_t cfg = {
            .num_bits = 512,
            .rx_config_xfer = true,
            .count_via_put = false,
            .dmactrl_tx = 0xC0000104,  /* threshold=4, hi-pri */
            .config_buf_size = 4096,
            .label = "baseline-512"
        };
        run_test(pio, sm, offset, &cfg);
    }
    printf("\n");

    /* ===== Test B: Without RX config_xfer ===== */
    printf("=== Test B: 512 bits, WITHOUT RX config_xfer ===\n");
    {
        test_config_t cfg = {
            .num_bits = 512,
            .rx_config_xfer = false,
            .count_via_put = false,
            .dmactrl_tx = 0xC0000104,
            .config_buf_size = 4096,
            .label = "no-rx-config-512"
        };
        run_test(pio, sm, offset, &cfg);
    }
    printf("\n");

    /* ===== Test C: Count word via put_blocking ===== */
    printf("=== Test C: 512 bits, count via put_blocking ===\n");
    {
        test_config_t cfg = {
            .num_bits = 512,
            .rx_config_xfer = true,
            .count_via_put = true,
            .dmactrl_tx = 0xC0000104,
            .config_buf_size = 4096,
            .label = "count-via-put-512"
        };
        run_test(pio, sm, offset, &cfg);
    }
    printf("\n");

    /* ===== Test D: config_xfer with exact transfer size ===== */
    printf("=== Test D: 512 bits, config_xfer buf_size=68 ===\n");
    {
        test_config_t cfg = {
            .num_bits = 512,
            .rx_config_xfer = true,
            .count_via_put = false,
            .dmactrl_tx = 0xC0000104,
            .config_buf_size = 68,
            .label = "exact-config-512"
        };
        run_test(pio, sm, offset, &cfg);
    }
    printf("\n");

    /* ===== Test E: Different DMACTRL thresholds ===== */
    printf("=== Test E: 512 bits, DMACTRL threshold=1 ===\n");
    {
        test_config_t cfg = {
            .num_bits = 512,
            .rx_config_xfer = true,
            .count_via_put = false,
            .dmactrl_tx = 0xC0000101,  /* threshold=1 */
            .config_buf_size = 4096,
            .label = "thresh1-512"
        };
        run_test(pio, sm, offset, &cfg);
    }
    printf("\n");

    printf("=== Test E2: 512 bits, DMACTRL threshold=8 ===\n");
    {
        test_config_t cfg = {
            .num_bits = 512,
            .rx_config_xfer = true,
            .count_via_put = false,
            .dmactrl_tx = 0xC0000108,  /* threshold=8 */
            .config_buf_size = 4096,
            .label = "thresh8-512"
        };
        run_test(pio, sm, offset, &cfg);
    }
    printf("\n");

    /* ===== Test F: Boundary sweep (fine granularity) ===== */
    printf("=== Test F: Boundary sweep (256-544 bits, step 32) ===\n");
    uint32_t sweep_sizes[] = {256, 288, 320, 352, 384, 416, 448, 480, 512, 544};
    int sweep_count = sizeof(sweep_sizes) / sizeof(sweep_sizes[0]);

    for (int i = 0; i < sweep_count; i++) {
        printf("  %u bits (%u TX words, %zu TX bytes): ",
               sweep_sizes[i],
               1 + (sweep_sizes[i] + 31) / 32,
               (size_t)(1 + (sweep_sizes[i] + 31) / 32) * 4);
        fflush(stdout);

        test_config_t cfg = {
            .num_bits = sweep_sizes[i],
            .rx_config_xfer = true,
            .count_via_put = false,
            .dmactrl_tx = 0xC0000104,
            .config_buf_size = 4096,
            .label = "sweep"
        };

        int rc = run_test(pio, sm, offset, &cfg);
        if (rc == 0) {
            printf("    PASS\n");
        } else {
            printf("    FAIL\n");
        }
    }
    printf("\n");

    /* ===== Test G: 256-bit control test (should pass) ===== */
    printf("=== Test G: 256-bit control (should PASS) ===\n");
    {
        test_config_t cfg = {
            .num_bits = 256,
            .rx_config_xfer = true,
            .count_via_put = false,
            .dmactrl_tx = 0xC0000104,
            .config_buf_size = 4096,
            .label = "control-256"
        };
        run_test(pio, sm, offset, &cfg);
    }
    printf("\n");

    /* ===== Test H: Count-via-put sweep ===== */
    printf("=== Test H: Count via put_blocking sweep ===\n");
    uint32_t put_sizes[] = {256, 512, 1024, 4096};
    int put_count = sizeof(put_sizes) / sizeof(put_sizes[0]);

    for (int i = 0; i < put_count; i++) {
        printf("  %u bits: ", put_sizes[i]);
        fflush(stdout);

        test_config_t cfg = {
            .num_bits = put_sizes[i],
            .rx_config_xfer = true,
            .count_via_put = true,
            .dmactrl_tx = 0xC0000104,
            .config_buf_size = 4096,
            .label = "put-sweep"
        };

        int rc = run_test(pio, sm, offset, &cfg);
        if (rc == 0) {
            printf("    PASS\n");
        } else {
            printf("    FAIL\n");
        }
    }
    printf("\n");

    /* ===== Test I: No RX config_xfer sweep ===== */
    printf("=== Test I: No RX config_xfer sweep ===\n");
    uint32_t norx_sizes[] = {256, 512, 1024, 4096};
    int norx_count = sizeof(norx_sizes) / sizeof(norx_sizes[0]);

    for (int i = 0; i < norx_count; i++) {
        printf("  %u bits: ", norx_sizes[i]);
        fflush(stdout);

        test_config_t cfg = {
            .num_bits = norx_sizes[i],
            .rx_config_xfer = false,
            .count_via_put = false,
            .dmactrl_tx = 0xC0000104,
            .config_buf_size = 4096,
            .label = "norx-sweep"
        };

        int rc = run_test(pio, sm, offset, &cfg);
        if (rc == 0) {
            printf("    PASS\n");
        } else {
            printf("    FAIL\n");
        }
    }
    printf("\n");

    /* ===== Test J: FIFO-write control (should always work) ===== */
    printf("=== Test J: FIFO-write control (no DMA, should always work) ===\n");
    {
        uint32_t fifo_sizes[] = {512, 1024, 4096};
        int fifo_count = sizeof(fifo_sizes) / sizeof(fifo_sizes[0]);

        for (int i = 0; i < fifo_count; i++) {
            uint32_t nbits = fifo_sizes[i];
            uint32_t nwords = (nbits + 31) / 32;

            printf("  %u bits (FIFO-write): ", nbits);
            fflush(stdout);

            pio_sm_set_enabled(pio, sm, false);
            pio_sm_clear_fifos(pio, sm);
            pio_sm_restart(pio, sm);

            pio_select(pio);
            {
                pio_sm_config c = jtag_shift_program_get_default_config(offset);
                sm_config_set_out_pins(&c, PIN_TDI, 1);
                sm_config_set_in_pins(&c, PIN_TDO);
                sm_config_set_sideset_pins(&c, PIN_TCK);
                sm_config_set_out_shift(&c, true, true, 32);
                sm_config_set_in_shift(&c, true, false, 32);
                float div = (float)RP1_PIO_CLK_HZ /
                            ((float)FREQ_HZ * INSTR_PER_BIT);
                if (div < 1.0f) div = 1.0f;
                sm_config_set_clkdiv(&c, div);
                pio_sm_init(pio, sm, offset, &c);
            }

            pio_sm_exec(pio, sm, pio_encode_jmp(offset));
            pio_sm_set_enabled(pio, sm, true);

            /* Write count word */
            pio_sm_put_blocking(pio, sm, nbits - 1);

            /* Write data words */
            for (uint32_t w = 0; w < nwords; w++) {
                pio_sm_put_blocking(pio, sm, 0xAAAAAAAA);
            }

            /* Wait for completion */
            int timeout_us = 0;
            while (pio_sm_get_rx_fifo_level(pio, sm) == 0) {
                usleep(10);
                timeout_us += 10;
                if (timeout_us > 50000) break;
            }

            if (pio_sm_get_rx_fifo_level(pio, sm) > 0) {
                printf("PASS (%d us)\n", timeout_us);
            } else {
                printf("FAIL (timeout)\n");
            }

            pio_sm_set_enabled(pio, sm, false);
            pio_sm_clear_fifos(pio, sm);
        }
    }
    printf("\n");

    printf("=== Diagnostics complete ===\n");

    pio_remove_program(pio, &jtag_shift_program, offset);
    pio_sm_unclaim(pio, sm);
    pio_close(pio);

    return 0;
}
