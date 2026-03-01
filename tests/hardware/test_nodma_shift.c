/*
 * test_nodma_shift.c - Test jtag_shift PIO program WITHOUT DMA
 *
 * Uses pio_sm_put_blocking/get_blocking for direct FIFO access,
 * bypassing DMA entirely. This isolates whether data corruption
 * is in the PIO program or the DMA path.
 *
 * Tests the jtag_shift program with the actual FPGA BYPASS register
 * to verify the PIO program produces correct TDI/TDO at >32 bits.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>

#ifdef HAVE_PIOLIB
#include "piolib.h"
#include "pio/generated/jtag_shift.pio.h"

#define PIN_TCK 4
#define PIN_TMS 17
#define PIN_TDI 27
#define PIN_TDO 22

/* Feed count word + data words to TX FIFO, read RX FIFO.
 * All via put_blocking/get_blocking — NO DMA. */
static int shift_nodma(PIO pio, uint sm, uint32_t num_bits,
                       const uint32_t *tdi_words, uint32_t *tdo_words)
{
    uint32_t data_words = (num_bits + 31) / 32;
    uint32_t rx_words = num_bits / 32 + 1;  /* autopush + explicit push */

    /* Feed count word */
    pio_sm_put_blocking(pio, sm, num_bits - 1);

    /* Feed data words and drain RX concurrently.
     * The FIFO is 8 words deep, so we can feed and drain interleaved
     * to avoid FIFO overflow/underflow for larger transfers. */
    uint32_t tx_idx = 0, rx_idx = 0;

    while (tx_idx < data_words || rx_idx < rx_words) {
        /* Feed TX if FIFO not full and we have more data */
        if (tx_idx < data_words && !pio_sm_is_tx_fifo_full(pio, sm)) {
            pio_sm_put_blocking(pio, sm, tdi_words[tx_idx]);
            tx_idx++;
        }

        /* Drain RX if FIFO not empty */
        if (rx_idx < rx_words && !pio_sm_is_rx_fifo_empty(pio, sm)) {
            tdo_words[rx_idx] = pio_sm_get_blocking(pio, sm);
            rx_idx++;
        }
    }

    return 0;
}

/* Navigate TAP by clocking TMS bits via put_blocking.
 * TMS is controlled as GPIO, so we set it before each shift. */
static void set_tms(PIO pio, uint sm_tms, int val) {
    uint32_t mask = 1u << PIN_TMS;
    pio_sm_set_pins_with_mask(pio, sm_tms, val ? mask : 0, mask);
}

/* Single-bit shift via non-DMA for TAP navigation */
static void tap_clock(PIO pio, uint sm0, uint offset, uint sm_tms,
                      int tms_val, int tdi_val)
{
    set_tms(pio, sm_tms, tms_val);

    /* Restart SM for each tiny shift */
    pio_sm_set_enabled(pio, sm0, false);
    pio_sm_clear_fifos(pio, sm0);
    pio_sm_restart(pio, sm0);
    pio_sm_exec(pio, sm0, pio_encode_jmp(offset));
    pio_sm_set_enabled(pio, sm0, true);

    /* count = 0 (1 bit), data = tdi_val */
    pio_sm_put_blocking(pio, sm0, 0);  /* count = 0 → 1 bit */
    pio_sm_put_blocking(pio, sm0, tdi_val ? 0xFFFFFFFF : 0);
    /* Read 1 RX word (explicit push only, since 1 bit < 32 threshold) */
    (void)pio_sm_get_blocking(pio, sm0);
}

/* Multi-bit shift for TAP navigation */
static void tap_shift(PIO pio, uint sm0, uint offset, uint sm_tms,
                      int tms_val, uint32_t num_bits, uint32_t tdi_val)
{
    set_tms(pio, sm_tms, tms_val);

    pio_sm_set_enabled(pio, sm0, false);
    pio_sm_clear_fifos(pio, sm0);
    pio_sm_restart(pio, sm0);
    pio_sm_exec(pio, sm0, pio_encode_jmp(offset));
    pio_sm_set_enabled(pio, sm0, true);

    pio_sm_put_blocking(pio, sm0, num_bits - 1);
    uint32_t data_words = (num_bits + 31) / 32;
    for (uint32_t i = 0; i < data_words; i++)
        pio_sm_put_blocking(pio, sm0, tdi_val);
    uint32_t rx_words = num_bits / 32 + 1;
    for (uint32_t i = 0; i < rx_words; i++)
        (void)pio_sm_get_blocking(pio, sm0);
}

int main(void)
{
    printf("JTAG Shift Non-DMA Diagnostic Test\n");
    printf("===================================\n\n");

    if (pio_init() < 0) { printf("FAIL: pio_init\n"); return 1; }
    PIO pio = pio_open(0);
    if (PIO_IS_ERR(pio)) { printf("FAIL: pio_open\n"); return 1; }

    int sm0 = pio_claim_unused_sm(pio, true);
    int sm1 = pio_claim_unused_sm(pio, true);
    if (sm0 < 0 || sm1 < 0) {
        printf("FAIL: claim SMs\n"); pio_close(pio); return 1;
    }

    /* Load jtag_shift program */
    uint offset = pio_add_program(pio, &jtag_shift_program);
    if (offset == PIO_ORIGIN_INVALID) {
        printf("FAIL: add_program\n"); pio_close(pio); return 1;
    }

    pio_select(pio);

    /* Configure SM0: jtag_shift */
    pio_sm_config c0 = jtag_shift_program_get_default_config(offset);
    sm_config_set_out_pins(&c0, PIN_TDI, 1);
    sm_config_set_in_pins(&c0, PIN_TDO);
    sm_config_set_sideset_pins(&c0, PIN_TCK);
    sm_config_set_out_shift(&c0, true, true, 32);
    sm_config_set_in_shift(&c0, true, true, 32);
    float div = 200000000.0f / (10000000.0f * 4.0f);  /* 10 MHz TCK */
    sm_config_set_clkdiv(&c0, div);
    pio_sm_init(pio, (uint)sm0, offset, &c0);

    pio_gpio_init(pio, PIN_TCK);
    pio_gpio_init(pio, PIN_TDI);
    pio_gpio_init(pio, PIN_TDO);
    pio_sm_set_consecutive_pindirs(pio, (uint)sm0, PIN_TCK, 1, true);
    pio_sm_set_consecutive_pindirs(pio, (uint)sm0, PIN_TDI, 1, true);
    pio_sm_set_consecutive_pindirs(pio, (uint)sm0, PIN_TDO, 1, false);

    /* Configure SM1 for TMS output */
    pio_sm_config c1 = pio_get_default_sm_config();
    sm_config_set_out_pins(&c1, PIN_TMS, 1);
    pio_sm_init(pio, (uint)sm1, 0, &c1);
    pio_gpio_init(pio, PIN_TMS);
    pio_sm_set_consecutive_pindirs(pio, (uint)sm1, PIN_TMS, 1, true);

    printf("PIO configured: SM%d (jtag_shift), SM%d (TMS)\n", sm0, sm1);
    printf("FIFO depth: %u\n\n", pio_get_fifo_depth(pio));

    /* === TAP Navigation: Reset → RTI → BYPASS → Shift-DR === */
    printf("=== TAP Navigation (non-DMA) ===\n");

    /* TLR: 6 clocks with TMS=1 */
    for (int i = 0; i < 6; i++)
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
    printf("  TLR done\n");

    /* TLR → RTI: TMS=0 */
    tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);
    printf("  RTI done\n");

    /* Read IDCODE: RTI → Shift-DR */
    tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);  /* Select-DR */
    tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);  /* Capture-DR */
    tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);  /* Shift-DR */

    /* Shift 32 bits from DR (IDCODE is default after TLR) */
    set_tms(pio, (uint)sm1, 0);
    pio_sm_set_enabled(pio, (uint)sm0, false);
    pio_sm_clear_fifos(pio, (uint)sm0);
    pio_sm_restart(pio, (uint)sm0);
    pio_sm_exec(pio, (uint)sm0, pio_encode_jmp(offset));
    pio_sm_set_enabled(pio, (uint)sm0, true);

    pio_sm_put_blocking(pio, (uint)sm0, 31);  /* 32 bits */
    pio_sm_put_blocking(pio, (uint)sm0, 0);   /* TDI=0 */
    uint32_t idcode_word = pio_sm_get_blocking(pio, (uint)sm0);
    (void)pio_sm_get_blocking(pio, (uint)sm0);  /* explicit push */
    printf("  IDCODE: 0x%08x", idcode_word);
    if (idcode_word == 0x13631093) printf(" (XC7A100T)\n");
    else printf(" (unexpected!)\n");

    /* Exit Shift-DR: TMS=1 */
    tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
    /* Update-DR → RTI */
    tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
    tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);

    /* Load BYPASS instruction */
    printf("  Loading BYPASS IR...\n");
    /* RTI → Select-DR → Select-IR → Capture-IR → Shift-IR */
    tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
    tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
    tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);
    tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);

    /* Shift 6-bit BYPASS=0x3F with last bit TMS=1 */
    /* Bits 0-4: TMS=0, bit 5: TMS=1 */
    for (int i = 0; i < 5; i++)
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 1);
    tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 1);  /* last bit */
    /* Exit1-IR → Update-IR → RTI */
    tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
    tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);
    printf("  BYPASS loaded\n");

    /* === BYPASS loopback test at various sizes === */
    printf("\n=== BYPASS Loopback (non-DMA shift_nodma) ===\n");
    printf("This uses put_blocking/get_blocking, NO DMA.\n\n");

    uint32_t sizes[] = {32, 48, 63, 64, 96, 128, 256};
    int nsizes = sizeof(sizes) / sizeof(sizes[0]);

    for (int t = 0; t < nsizes; t++) {
        uint32_t num_bits = sizes[t];
        uint32_t data_words = (num_bits + 31) / 32;
        uint32_t rx_words = num_bits / 32 + 1;

        uint32_t *tdi_words = calloc(data_words, sizeof(uint32_t));
        uint32_t *tdo_words = calloc(rx_words, sizeof(uint32_t));
        if (!tdi_words || !tdo_words) { free(tdi_words); free(tdo_words); break; }

        /* Fill TDI with all-ones */
        for (uint32_t i = 0; i < data_words; i++)
            tdi_words[i] = 0xFFFFFFFF;
        /* Mask unused bits in last word */
        if (num_bits % 32)
            tdi_words[data_words - 1] &= (1u << (num_bits % 32)) - 1;

        /* Navigate to Shift-DR */
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);  /* Select-DR */
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);  /* Capture-DR */
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);  /* Shift-DR */

        /* Shift data with TMS=0 (stay in Shift-DR) */
        set_tms(pio, (uint)sm1, 0);

        pio_sm_set_enabled(pio, (uint)sm0, false);
        pio_sm_clear_fifos(pio, (uint)sm0);
        pio_sm_restart(pio, (uint)sm0);
        pio_sm_exec(pio, (uint)sm0, pio_encode_jmp(offset));
        pio_sm_set_enabled(pio, (uint)sm0, true);

        shift_nodma(pio, (uint)sm0, num_bits, tdi_words, tdo_words);

        /* Fix partial word alignment (right-shift ISR) */
        uint32_t remainder = num_bits % 32;
        if (remainder != 0)
            tdo_words[data_words - 1] >>= (32 - remainder);

        printf("  %3u bits: TDO words:", num_bits);
        for (uint32_t i = 0; i < data_words; i++)
            printf(" 0x%08x", tdo_words[i]);
        printf("\n");

        /* Verify BYPASS: TDO[0]=0, TDO[i]=TDI[i-1] */
        int errors = 0;
        if (tdo_words[0] & 1) errors++;
        for (uint32_t i = 1; i < num_bits; i++) {
            int expected = (tdi_words[(i-1)/32] >> ((i-1)%32)) & 1;
            int actual = (tdo_words[i/32] >> (i%32)) & 1;
            if (actual != expected) errors++;
        }

        if (errors == 0)
            printf("           PASS\n");
        else
            printf("           FAIL (%d errors)\n", errors);

        /* Exit Shift-DR → RTI */
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);  /* Exit1-DR */
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);  /* Update-DR */
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);  /* RTI */

        /* Re-enter BYPASS */
        for (int i = 0; i < 6; i++)
            tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);
        /* Load BYPASS */
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);
        for (int i = 0; i < 5; i++)
            tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 1);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 1);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);

        free(tdi_words);
        free(tdo_words);
    }

    /* === Test: DMA TX + non-DMA RX === */
    printf("\n=== BYPASS Loopback (DMA TX, non-DMA RX) ===\n");
    printf("If PASS: TX DMA works, problem is RX DMA.\n");
    printf("If FAIL: TX DMA is broken.\n\n");

    /* Need to configure DMA for TX */
    int dma_ret;
    dma_ret = pio_sm_config_xfer(pio, (uint)sm0, PIO_DIR_TO_SM, 4096, 1);
    if (dma_ret < 0) {
        printf("FAIL: config_xfer TX: %d\n", dma_ret);
        goto cleanup;
    }
    pio_sm_set_dmactrl(pio, (uint)sm0, true, 0xC0000104);  /* threshold=4 */

    for (int t = 0; t < nsizes; t++) {
        uint32_t num_bits = sizes[t];
        uint32_t data_words = (num_bits + 31) / 32;
        uint32_t rx_words = num_bits / 32 + 1;

        uint32_t *tdi_words = calloc(1 + data_words, sizeof(uint32_t));
        uint32_t *tdo_words = calloc(rx_words, sizeof(uint32_t));
        if (!tdi_words || !tdo_words) { free(tdi_words); free(tdo_words); break; }

        /* TX buffer: count word + data words */
        tdi_words[0] = num_bits - 1;
        for (uint32_t i = 1; i <= data_words; i++)
            tdi_words[i] = 0xFFFFFFFF;
        if (num_bits % 32)
            tdi_words[data_words] &= (1u << (num_bits % 32)) - 1;

        size_t tx_bytes = (1 + data_words) * sizeof(uint32_t);
        if (tx_bytes < 8) tx_bytes = 8;

        /* Navigate to Shift-DR */
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);

        set_tms(pio, (uint)sm1, 0);

        pio_sm_set_enabled(pio, (uint)sm0, false);
        pio_sm_clear_fifos(pio, (uint)sm0);
        pio_sm_restart(pio, (uint)sm0);
        pio_sm_exec(pio, (uint)sm0, pio_encode_jmp(offset));
        pio_sm_set_enabled(pio, (uint)sm0, true);

        /* TX via DMA */
        dma_ret = pio_sm_xfer_data(pio, (uint)sm0, PIO_DIR_TO_SM,
                                   tx_bytes, tdi_words);
        if (dma_ret < 0) {
            printf("  %3u bits: TX DMA failed: %d\n", num_bits, dma_ret);
            free(tdi_words); free(tdo_words);
            /* Reset TAP */
            for (int i = 0; i < 6; i++)
                tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
            tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);
            tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
            tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
            tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);
            tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);
            for (int i = 0; i < 5; i++)
                tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 1);
            tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 1);
            tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
            tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);
            continue;
        }

        /* RX via non-DMA (get_blocking) */
        for (uint32_t i = 0; i < rx_words; i++)
            tdo_words[i] = pio_sm_get_blocking(pio, (uint)sm0);

        /* Fix partial word alignment */
        uint32_t remainder = num_bits % 32;
        if (remainder != 0)
            tdo_words[data_words - 1] >>= (32 - remainder);

        printf("  %3u bits: TDO words:", num_bits);
        for (uint32_t i = 0; i < data_words; i++)
            printf(" 0x%08x", tdo_words[i]);
        printf("\n");

        int errors = 0;
        uint32_t exp0 = 0xfffffffe;
        if (num_bits < 32) exp0 = (0xFFFFFFFF << 1) & ((1u << num_bits) - 1);
        if (tdo_words[0] != exp0 && num_bits >= 32) errors++;
        for (uint32_t i = 1; i < num_bits; i++) {
            int expected = (num_bits >= 32) ? 1 : ((i > 0) ? 1 : 0);
            /* All-ones TDI: TDO[0]=0, TDO[i]=1 for i>0 */
            if (i == 0) expected = 0;
            else expected = 1;
            int actual = (tdo_words[i/32] >> (i%32)) & 1;
            if (actual != expected) errors++;
        }

        if (errors == 0)
            printf("           PASS\n");
        else
            printf("           FAIL (%d errors)\n", errors);

        /* Exit Shift-DR → RTI → BYPASS */
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);
        for (int i = 0; i < 6; i++)
            tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);
        for (int i = 0; i < 5; i++)
            tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 1);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 1);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);

        free(tdi_words);
        free(tdo_words);
    }

    /* === Test: non-DMA TX + DMA RX === */
    printf("\n=== BYPASS Loopback (non-DMA TX, DMA RX) ===\n");
    printf("If PASS: RX DMA works, problem is TX DMA.\n");
    printf("If FAIL: RX DMA is broken.\n\n");

    /* Configure DMA for RX */
    dma_ret = pio_sm_config_xfer(pio, (uint)sm0, PIO_DIR_FROM_SM, 4096, 1);
    if (dma_ret < 0) {
        printf("FAIL: config_xfer RX: %d\n", dma_ret);
        goto cleanup;
    }
    pio_sm_set_dmactrl(pio, (uint)sm0, false, 0xC0000101);  /* threshold=1 */

    for (int t = 0; t < nsizes; t++) {
        uint32_t num_bits = sizes[t];
        uint32_t data_words = (num_bits + 31) / 32;
        uint32_t rx_words = num_bits / 32 + 1;

        uint32_t *tdi_words = calloc(data_words, sizeof(uint32_t));
        uint32_t *tdo_words = calloc(rx_words, sizeof(uint32_t));
        if (!tdi_words || !tdo_words) { free(tdi_words); free(tdo_words); break; }

        for (uint32_t i = 0; i < data_words; i++)
            tdi_words[i] = 0xFFFFFFFF;
        if (num_bits % 32)
            tdi_words[data_words - 1] &= (1u << (num_bits % 32)) - 1;

        size_t rx_bytes = rx_words * sizeof(uint32_t);
        if (rx_bytes < 8) rx_bytes = 8;

        /* Navigate to Shift-DR */
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);

        set_tms(pio, (uint)sm1, 0);

        /* Need to re-configure DMA before each transfer */
        dma_ret = pio_sm_config_xfer(pio, (uint)sm0, PIO_DIR_FROM_SM, 4096, 1);
        if (dma_ret < 0) {
            printf("  %3u bits: config_xfer RX failed\n", num_bits);
            free(tdi_words); free(tdo_words); break;
        }
        pio_sm_set_dmactrl(pio, (uint)sm0, false, 0xC0000101);

        pio_sm_set_enabled(pio, (uint)sm0, false);
        pio_sm_clear_fifos(pio, (uint)sm0);
        pio_sm_restart(pio, (uint)sm0);
        pio_sm_exec(pio, (uint)sm0, pio_encode_jmp(offset));
        pio_sm_set_enabled(pio, (uint)sm0, true);

        /* TX via non-DMA: count word + data words */
        pio_sm_put_blocking(pio, (uint)sm0, num_bits - 1);

        /* Start RX DMA in background (need a thread) */
        /* Actually, let's feed TX first since FIFO can hold 8 words,
         * then start RX DMA. For 256 bits (8 data words + 1 count = 9),
         * FIFO will fill up. We need interleaving.
         *
         * For simplicity, let's limit to sizes that fit in the 8-word FIFO:
         * count + data_words <= 8 → data_words <= 7 → num_bits <= 224 */
        if (1 + data_words <= 8) {
            /* Can fit everything in TX FIFO */
            for (uint32_t i = 0; i < data_words; i++)
                pio_sm_put_blocking(pio, (uint)sm0, tdi_words[i]);

            /* RX via DMA */
            memset(tdo_words, 0, rx_bytes);
            dma_ret = pio_sm_xfer_data(pio, (uint)sm0, PIO_DIR_FROM_SM,
                                       rx_bytes, tdo_words);
            if (dma_ret < 0) {
                printf("  %3u bits: RX DMA failed: %d\n", num_bits, dma_ret);
            } else {
                uint32_t remainder = num_bits % 32;
                if (remainder != 0)
                    tdo_words[data_words - 1] >>= (32 - remainder);

                printf("  %3u bits: TDO words:", num_bits);
                for (uint32_t i = 0; i < data_words; i++)
                    printf(" 0x%08x", tdo_words[i]);
                printf("\n");

                int errors = 0;
                for (uint32_t i = 1; i < num_bits; i++) {
                    int expected = 1;
                    int actual = (tdo_words[i/32] >> (i%32)) & 1;
                    if (actual != expected) errors++;
                }
                if (tdo_words[0] & 1) errors++;  /* bit 0 should be 0 */

                if (errors == 0)
                    printf("           PASS\n");
                else
                    printf("           FAIL (%d errors)\n", errors);
            }
        } else {
            printf("  %3u bits: SKIP (too large for non-DMA TX)\n", num_bits);
        }

        /* Exit + reset + BYPASS */
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);
        for (int i = 0; i < 6; i++)
            tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);
        for (int i = 0; i < 5; i++)
            tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 1);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 1);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 1, 0);
        tap_clock(pio, (uint)sm0, offset, (uint)sm1, 0, 0);

        free(tdi_words);
        free(tdo_words);
    }

cleanup:
    pio_sm_set_enabled(pio, (uint)sm0, false);
    pio_remove_program(pio, &jtag_shift_program, offset);
    pio_sm_unclaim(pio, (uint)sm1);
    pio_sm_unclaim(pio, (uint)sm0);
    pio_close(pio);
    return 0;
}

#else

int main(void)
{
    printf("Compiled without PIOLib support (HAVE_PIOLIB not defined)\n");
    return 1;
}

#endif
