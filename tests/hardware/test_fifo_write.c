/*
 * test_fifo_write.c - Test FIFO write speed and correctness
 *
 * Uses pio_sm_put_blocking (no DMA) to feed data to the SM.
 * Measures throughput to determine if FIFO writes are fast enough
 * for write-only bitstream transfers.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include "piolib.h"
#include "pio/generated/jtag_shift.pio.h"

#define RP1_PIO_CLK_HZ  200000000
#define INSTR_PER_BIT   4

#define PIN_TDI 27
#define PIN_TDO 22
#define PIN_TCK 4
#define PIN_TMS 17

static double elapsed_ms(struct timespec *start, struct timespec *end)
{
    return (end->tv_sec - start->tv_sec) * 1000.0 +
           (end->tv_nsec - start->tv_nsec) / 1e6;
}

static int test_fifo_write(PIO pio, uint sm, uint offset,
                           uint32_t num_bits, uint32_t freq_hz)
{
    uint32_t num_data_words = (num_bits + 31) / 32;

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

    /* Start SM */
    pio_sm_exec(pio, sm, pio_encode_jmp(offset));
    pio_sm_set_enabled(pio, sm, true);

    struct timespec t_start, t_end;
    clock_gettime(CLOCK_MONOTONIC, &t_start);

    /* FIFO write: count word + data words */
    pio_sm_put_blocking(pio, sm, num_bits - 1);
    for (uint32_t i = 0; i < num_data_words; i++)
        pio_sm_put_blocking(pio, sm, 0xAAAAAAAA);

    /* Wait for SM completion */
    int timeout_us = 0;
    while (pio_sm_get_rx_fifo_level(pio, sm) == 0) {
        usleep(1);
        timeout_us++;
        if (timeout_us > 500000) {
            clock_gettime(CLOCK_MONOTONIC, &t_end);
            printf("TIMEOUT (%.1f ms)\n", elapsed_ms(&t_start, &t_end));
            pio_sm_set_enabled(pio, sm, false);
            return -1;
        }
    }

    clock_gettime(CLOCK_MONOTONIC, &t_end);
    double ms = elapsed_ms(&t_start, &t_end);
    double kbs = (num_bits / 8.0) / (ms / 1000.0) / 1024.0;
    uint32_t total_words = 1 + num_data_words;
    double us_per_word = (ms * 1000.0) / total_words;

    printf("%u bits (%u words): %.2f ms, %.1f kB/s, %.2f us/word\n",
           num_bits, total_words, ms, kbs, us_per_word);

    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    return 0;
}

int main(void)
{
    printf("FIFO Write Speed Test\n");
    printf("=====================\n\n");

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

    uint32_t freqs[] = {10000000, 20000000, 33000000};
    int nfreqs = sizeof(freqs) / sizeof(freqs[0]);

    uint32_t sizes[] = {224, 1024, 7680, 24544, 30720,
                        245440, 960000, 3840000};
    int nsizes = sizeof(sizes) / sizeof(sizes[0]);

    for (int f = 0; f < nfreqs; f++) {
        printf("\n=== %u MHz ===\n", freqs[f] / 1000000);
        for (int i = 0; i < nsizes; i++) {
            printf("  ");
            int rc = test_fifo_write(pio, sm, offset, sizes[i], freqs[f]);
            if (rc < 0) break;
        }
    }

    pio_sm_set_enabled(pio, sm, false);
    pio_remove_program(pio, &jtag_shift_program, offset);
    pio_sm_unclaim(pio, sm);
    pio_close(pio);
    return 0;
}
