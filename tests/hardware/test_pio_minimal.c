/*
 * test_pio_minimal.c - Minimal PIO test using PIOLib directly
 *
 * Tests PIO at multiple levels:
 *   1. Non-DMA: pio_sm_put/get (direct FIFO access via ioctl)
 *   2. DMA: pio_sm_xfer_data (DMA transfer)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#ifdef HAVE_PIOLIB
#include "piolib.h"
#include "pio/generated/jtag_loopback.pio.h"

int main(void)
{
    printf("Minimal PIO test\n");
    printf("================\n\n");

    /* Step 1: Init and open PIO */
    if (pio_init() < 0) { printf("FAIL: pio_init\n"); return 1; }
    PIO pio = pio_open(0);
    if (PIO_IS_ERR(pio)) { printf("FAIL: pio_open\n"); return 1; }

    int sm = pio_claim_unused_sm(pio, true);
    if (sm < 0) { printf("FAIL: claim SM\n"); pio_close(pio); return 1; }
    printf("PIO opened, SM%d claimed\n", sm);

    /* Step 2: Load and configure loopback program */
    uint offset = pio_add_program(pio, &jtag_loopback_program);
    if (offset == PIO_ORIGIN_INVALID) {
        printf("FAIL: add_program\n");
        pio_sm_unclaim(pio, (uint)sm);
        pio_close(pio);
        return 1;
    }

    pio_select(pio);
    pio_sm_config c = jtag_loopback_program_get_default_config(offset);
    sm_config_set_out_shift(&c, true, true, 32);
    sm_config_set_in_shift(&c, true, true, 32);
    sm_config_set_clkdiv(&c, 1.0f);
    pio_sm_init(pio, (uint)sm, offset, &c);
    pio_sm_set_enabled(pio, (uint)sm, true);
    printf("Program loaded, SM enabled\n\n");

    /* Test A: Non-DMA FIFO access */
    printf("=== Test A: Non-DMA (pio_sm_put/get) ===\n");
    uint32_t test_val = 0xDEADBEEF;
    printf("  TX FIFO level before put: %u\n",
           pio_sm_get_tx_fifo_level(pio, (uint)sm));
    printf("  RX FIFO level before put: %u\n",
           pio_sm_get_rx_fifo_level(pio, (uint)sm));

    printf("  pio_sm_put_blocking(0x%08x)... ", test_val);
    pio_sm_put_blocking(pio, (uint)sm, test_val);
    printf("OK\n");

    printf("  TX FIFO level after put: %u\n",
           pio_sm_get_tx_fifo_level(pio, (uint)sm));
    printf("  RX FIFO level after put: %u\n",
           pio_sm_get_rx_fifo_level(pio, (uint)sm));

    printf("  pio_sm_get_blocking()... ");
    uint32_t rx_val = pio_sm_get_blocking(pio, (uint)sm);
    printf("0x%08x\n", rx_val);

    if (rx_val == test_val) {
        printf("  PASS: Non-DMA loopback works!\n\n");
    } else {
        printf("  FAIL: Expected 0x%08x, got 0x%08x\n\n", test_val, rx_val);
    }

    /* Test B: DMA transfer */
    printf("=== Test B: DMA (pio_sm_xfer_data) ===\n");

    /* Disable SM, reconfigure for DMA */
    pio_sm_set_enabled(pio, (uint)sm, false);
    pio_sm_restart(pio, (uint)sm);
    pio_sm_exec(pio, (uint)sm, pio_encode_jmp(offset));

    printf("  pio_sm_config_xfer(TX, 8)... ");
    errno = 0;
    int ret = pio_sm_config_xfer(pio, (uint)sm, PIO_DIR_TO_SM, 8, 1);
    printf("%s (ret=%d)\n", ret >= 0 ? "OK" : "FAIL", ret);
    if (ret < 0) { printf("  errno=%d: %s\n", errno, strerror(errno)); goto done; }

    printf("  pio_sm_config_xfer(RX, 8)... ");
    errno = 0;
    ret = pio_sm_config_xfer(pio, (uint)sm, PIO_DIR_FROM_SM, 8, 1);
    printf("%s (ret=%d)\n", ret >= 0 ? "OK" : "FAIL", ret);
    if (ret < 0) { printf("  errno=%d: %s\n", errno, strerror(errno)); goto done; }

    pio_sm_set_enabled(pio, (uint)sm, true);

    uint32_t tx_buf[2] = {0xCAFEBABE, 0x12345678};
    uint32_t rx_buf[2] = {0, 0};

    printf("  pio_sm_xfer_data(TX, 8)... ");
    errno = 0;
    ret = pio_sm_xfer_data(pio, (uint)sm, PIO_DIR_TO_SM, 8, tx_buf);
    printf("%s (ret=%d, errno=%d: %s)\n",
           ret >= 0 ? "OK" : "FAIL", ret, errno, strerror(errno));
    if (ret < 0) goto done;

    printf("  pio_sm_xfer_data(RX, 8)... ");
    errno = 0;
    ret = pio_sm_xfer_data(pio, (uint)sm, PIO_DIR_FROM_SM, 8, rx_buf);
    printf("%s (ret=%d, errno=%d: %s)\n",
           ret >= 0 ? "OK" : "FAIL", ret, errno, strerror(errno));
    if (ret < 0) goto done;

    printf("  RX = {0x%08x, 0x%08x}\n", rx_buf[0], rx_buf[1]);
    if (rx_buf[0] == tx_buf[0] && rx_buf[1] == tx_buf[1])
        printf("  PASS: DMA loopback works!\n");
    else
        printf("  FAIL: DMA data mismatch\n");

done:
    pio_sm_set_enabled(pio, (uint)sm, false);
    pio_remove_program(pio, &jtag_loopback_program, offset);
    pio_sm_unclaim(pio, (uint)sm);
    pio_close(pio);
    return 0;
}

#else

int main(void)
{
    printf("Compiled without PIOLib support\n");
    return 1;
}

#endif
