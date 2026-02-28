# Bulk DMA Performance Optimization

## Problem Statement

Programming the NeTV2's XC7A100T Artix-7 FPGA (3,825,888-byte bitstream) via
rp1-jtag currently takes **6.3 seconds** at **589 kB/s**. This is slower than
the RPi 3B+ at 1,428 kB/s via bcm2835gpio, despite the RPi 5 having a far
more capable PIO subsystem.

TCK frequency (6/10/20 MHz) has zero measurable effect on transfer time,
proving that PIO execution speed is not the bottleneck.

## Root Cause Analysis

The library uses two PIO state machines (SM0 for TDI/TDO/TCK, SM1 for TMS)
with a **704-bit chunk limit** forced by SM1 TX DMA timeouts. This results in:

- **43,475 chunks** per 3.8 MB bitstream (3,825,888 * 8 / 704)
- **~11 ioctls per chunk** (config_xfer * 3 + xfer_data * 3 + SM control * 5)
- **~478,000 total ioctls** per bitstream
- At ~13 us per ioctl, **ioctl overhead alone is ~6.2 seconds**

This explains why changing TCK frequency has no effect: the PIO state machine
is idle >99% of the time, waiting for the next DMA ioctl to feed it data.

## Architecture Decision

During bitstream programming, TMS is constant (all zeros — staying in
Shift-DR state). SM1 (TMS via DMA) is entirely unnecessary. TMS can be
driven as a constant GPIO output via `pio_sm_set_pins_with_mask`, eliminating
SM1 and its DMA limitations entirely.

This enables **single-SM bulk DMA** transfers following the architecture
proven by the rpi5-rp1-pio-bench benchmark (42 MB/s aggregate throughput):

- `pio_sm_config_xfer(256 KB, buf_count=1)` for SM0 TX and RX
- SM0 enabled once, two concurrent pthreads (TX loop + RX loop)
- Each thread calls `pio_sm_xfer_data` in a loop with 256 KB chunks

Two transfer paths, selected automatically:

| Path | When | SMs | DMA buffer | Chunks for 3.8 MB |
|------|------|-----|------------|-------------------|
| **Bulk** (new) | TMS constant | SM0 only | 256 KB | ~15 |
| **Chunked** (existing) | TMS varies | SM0 + SM1 | 4 KB/256 B | 43,475 |

Expected ioctls per bitstream: ~30 (15 TX + 15 RX) vs current ~478,000.

## Performance Targets

- **Target 1**: Transfer time scales with 1/TCK_frequency (proves ioctl
  overhead eliminated)
- **Target 2**: At 33 MHz TCK, bitstream load < 1.5 seconds (3.8 MB, XC7A100T)
- **Target 3**: At 33 MHz TCK, throughput > 2,500 kB/s (beating RPi 3B+
  bcm2835gpio at 1,428 kB/s)
- **Target 4**: At 10 MHz TCK, bitstream load ~3.0 seconds (proving TCK is
  bottleneck, not ioctl)

## Prerequisites

Requires kernel 6.12+ with:
- **PR #6994**: Heavy DMA channel reservation for SM0
- **PR #7190**: FIFO threshold fix for large transfers

Verified: rpi5-netv2 runs kernel 6.12.47 (satisfies requirement).

## Reference

- rpi5-rp1-pio-bench benchmark: 42 MB/s aggregate throughput using single SM
  with 256 KB DMA transfers and only 2 ioctls per transfer
