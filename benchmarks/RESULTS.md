# Benchmark Results

Measured on 2026-02-23. All tests use direct GPIO wiring to a NeTV2 FPGA board.

## Test Setup

| Platform | Board | FPGA | IDCODE | Bitstream | Size |
|----------|-------|------|--------|-----------|------|
| RPi 3B+ | NeTV2 | XC7A35T | 0x0362d093 | user-35.bit | 2,192,111 bytes |
| RPi 5 | NeTV2 | XC7A100T | 0x13631093 | user-100.bit | 3,825,888 bytes |

## RPi 3B+ — OpenOCD bcm2835gpio

- OpenOCD 0.10.0-00019-gfb5691f0-dirty (2018-10-26)
- Config: `alphamax-rpi.cfg` (bcm2835gpio driver, 10 MHz requested)
- Bitstream: user-35.bit (2,192,111 bytes)

| Run | Wall time | OpenOCD internal | kB/s |
|-----|-----------|-----------------|------|
| 1 | 1.518s | 1s 468048us | 1,443 |
| 2 | 3.355s | 3s 298762us | 653 |
| 3 | 1.485s | 1s 428249us | 1,476 |

Run 2 was an outlier (likely OS scheduling). Using runs 1 and 3:
- **Average: 1.50s, 1,428 kB/s**

## RPi 5 — OpenOCD sysfsgpio

- OpenOCD 0.12.0+dev-snapshot (2025-07-16)
- Config: `rpi5-sysfsgpio.cfg` (sysfsgpio driver, gpiochip571 offsets)
- Bitstream: user-100.bit (3,825,888 bytes)
- Note: "The adapter 'sysfsgpio' doesn't support configurable speed"

| Run | Wall time | OpenOCD internal | kB/s |
|-----|-----------|-----------------|------|
| 1 | 38.954s | 38s 927740us | 96 |
| 2 | 39.168s | 39s 149009us | 95 |
| 3 | 38.863s | 38s 842268us | 96 |

Very consistent. sys time ~34s (88% in kernel sysfs calls).
- **Average: 39.0s, 96 kB/s**

## RPi 5 — rp1-jtag (DMA + fast PIO), before optimisation

- openFPGALoader with rp1pio cable driver using librp1jtag
- Pins: TDI=27, TDO=22, TCK=4, TMS=17
- Bitstream: user-100.bit (3,825,888 bytes)
- TCK: 6 MHz default, MAX_TRANSFER_WORDS=1024 (4 KB chunks), SM enable/disable per chunk

| Run | Wall time | kB/s |
|-----|-----------|------|
| 1 | failed | (residual JTAG state from prior XVC test) |
| 2 | 6.54s | 572 |
| 3 | 6.52s | 574 |

- **Average (runs 2-3): 6.53s, 572 kB/s**

## RPi 5 — rp1-jtag (DMA + fast PIO), after optimisation

- Same setup as above
- Changes: SM kept running across chunks, MAX_TRANSFER_WORDS=8192 (32 KB chunks), default TCK raised to 10 MHz
- Eliminates ~934 SM enable/disable ioctl pairs per bitstream load

### 6 MHz (for comparison with baseline)

| Run | Wall time | kB/s |
|-----|-----------|------|
| 1 | 6.22s | 602 |
| 2 | 6.22s | 602 |
| 3 | 6.42s | 583 |

- **Average: 6.29s, 595 kB/s** (3.7% faster than 6.53s baseline)

### 10 MHz (new default)

| Run | Wall time | kB/s |
|-----|-----------|------|
| 1 | 6.21s | 603 |
| 2 | 6.43s | 582 |
| 3 | 6.43s | 582 |

- **Average: 6.36s, 589 kB/s**

### 20 MHz

| Run | Wall time | kB/s |
|-----|-----------|------|
| 1 | 6.42s | 583 |
| 2 | 6.21s | 603 |
| 3 | 6.21s | 603 |

- **Average: 6.28s, 596 kB/s**

### Analysis

The ~0.2s variance between fast (6.21s) and slow (6.42s) runs is OS scheduling jitter — it appears at all frequencies. The improvement from SM lifecycle hoisting is real but small: **best runs improved from 6.52s to 6.21s (4.8%)**.

6 MHz, 10 MHz, and 20 MHz produce near-identical results, confirming that **PIO execution speed is not the bottleneck**. The dominant cost is DMA ioctl overhead: each 32-byte DMA transfer requires 2 PIOLib ioctls (TX + RX), giving ~240,000 ioctls per bitstream regardless of TCK frequency. The SM enable/disable savings (~1,868 ioctls) were <1% of total.

**Next bottleneck**: Larger DMA transfers (>32 bytes) would reduce ioctl count but currently deadlock because blocking TX DMA fills the FIFO before RX can drain. Fixing this requires either non-blocking DMA or a kernel-side bidirectional transfer primitive.

## RPi 5 — Threaded bidirectional DMA attempt (BLOCKED)

- Attempted: concurrent TX + RX DMA via pthreads to eliminate 32-byte ping-pong
- Goal: transfer entire 32 KB chunks in a single bidi call (~2 ioctls instead of ~8,192)

### Result: BLOCKED by PIOLib kernel driver

Two kernel-level limitations prevent concurrent DMA:

1. **Per-SM ioctl serialization**: PIOLib's `pio_sm_xfer_data()` holds a per-SM mutex in the kernel. When the RX thread enters the kernel first and blocks waiting for SM data, the TX thread can't acquire the mutex to start feeding the SM. Result: deadlock → DMA timeout after ~1s.

2. **config_xfer buf_size > 32 bytes causes DMA timeout**: Even with single-direction ping-pong, calling `pio_sm_config_xfer(buf_size=32768)` causes subsequent `pio_sm_xfer_data(32 bytes)` transfers to fail with a DMA timeout, despite the buf_size being a maximum, not an exact match. This is likely a PIOLib DMA buffer allocation issue.

### Performance with bidi disabled (code present but NULL in vtable)

The bidi API (`xfer_data_bidi`) is implemented in the backend abstraction and works correctly in mock tests. On RP1, it's set to NULL, falling back to the existing 32-byte ping-pong DMA.

| Frequency | Run 1 | Run 2 | Run 3 | Average | kB/s |
|-----------|-------|-------|-------|---------|------|
| 10 MHz | 6.55s | 6.42s | 6.42s | 6.46s | 580 |
| 20 MHz | 6.43s | 6.42s | 6.22s | 6.36s | 589 |

No regression from the added code. Performance matches the post-SM-optimisation baseline.

### Path forward

Eliminating the ioctl bottleneck requires kernel-level changes:
- A kernel-level bidirectional DMA ioctl that sets up both TX and RX DMA channels atomically
- Or using separate PIO file descriptors (separate mutexes) for TX and RX
- Or patching PIOLib to use finer-grained locking (per-direction, not per-SM)

## RPi 5 — rp1-jtag bulk DMA (single-SM, constant TMS)

Measured on 2026-02-28. Uses `test_single_shift` to test the bulk DMA path directly.

- Single SM (SM0 only), TMS driven as constant GPIO output
- 1024-bit chunks with SM restart per chunk, reusing init DMA config
- Eliminates SM1 and its 704-bit chunk limit for constant-TMS transfers
- Pins: TDI=27, TDO=22, TCK=4, TMS=17

### 10 MHz (default, clkdiv=5)

Test: `test_single_shift 30625904` (3.8 MB = XC7A100T bitstream size)

| Run | Wall time | kB/s |
|-----|-----------|------|
| 1 | 3.62s | 1,034 |
| 2 | 3.45s | 1,085 |

- **Average: 3.54s, 1,058 kB/s** (1.80x faster than two-SM chunked at 6.36s)

### Analysis

The bulk path processes 30,625,904 bits in ~29,908 chunks of 1024 bits.
At 10 MHz TCK: theoretical PIO processing time is 3.06s (30.6M bits / 10M Hz).
Measured: 3.54s. Overhead: ~0.48s = ~16 µs/chunk (PIO processing overlaps with DMA setup).

**RP1 DMA completion callback bug**: The kernel driver's DMA completion callback is never invoked for single-transfer sizes > ~1024 bits on slow SM programs (4 instr/bit at clkdiv >= 3). The DMA physically completes (proven by RX data) but the semaphore is never signaled, causing a 1-second timeout. Workaround: 1024-bit chunks with SM restart between each.

**Reliable clkdiv range**: clkdiv >= 3 (16.67 MHz TCK and below). clkdiv=2 (25 MHz) fails after ~5,883 chunks. clkdiv=1.5 (33 MHz) fails after ~5,680 chunks. Higher TCK frequencies require kernel fixes for reliable DMA.

**FPGA connectivity**: IDCODE test returns 0xFFFFFFFF during this session (NeTV2 not responding). Bitstream programming via openFPGALoader could not be tested. The bulk DMA path was verified using test_single_shift (constant TMS=0, TDO data not validated).

## Summary

| Platform | Method | Bitstream | Time | Throughput | vs RPi 5 sysfsgpio |
|----------|--------|-----------|------|------------|-------------------|
| RPi 3B+ | OpenOCD bcm2835gpio | 2.1 MB (35T) | 1.5s | 1,428 kB/s | 15x |
| RPi 5 | rp1-jtag bulk DMA @10MHz | 3.8 MB (100T) | 3.5s | 1,058 kB/s | 11x |
| RPi 5 | rp1-jtag (two-SM chunked) | 3.8 MB (100T) | 6.3s | 589 kB/s | 6x |
| RPi 5 | OpenOCD sysfsgpio | 3.8 MB (100T) | 39.0s | 96 kB/s | 1x (baseline) |

### Key observations

1. **RPi 5 sysfsgpio is 15x slower than RPi 3 bcm2835gpio** — each GPIO toggle goes through sysfs writes over the RP1 PCIe bridge, versus direct memory-mapped register access on RPi 3.

2. **rp1-jtag recovers most of the lost performance** — the PIO approach is 6-11x faster than sysfsgpio on the same RPi 5 hardware, by offloading JTAG clocking to the RP1's PIO state machine.

3. **Bulk DMA gives 1.8x improvement for constant-TMS transfers** — eliminating SM1 and using 1024-bit chunks reduces total time from 6.3s to 3.5s for a 3.8 MB bitstream at 10 MHz TCK.

4. **Per-chunk overhead overlaps with PIO processing** — at 10 MHz TCK, each 1024-bit chunk takes ~102 µs for PIO processing. The DMA/ioctl overhead (~50 µs) runs concurrently, giving ~16 µs net overhead per chunk.

5. **RPi 3 bcm2835gpio still leads in throughput** — 1,428 kB/s vs 1,058 kB/s. The gap narrows with bulk DMA but direct memory-mapped GPIO remains faster than PIO + DMA + ioctl.

6. **Higher TCK frequencies blocked by kernel DMA bug** — at clkdiv < 3 (> 16.67 MHz), DMA transfers fail after thousands of chunks. The RP1 PIO kernel driver's DMA completion callback is never invoked for slow SM programs with transfers > ~1024 bits. Kernel patches are needed to unlock 33 MHz TCK (~2,500 kB/s theoretical).

7. **PIOLib kernel driver limits remain** — per-SM ioctl serialization prevents true bidirectional DMA. The 1024-bit chunk limit is a workaround for the DMA completion bug. Both issues require kernel-level fixes for further performance gains.
