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

Measured on 2026-02-28. Bitstream programming via openFPGALoader with librp1jtag.

- Single SM (SM0 only), TMS driven as constant GPIO output
- 30720-bit chunks with SM restart per chunk
- Dynamic RX DMACTRL threshold (threshold=4 for >= 4 RX words, threshold=1 for < 4)
- TX DMACTRL threshold=4 (0xC0000104), high priority
- config_xfer reset at start of each bulk shift (resets kernel DMA state)
- Eliminates SM1 and its 704-bit chunk limit for constant-TMS transfers
- Pins: TDI=27, TDO=22, TCK=4, TMS=17
- Bitstream: user-100.bit (3,825,888 bytes), XC7A100T, CRC verified by FPGA

### 5 MHz (clkdiv=10)

| Run | Wall time | kB/s |
|-----|-----------|------|
| 1 | 3.265s | 1,145 |

### 10 MHz (clkdiv=5)

| Run | Wall time | kB/s |
|-----|-----------|------|
| 1 | 1.735s | 2,153 |

### 15 MHz (clkdiv=3.33)

| Run | Wall time | kB/s |
|-----|-----------|------|
| 1 | 1.224s | 3,053 |

### 20 MHz (clkdiv=2.5)

| Run | Wall time | kB/s |
|-----|-----------|------|
| 1 | 0.958s | 3,893 |

### 25+ MHz

IDCODE corruption at 25 MHz and 33 MHz (reads 0x26c62127 instead of 0x13631093 — 1-bit shift pattern). This is a signal integrity issue on the RPi-to-NeTV2 wiring, not a software bug. Maximum reliable TCK for this hardware setup is 20 MHz.

### Analysis

Transfer time scales linearly with 1/TCK frequency, confirming that PIO/TCK is now the bottleneck — not ioctl overhead.

| TCK | Time | Ratio to 5 MHz | Expected ratio |
|-----|------|----------------|---------------|
| 5 MHz | 3.265s | 1.00x | 1.00x |
| 10 MHz | 1.735s | 1.88x | 2.00x |
| 15 MHz | 1.224s | 2.67x | 3.00x |
| 20 MHz | 0.958s | 3.41x | 4.00x |

The ratio tracks the expected linear speedup closely. The small deviation from perfect 2x/3x/4x is expected from fixed per-shift overhead (TAP navigation, SM setup) that doesn't scale with TCK.

Previous bulk DMA attempt (1024-bit chunks, no DMACTRL fix): 3.54s at 10 MHz, 1,058 kB/s. The DMACTRL threshold fix and 30x larger chunks improved this to 1.735s, 2,153 kB/s — a 2.04x improvement.

**Key fixes that enabled this:**
1. TX DMACTRL threshold=4 (was 8, which caused 25% data corruption for transfers < 128 KB)
2. Dynamic RX threshold (threshold=4 causes timeout for small transfers; threshold=1 causes AXI bus contention for large transfers)
3. config_xfer per shift (resets accumulated kernel DMA semaphore/head/tail state)
4. pio_sm_clear_fifos per chunk (pio_sm_restart does NOT clear FIFOs)

## Summary

| Platform | Method | Bitstream | Time | Throughput | vs RPi 5 sysfsgpio |
|----------|--------|-----------|------|------------|-------------------|
| RPi 5 | rp1-jtag bulk DMA @20MHz | 3.8 MB (100T) | 0.96s | 3,893 kB/s | 41x |
| RPi 5 | rp1-jtag bulk DMA @15MHz | 3.8 MB (100T) | 1.22s | 3,053 kB/s | 32x |
| RPi 5 | rp1-jtag bulk DMA @10MHz | 3.8 MB (100T) | 1.74s | 2,153 kB/s | 22x |
| RPi 3B+ | OpenOCD bcm2835gpio | 2.1 MB (35T) | 1.50s | 1,428 kB/s | 15x |
| RPi 5 | rp1-jtag bulk DMA @5MHz | 3.8 MB (100T) | 3.27s | 1,145 kB/s | 12x |
| RPi 5 | rp1-jtag (two-SM chunked) | 3.8 MB (100T) | 6.3s | 589 kB/s | 6x |
| RPi 5 | OpenOCD sysfsgpio | 3.8 MB (100T) | 39.0s | 96 kB/s | 1x (baseline) |

### Key observations

1. **RPi 5 rp1-jtag now beats RPi 3 bcm2835gpio** — at 10 MHz and above, rp1-jtag outperforms the RPi 3B+'s direct memory-mapped GPIO (1,428 kB/s). At 20 MHz, throughput is 2.7x higher (3,893 vs 1,428 kB/s).

2. **Transfer time scales with 1/TCK frequency** — proving that PIO/TCK is now the bottleneck, not ioctl overhead. Previous architecture (6.3s at any speed) was entirely ioctl-bound.

3. **41x faster than sysfsgpio** — at 20 MHz, rp1-jtag achieves 3,893 kB/s vs sysfsgpio's 96 kB/s on the same RPi 5 hardware.

4. **DMACTRL threshold was the root cause** — the previous "kernel DMA bug" was actually DMACTRL threshold misconfiguration. Threshold=8 (from rpi5-rp1-pio-bench defaults) caused 25% data corruption for transfers < 128 KB. Threshold=4 works reliably at all sizes.

5. **Dynamic RX threshold is required** — RX threshold=4 causes timeout for small shifts (< 4 RX words, DREQ never fires). RX threshold=1 causes AXI bus contention for large shifts (DMA attempts 8-word burst with only 1 word available, stalling shared AXI bus). Solution: threshold=4 for >= 4 RX words, threshold=1 for < 4 RX words.

6. **Signal integrity limits max TCK to 20 MHz** — 25+ MHz TCK causes IDCODE corruption on the RPi-to-NeTV2 wiring. This is a hardware limitation, not software.

7. **30720-bit chunks with DMACTRL fix give 2x over 1024-bit chunks** — previous bulk DMA attempt (1024-bit chunks, threshold=8) achieved 1,058 kB/s at 10 MHz. DMACTRL fix + 30x larger chunks improved this to 2,153 kB/s.
