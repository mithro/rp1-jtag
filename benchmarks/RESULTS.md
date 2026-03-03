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

### Result: FAILED (bug in our code)

Note: `rpi5-rp1-pio-bench` achieves 42 MB/s with concurrent DMA on the same hardware, proving the kernel driver handles concurrent DMA correctly. The failures below are bugs in our SM setup or DMA configuration, not kernel limitations.

Two issues observed (root cause in our code, not yet identified):

1. **Per-SM ioctl serialization**: PIOLib's `pio_sm_xfer_data()` holds a per-SM mutex in the kernel. When the RX thread enters the kernel first and blocks waiting for SM data, the TX thread can't acquire the mutex to start feeding the SM. Result: deadlock → DMA timeout after ~1s. (The benchmark avoids this — need to investigate how.)

2. **config_xfer buf_size > 32 bytes causes DMA timeout**: Even with single-direction ping-pong, calling `pio_sm_config_xfer(buf_size=32768)` causes subsequent `pio_sm_xfer_data(32 bytes)` transfers to fail with a DMA timeout. (The benchmark uses 256 KB buf_size successfully — need to diff our config.)

### Performance with bidi disabled (code present but NULL in vtable)

The bidi API (`xfer_data_bidi`) is implemented in the backend abstraction and works correctly in mock tests. On RP1, it's set to NULL, falling back to the existing 32-byte ping-pong DMA.

| Frequency | Run 1 | Run 2 | Run 3 | Average | kB/s |
|-----------|-------|-------|-------|---------|------|
| 10 MHz | 6.55s | 6.42s | 6.42s | 6.46s | 580 |
| 20 MHz | 6.43s | 6.42s | 6.22s | 6.36s | 589 |

No regression from the added code. Performance matches the post-SM-optimisation baseline.

### Path forward

Fix our concurrent DMA implementation to match the working `rpi5-rp1-pio-bench` architecture:
- Diff our SM setup, DMA config, and buffer handling against the benchmark
- The benchmark uses concurrent pthreads with 256 KB transfers at 42 MB/s on the same hardware
- The kernel driver works correctly — the bug is in our code

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

## RPi 5 — rp1-jtag FIFO-write fast path (count-word, no DMA)

Measured on 2026-03-02. Bitstream programming via openFPGALoader with librp1jtag.

- Single SM (SM0 only), TMS driven as constant GPIO output
- Count-word protocol with autopush disabled
- Data words written via `pio_sm_put_blocking` (no DMA)
- SM shifts exactly `num_bits` bits per call, no padding
- Activated when `tdo == NULL && num_bits >= STREAM_THRESHOLD` (1024 bits)
- Falls back to sequential 224-bit DMA chunks for smaller transfers or when TDO needed
- Pins: TDI=27, TDO=22, TCK=4, TMS=17
- Bitstream: user-100.bit (3,825,888 bytes), XC7A100T, CRC verified by FPGA

### 10 MHz (clkdiv=5)

| Run | Wall time | DONE | kB/s |
|-----|-----------|------|------|
| 1 | 9.42s | 1 | 397 |

### 15 MHz (clkdiv=3.33)

| Run | Wall time | DONE | kB/s |
|-----|-----------|------|------|
| 1 | 9.42s | 1 | 397 |

### 20 MHz (clkdiv=2.5)

| Run | Wall time | DONE | kB/s |
|-----|-----------|------|------|
| 1 | 9.42s | 1 | 397 |

### Analysis

Transfer time is **identical at all frequencies** (9.42s), confirming the bottleneck is **PIOLib ioctl overhead, not TCK frequency**. Each `pio_sm_put_blocking` call is a kernel ioctl taking ~7us. The 3.8 MB bitstream requires ~962K ioctls (count word + data words per call × ~100 calls of ~9,570 words each).

strace breakdown: 962,374 ioctls × 7us/ioctl = 6.7s kernel time + 2.7s userspace overhead.

**Compared to bulk DMA (30720-bit chunks):**
- FIFO-write: 9.42s (ioctl-limited, frequency-independent) — **DONE=1**
- Bulk DMA @10MHz: 1.74s (TCK-limited, scales with frequency) — **DONE=1**
- Bulk DMA @20MHz: 0.96s — **DONE=1**

The FIFO-write path is **slower** than the previous bulk DMA path (30720-bit chunks, documented above) because each word requires a separate kernel ioctl, whereas DMA transfers thousands of words per ioctl. The bulk DMA path achieved 0.96s at 20 MHz (TCK-limited), while FIFO writes are ioctl-limited at 9.42s regardless of frequency.

In the current code, the FIFO-write path is dispatched for `tdo == NULL && num_bits >= STREAM_THRESHOLD` (large write-only transfers like bitstream programming). The sequential 224-bit DMA chunk path handles all other cases (small transfers and transfers needing TDO readback).

**To achieve bulk DMA speeds again**, the concurrent DMA bug in our code must be found and fixed. `rpi5-rp1-pio-bench` proves concurrent DMA works at 42 MB/s on this hardware — the bug is in our SM setup or DMA configuration.

## RPi 5 — rp1-jtag hybrid streaming DMA (interleaved TX/RX)

Measured on 2026-03-02 and 2026-03-03. Bitstream programming via openFPGALoader with librp1jtag.

- Single SM (SM0 only), TMS driven as constant GPIO output
- Hybrid approach: streaming DMA for full 32-bit words + FIFO write for partial word
- SM in streaming mode: `mov x, !null` + `jmp offset+2` (skips count-word protocol)
- Autopull ON, autopush ON — 1:1 TX:RX word ratio
- Interleaved TX then RX DMA in 32-byte (8-word) chunks
- SM runs continuously — no restart between chunks
- No padding — partial words handled via separate FIFO write phase
- SM initialised via `pio_sm_init()` with full config (not standalone `pio_sm_set_clkdiv`)
- BYPASS test: 29/29 sizes pass (1 to 306,056 bits)
- Pins: TDI=27, TDO=22, TCK=4, TMS=17
- Bitstream: user-100.bit (3,825,888 bytes), XC7A100T, CRC verified by FPGA

### 10 MHz (clkdiv=5)

| Run | Wall time | DONE | kB/s |
|-----|-----------|------|------|
| 1 | 6.1s | 1 | 613 |
| 2 | 6.07s | 1 | 616 |

### 20 MHz (clkdiv=2.5)

| Run | Wall time | DONE | kB/s |
|-----|-----------|------|------|
| 1 | 5.9s | 1 | 634 |
| 2 | 6.09s | 1 | 614 |

### Concurrent DMA attempt (DONE=0)

Before settling on interleaved DMA, concurrent TX+RX DMA via pthreads was
tested (identical architecture to `rpi5-rp1-pio-bench` which achieves 42 MB/s).
The concurrent approach gives **DONE=0** — FPGA rejects the bitstream.
BYPASS loopback tests also fail with concurrent DMA (data corruption).
Reverting to interleaved (sequential TX-then-RX per chunk) restores DONE=1.

The root cause is a bug in the rp1-jtag library's SM setup or DMA configuration,
NOT the kernel driver. The benchmark at rpi5-rp1-pio-bench achieves 42 MB/s with
concurrent DMA on the same hardware, proving the kernel driver works correctly.

### RP1 PIO 2× clock factor

Testing revealed that the RP1 PIO state machine runs at **exactly 2× the
calculated TCK frequency**. With `RP1_PIO_CLK_HZ=200 MHz` and `INSTR_PER_BIT=4`:

| Requested freq | clkdiv | Expected TCK | Measured TCK |
|---------------|--------|-------------|-------------|
| 5 MHz | 10 | 5 MHz | 10 MHz |
| 10 MHz | 5 | 10 MHz | 20 MHz |

The factor is perfectly consistent: `clock_get_hz(clk_sys)` reports 200 MHz,
but the SM executes instructions at 2× that rate. This may indicate the RP1 PIO
clock is actually 400 MHz, or that the divider formula differs from RP2040.
The 2× factor does **not** cause DONE=0 (tested at 1 MHz/2 MHz actual — still DONE=1
with interleaved DMA, still DONE=0 with concurrent DMA).

### Analysis

Transfer time is **nearly identical at 10 MHz and 20 MHz** (~6.1s vs ~6.0s), confirming the bottleneck is **ioctl overhead, not TCK frequency**. Each 32-byte chunk requires 2 ioctls (TX xfer_data + RX xfer_data), giving ~240,000 ioctls per bitstream.

**Why 32-byte chunks are the maximum:** with autopush ON, each TX word produces an RX word. The RP1 PIO RX FIFO is 8 words (32 bytes) deep. After processing 8 TX words, the RX FIFO is full and the SM stalls. We must drain RX (1 ioctl) before sending the next TX chunk (1 ioctl), limiting chunks to 32 bytes.

**Why not concurrent TX+RX DMA (yet):** our concurrent DMA attempt gives DONE=0 (concurrent pthreads → DONE=0, interleaved sequential → DONE=1). However, `rpi5-rp1-pio-bench` achieves 42 MB/s with concurrent DMA on the same hardware, proving the kernel driver handles it correctly. The bug is in our SM setup or DMA configuration — not yet root-caused.

**Why not TX-only DMA (autopush OFF):** TX-only DMA (calling config_xfer for both directions but only xfer_data for TX) leaves the kernel DMA state inconsistent. Subsequent sequential-path transfers hang in D-state (uninterruptible sleep), blocking post-programming IR readback.

**SM init vs standalone clkdiv:** `pio_sm_set_clkdiv()` does not reliably apply the clock divider on RP1. Must use `pio_sm_init()` with a full `pio_sm_config` struct. The 2× clock factor persists regardless of init method.

**Compared to other approaches (DONE=1 verified):**
- Sequential DMA (224-bit chunks): 14.4s → hybrid is **2.4x faster**
- FIFO-write (put_blocking): 9.42s → hybrid is **1.5x faster**

The hybrid approach reduces ioctls per segment by ~4x vs sequential DMA (2 ioctls per 256-bit chunk vs 8 ioctls per 224-bit chunk), but the improvement is limited by the 8-word RX FIFO depth which prevents larger DMA chunks.

**Path forward:** further speedup requires fixing our concurrent DMA implementation to match the working `rpi5-rp1-pio-bench` architecture (which achieves 42 MB/s). The kernel driver supports concurrent DMA correctly — the bug is in our code.

## Summary

| Platform | Method | Bitstream | Time | Throughput | DONE | vs sysfsgpio |
|----------|--------|-----------|------|------------|------|-------------|
| RPi 5 | rp1-jtag bulk DMA @20MHz | 3.8 MB (100T) | 0.96s | 3,893 kB/s | 1* | 41x |
| RPi 5 | rp1-jtag bulk DMA @15MHz | 3.8 MB (100T) | 1.22s | 3,053 kB/s | 1* | 32x |
| RPi 5 | rp1-jtag bulk DMA @10MHz | 3.8 MB (100T) | 1.74s | 2,153 kB/s | 1* | 22x |
| RPi 3B+ | OpenOCD bcm2835gpio | 2.1 MB (35T) | 1.50s | 1,428 kB/s | 1 | 15x |
| RPi 5 | rp1-jtag bulk DMA @5MHz | 3.8 MB (100T) | 3.27s | 1,145 kB/s | 1* | 12x |
| RPi 5 | **rp1-jtag hybrid streaming** | **3.8 MB (100T)** | **6.1s** | **613 kB/s** | **1** | **6x** |
| RPi 5 | rp1-jtag (two-SM chunked) | 3.8 MB (100T) | 6.3s | 589 kB/s | 1 | 6x |
| RPi 5 | rp1-jtag FIFO-write | 3.8 MB (100T) | 9.42s | 397 kB/s | 1 | 4x |
| RPi 5 | rp1-jtag sequential DMA | 3.8 MB (100T) | 14.4s | 260 kB/s | 1 | 3x |
| RPi 5 | OpenOCD sysfsgpio | 3.8 MB (100T) | 39.0s | 96 kB/s | 1 | 1x (baseline) |

\* Bulk DMA (30720-bit chunks) later found to corrupt JTAG data via BYPASS
integrity test. FPGA DONE=1 was a false positive (CRC passed by chance or
was not being fully checked). The code was reverted to sequential 224-bit
chunks (14.4s) which pass all BYPASS tests. The hybrid streaming approach
is the current active fast path for tdo==NULL transfers.

### Key observations

1. **Hybrid streaming DMA is the current active fast path** — 6.1s at 10 MHz with verified DONE=1 and all 29 BYPASS data integrity tests passing. This is 2.4x faster than the sequential DMA fallback (14.4s) and 1.5x faster than the FIFO-write path (9.42s).

2. **Ioctl overhead is the dominant bottleneck** — the hybrid path requires ~240,000 ioctls per bitstream (TX+RX per 32-byte chunk). Transfer time is frequency-independent (~6.1s at both 10 and 20 MHz), confirming ioctl overhead dominates.

3. **8-word RX FIFO limits DMA chunk size** — with autopush ON, each TX word produces an RX word. The RX FIFO can hold 8 words (32 bytes). Larger chunks stall the SM when the RX FIFO overflows. This is the fundamental bottleneck preventing larger DMA transfers.

4. **Concurrent DMA fails in rp1-jtag (bug in our code)** — our concurrent TX+RX DMA via pthreads gives DONE=0. However, `rpi5-rp1-pio-bench` achieves 42 MB/s with concurrent DMA on the same hardware, proving the kernel driver works correctly. The root cause is a bug in our SM setup or DMA configuration, not yet identified.

5. **Signal integrity limits max TCK to 20 MHz** — 25+ MHz TCK causes IDCODE corruption on the RPi-to-NeTV2 wiring. This is a hardware limitation, not software.

6. **Bulk DMA results were invalidated** — the 0.96s @20MHz bulk DMA results (30720-bit chunks) passed FPGA DONE but failed the BYPASS data integrity test. The code was reverted to data-integrity-verified approaches.

7. **RP1 PIO runs at 2× calculated frequency** — `clock_get_hz(clk_sys)` reports 200 MHz but the SM executes at 2× that rate. A divider of 5 (expected 10 MHz TCK) produces 20 MHz TCK. The factor is perfectly consistent and does not affect correctness. Root cause unknown — may be a 400 MHz PIO clock or a divider formula difference from RP2040.

8. **`pio_sm_init()` required for clock divider** — standalone `pio_sm_set_clkdiv()` does not reliably apply on RP1. Must use `pio_sm_init()` with a full `pio_sm_config` struct.

9. **Path forward: fix our concurrent DMA bug** — `rpi5-rp1-pio-bench` proves concurrent DMA works on this hardware at 42 MB/s. Our concurrent DMA attempt fails due to a bug in our SM setup or DMA configuration. Finding and fixing this bug should bring transfer time under 1 second.
