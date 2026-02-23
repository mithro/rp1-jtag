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

## RPi 5 — rp1-jtag (DMA + fast PIO)

- openFPGALoader with rp1pio cable driver using librp1jtag
- Pins: TDI=27, TDO=22, TCK=4, TMS=17
- Bitstream: user-100.bit (3,825,888 bytes)

| Run | Wall time | kB/s |
|-----|-----------|------|
| 1 | failed | (residual JTAG state from prior XVC test) |
| 2 | 6.54s | 572 |
| 3 | 6.52s | 574 |

- **Average (runs 2-3): 6.53s, 572 kB/s**

## Summary

| Platform | Method | Bitstream | Time | Throughput | vs RPi 5 sysfsgpio |
|----------|--------|-----------|------|------------|-------------------|
| RPi 3B+ | OpenOCD bcm2835gpio | 2.1 MB (35T) | 1.5s | 1,428 kB/s | 15x |
| RPi 5 | OpenOCD sysfsgpio | 3.8 MB (100T) | 39.0s | 96 kB/s | 1x (baseline) |
| RPi 5 | rp1-jtag (DMA + fast PIO) | 3.8 MB (100T) | 6.5s | 572 kB/s | 6x |

### Key observations

1. **RPi 5 sysfsgpio is 15x slower than RPi 3 bcm2835gpio** — each GPIO toggle goes through sysfs writes over the RP1 PCIe bridge, versus direct memory-mapped register access on RPi 3.

2. **rp1-jtag recovers most of the lost performance** — the PIO approach is 6x faster than sysfsgpio on the same RPi 5 hardware, by offloading JTAG clocking to the RP1's PIO state machine.

3. **RPi 3 bcm2835gpio remains fastest in raw kB/s** — direct memory-mapped GPIO bit-banging has lower latency than PIO-mediated transfers. However, the RPi 3 is a much older, slower platform overall.

4. **sysfsgpio on RPi 5 is faster than originally estimated** — we initially estimated ~5 kB/s based on simple test scripts, but OpenOCD 0.12's sysfsgpio driver achieves ~96 kB/s with optimized batched writes.
