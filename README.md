# rp1-jtag

High-speed JTAG via Raspberry Pi 5 RP1 PIO.

The RPi 5's GPIO pins connect to the RP1 chip via PCIe, making traditional software bit-banging (sysfsgpio, linuxgpiod) ~50-200x slower than the direct-register approach that worked on Pi 1-4. The RP1 contains a PIO (Programmable I/O) subsystem that can run autonomous state machines at up to 200 MHz, bypassing the PCIe latency bottleneck entirely.

## Components

- **librp1jtag** — C library wrapping PIOLib for high-speed JTAG shift operations
- **openFPGALoader driver** — Native `rp1pio` cable driver
- **OpenOCD driver** — Native `rp1_pio` adapter driver
- **XVC daemon** — Standalone Xilinx Virtual Cable server

## Performance

Bitstream load benchmarks on Xilinx Artix-7 FPGAs:

| Platform | Method | Bitstream | Time | Throughput | vs sysfsgpio |
|----------|--------|-----------|------|------------|-------------|
| RPi 3B+ | OpenOCD bcm2835gpio | 2.1 MB (XC7A35T) | 1.5s | 1,428 kB/s | — |
| RPi 5 | OpenOCD sysfsgpio | 3.8 MB (XC7A100T) | 39s | 96 kB/s | 1x |
| RPi 5 | rp1-jtag (DMA + fast PIO) | 3.8 MB (XC7A100T) | 6.5s | 572 kB/s | 6x |

On RPi 1-4, the BCM GPIO registers are memory-mapped directly, giving fast bit-bang performance. On RPi 5, GPIO goes through RP1 via PCIe — sysfsgpio is ~15x slower than RPi 3's bcm2835gpio. The rp1-jtag PIO approach recovers most of that lost performance.

See [`benchmarks/`](benchmarks/) for scripts and raw results.

## Quick Start

### Build

```bash
# Prerequisites (Raspberry Pi 5 with Raspberry Pi OS)
sudo apt install cmake build-essential git

# Build PIOLib from raspberrypi/utils
git clone https://github.com/raspberrypi/utils.git
cd utils && cmake -B build && cmake --build build
sudo cmake --install build
cd ..

# Build rp1-jtag
git clone https://github.com/mithro/rp1-jtag.git
cd rp1-jtag
cmake -B build
cmake --build build
```

### Test (no FPGA needed)

```bash
# PIO simulator tests + unit tests (any Linux host)
ctest --test-dir build

# PIO loopback test (RPi 5 only, no wiring)
sudo ./build/tests/hardware/test_pio_loopback
```

### Program an FPGA

```bash
# Using openFPGALoader with rp1pio cable
openFPGALoader -c rp1pio --detect
openFPGALoader -c rp1pio bitstream.bit

# Using XVC (Xilinx Virtual Cable) for Vivado
sudo ./build/xvc/rp1-xvc --pins 27:22:4:17
# Then in Vivado: Open Hardware Manager → Add Virtual Cable → localhost:2542

# Using the library directly
sudo ./build/examples/idcode_read --tck 4 --tms 17 --tdi 27 --tdo 22
```

## Pin Configuration

Default pins (NeTV2 wiring):

| Signal | BCM GPIO | Physical Pin |
|--------|----------|-------------|
| TCK | 4 | 7 |
| TMS | 17 | 11 |
| TDI | 27 | 13 |
| TDO | 22 | 15 |

All pin numbers are configurable at runtime.

## Architecture

```
openFPGALoader / OpenOCD / XVC daemon
            |
      librp1jtag (C library)
            |
      PIOLib (raspberrypi/utils)
            |
      RP1 PIO Hardware (1 SM of 4)
            |
      GPIO → JTAG target
```

The PIO state machine shifts TDI/TDO data autonomously at MHz rates. TMS is controlled by the host between shift blocks via PIOLib GPIO calls. The library splits per-bit TMS vectors into constant-value runs, each becoming one PIO transfer.

## License

Apache-2.0. See [LICENSE](LICENSE).

The OpenOCD driver (`drivers/openocd/`) is GPL-2.0 as required by OpenOCD's license.
