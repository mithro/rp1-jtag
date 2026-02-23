# rp1-jtag

High-speed JTAG via Raspberry Pi 5 RP1 PIO.

The RPi 5's GPIO pins connect to the RP1 chip via PCIe, making traditional software bit-banging (sysfsgpio, linuxgpiod) ~50-200x slower than the direct-register approach that worked on Pi 1-4. The RP1 contains a PIO (Programmable I/O) subsystem that can run autonomous state machines at up to 200 MHz, bypassing the PCIe latency bottleneck entirely.

## Components

- **librp1jtag** — C library wrapping PIOLib for high-speed JTAG shift operations
- **openFPGALoader driver** — Native `rp1pio` cable driver
- **OpenOCD driver** — Native `rp1_pio` adapter driver
- **XVC daemon** — Standalone Xilinx Virtual Cable server

## Performance

| Method | Throughput | 3.6 MB bitstream |
|--------|-----------|-----------------|
| sysfsgpio | ~5 kB/s | ~10 minutes |
| librp1jtag (word-by-word) | ~400 kB/s | ~9 seconds |
| librp1jtag (DMA) | ~750 kB/s+ | ~5 seconds |

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
