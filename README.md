# rp1-jtag

High-speed JTAG via Raspberry Pi 5 RP1 PIO.

The RPi 5's GPIO pins connect to the RP1 chip via PCIe, making traditional software bit-banging (sysfsgpio, linuxgpiod) ~50-200x slower than the direct-register approach that worked on Pi 1-4. The RP1 contains a PIO (Programmable I/O) subsystem that can run autonomous state machines at up to 200 MHz, bypassing the PCIe latency bottleneck entirely.

## This is the development repository

It holds the library and the two drivers, and its CI uploads binaries as
workflow artifacts for development and hardware testing. **It publishes no
Debian packages and no binary releases.**

Installable packages and static binaries come from
[fpgas-online/fpgas.online-fpga-tools](https://github.com/fpgas-online/fpgas.online-fpga-tools),
which carries these drivers inside a larger patch series (SPI flash info,
NeTV2 boards, Tiny Tapeout FPGA Demo Board, ECP5 TraceID) and builds
`openfpgaloader-fpgasonline` / `openocd-fpgasonline` for bookworm, trixie and
sid on arm64 and armhf, plus static arm64/armv7/armv6 binaries.

The packages this repository used to publish — `openfpgaloader-rp1pio` and
`openocd-rp1pio` from `mith.ro/rp1-jtag` — are superseded by those.

## Components

- **librp1jtag** — C library wrapping PIOLib for high-speed JTAG shift operations
- **openFPGALoader driver** — Native `rp1pio` cable driver
- **OpenOCD driver** — Native `rp1_pio_jtag` adapter driver

## Performance

Measured 2026-09-22 on a Pi 5 (rpi5-netv2) loading a 3.8 MB XC7A100T
bitstream to `DONE=1`, stock kernel 6.12.47:

| Method | Throughput | 3.8 MB bitstream |
|--------|-----------|-----------------|
| libgpiod bit-bang (Pi 5) | ~51 kB/s | ~75 s |
| librp1jtag as shipped (word-by-word FIFO) | ~97 kB/s | ~39 s |
| librp1jtag streaming DMA (work in progress) | ~640 kB/s | ~5.9 s |

The streaming DMA path is not on `main`; the shipped library has
`use_dma = false`. Kernel 6.18.50 additionally fails the Raspberry Pi PIO DMA
reference test, so the DMA numbers above hold only on 6.12.47.

## Quick Start

### Build

```bash
# Prerequisites (Raspberry Pi 5 with Raspberry Pi OS)
sudo apt install libfdt-dev cmake build-essential git

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

### Install the library

```bash
# Install librp1jtag system-wide (needed by openFPGALoader / OpenOCD drivers)
sudo cmake --install build
sudo ldconfig
```

### Build openFPGALoader with rp1pio support

openFPGALoader does not yet include the rp1pio cable driver upstream.
Use the integration script in `drivers/openfpgaloader/` to patch a source tree:

```bash
# Install openFPGALoader build dependencies
sudo apt install libfdt-dev libftdi1-dev libhidapi-dev pkg-config

# Clone openFPGALoader and apply rp1pio driver patches
git clone https://github.com/trabucayre/openFPGALoader.git
cd rp1-jtag
python3 drivers/openfpgaloader/integrate.py ../openFPGALoader

# Build openFPGALoader with RP1 PIO enabled
cmake -DENABLE_RP1_PIO=ON -B ../openFPGALoader/build -S ../openFPGALoader
cmake --build ../openFPGALoader/build
```

See [drivers/openfpgaloader/README.md](drivers/openfpgaloader/README.md) for
details on pin mapping, method mapping, and performance.

### Program an FPGA

```bash
# Using openFPGALoader with rp1pio cable (requires sudo for /dev/pio0)
# Default pins are NeTV2 wiring (TCK=4, TMS=17, TDI=27, TDO=22)
sudo openFPGALoader -c rp1pio --detect
sudo openFPGALoader -c rp1pio bitstream.bit

# With explicit pin configuration (format: --pins TDI:TDO:TCK:TMS)
sudo openFPGALoader -c rp1pio --pins 27:22:4:17 --detect

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
     openFPGALoader / OpenOCD
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
