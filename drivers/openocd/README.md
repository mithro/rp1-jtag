# OpenOCD Integration

Adapter driver for OpenOCD using RP1 PIO JTAG on Raspberry Pi 5.

## Files

- `rp1_pio_jtag.c` - OpenOCD adapter driver (C, GPL-2.0-or-later)
- `integrate.py` - Automated integration script

## Quick Start

```bash
# 1. Install librp1jtag (from the rp1-jtag repo)
cd /path/to/rp1-jtag
cmake -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr
cmake --build build
sudo cmake --install build
sudo ldconfig

# 2. Clone and patch OpenOCD
git clone https://github.com/openocd-org/openocd.git
python3 integrate.py openocd

# 3. Build OpenOCD
cd openocd
./bootstrap
./configure --enable-rp1-pio-jtag
make -j$(nproc)

# 4. Use (requires sudo for /dev/pio0 access)
sudo ./src/openocd -f netv2.cfg
```

## OpenOCD Configuration

Example config for NeTV2 FPGA board (`netv2.cfg`):

```tcl
# Adapter
adapter driver rp1_pio_jtag
rp1_pio_jtag jtag_nums 4 17 27 22
adapter speed 1000

# Transport
transport select jtag

# Target: Xilinx Artix-7 XC7A100T
set _CHIPNAME xc7a100t
jtag newtap $_CHIPNAME tap -irlen 6 -expected-id 0x13631093

# Init
init
scan_chain
```

## GPIO Pin Configuration

Use the `rp1_pio_jtag jtag_nums` command with BCM GPIO numbers:

```tcl
# rp1_pio_jtag jtag_nums <TCK> <TMS> <TDI> <TDO>
rp1_pio_jtag jtag_nums 4 17 27 22
```

Optional reset lines:

```tcl
rp1_pio_jtag srst_num <gpio>
rp1_pio_jtag trst_num <gpio>
```

For NeTV2 default wiring: TCK=GPIO4, TMS=GPIO17, TDI=GPIO27, TDO=GPIO22.

## Integration Script

`integrate.py` automates all source patches:

```bash
python3 integrate.py /path/to/openocd
```

It patches:
- `src/jtag/interfaces.c` (extern + driver array entry)
- `configure.ac` (enable option + pkg-config check)
- `src/jtag/drivers/Makefile.am` (source file)

The script is idempotent — safe to run multiple times.

## Architecture

This is a **non-bitbang** driver. Unlike `linuxgpiod` or `sysfsgpio` which
toggle GPIO pins one-at-a-time per TCK cycle, this driver uses librp1jtag
which programs the RP1's PIO state machine to shift data autonomously:

| OpenOCD Command | librp1jtag Call | Notes |
|---|---|---|
| `JTAG_SCAN` | `rp1_jtag_shift(bits, tms, tdi, tdo)` | Bulk shift with auto TMS |
| `JTAG_RUNTEST` | `rp1_jtag_toggle_clk(cycles, 0, 0)` | Clock in Run-Test/Idle |
| `JTAG_STABLECLOCKS` | `rp1_jtag_toggle_clk(cycles, tms, 0)` | Clock in stable state |
| `JTAG_TLR_RESET` | `rp1_jtag_shift(len, tms_path, ...)` | Navigate TAP states |
| `JTAG_PATHMOVE` | `rp1_jtag_shift(len, tms_seq, ...)` | Explicit state path |
| `JTAG_TMS` | `rp1_jtag_shift(len, tms_bits, ...)` | Raw TMS sequence |
| `JTAG_RESET` | `rp1_jtag_reset(srst, trst)` | Assert/deassert resets |
| Speed control | `rp1_jtag_set_freq(hz)` | kHz from `adapter speed` |

## Performance

With word-by-word PIO interleaving (Phase 1):
- ~88 kB/s throughput (~18x faster than sysfsgpio)
- Bypasses per-bit PCIe round-trip latency
