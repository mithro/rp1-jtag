# CLAUDE.md

## Project Overview

rp1-jtag: High-speed JTAG via Raspberry Pi 5 RP1 PIO. C library (`librp1jtag`) wrapping PIOLib for autonomous JTAG shift operations, plus drivers for openFPGALoader and OpenOCD, plus a standalone XVC daemon.

## Build

```bash
cmake -B build
cmake --build build
ctest --test-dir build    # Run PIO simulator + unit tests
```

Requires PIOLib from `raspberrypi/utils` installed system-wide for hardware builds. Simulator and unit tests build on any Linux host without PIOLib.

## Testing on RPi 5

Target: `tim@10.1.10.14` (rpi5-netv2) with NeTV2 FPGA board connected via JTAG.

```bash
# Hardware tests (need sudo for /dev/pio0)
sudo ./build/tests/hardware/test_pio_loopback     # No wiring needed
sudo ./build/tests/hardware/test_gpio_loopback     # 1 jumper: TDI→TDO
sudo ./build/tests/hardware/test_target_loopback   # 4 jumpers
sudo ./build/tests/hardware/test_idcode            # Needs NeTV2
```

## Code Style

- **C library**: C11, snake_case, 4-space indent, `rp1_jtag_` prefix
- **PIO programs**: .pio format, assembled with pioasm, generated headers committed
- **openFPGALoader driver**: C++, PascalCase class, camelCase methods, spaces, `#pragma once`
- **OpenOCD driver**: C, snake_case, tab indent, LOG_INFO/LOG_ERROR, ERROR_OK returns
- **Tests**: C, CTest, each test is a standalone executable

## Architecture

- `lib/` — Core C library with PIOLib abstraction layer
- `lib/src/pio/` — PIO programs (.pio source + generated .pio.h)
- `drivers/openfpgaloader/` — openFPGALoader cable driver (C++)
- `drivers/openocd/` — OpenOCD adapter driver (C, GPL-2.0)
- `xvc/` — Standalone XVC daemon
- `tests/pio_sim/` — Custom PIO simulator (C, no hardware)
- `tests/unit/` — Unit tests with mock PIOLib backend
- `tests/hardware/` — Hardware integration tests (RPi 5 required)

## Key Design Decisions

- TMS controlled by host GPIO (not PIO pin) because TDI and TMS are non-contiguous
- TMS vector split into constant-value runs, each becoming one PIO transfer
- Phase 1: word-by-word FIFO interleaving (~400 kB/s, 80x over sysfsgpio)
- Phase 2: DMA for bulk transfers
- 1 state machine used (of 4 available on RP1's single PIO block)

## Development Methodology

- **Small changes**: Each logical unit of work is its own commit. Never batch multiple unrelated changes.
- **Frequent commits**: Commit after each step, not at the end. Every file added, every function written, every test passing.
- **Progress documentation**: Keep `docs/plans/` and `benchmarks/RESULTS.md` updated with findings as they are discovered, not retroactively.
- **Verification before claims**: Never claim performance improvements without measured data. Run benchmarks at multiple frequencies and report actual numbers.
- **Hardware testing protocol**: Always test on rpi5-netv2 after local tests pass. Always reset PIO module before testing (`sudo rmmod rp1-pio && sudo modprobe rp1-pio`). Always run test_many_shifts before bitstream tests.

## NeTV2 JTAG Pins (default)

TCK=GPIO4, TMS=GPIO17, TDI=GPIO27, TDO=GPIO22
FPGA: Xilinx Artix-7 XC7A100T, IDCODE 0x13631093
