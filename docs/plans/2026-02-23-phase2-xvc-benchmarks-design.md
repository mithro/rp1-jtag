# Phase 2 Design: XVC Daemon + Benchmarks

## Goal

Add an XVC (Xilinx Virtual Cable) daemon for remote JTAG access via Vivado,
and produce a benchmark comparison across all available JTAG methods.

## XVC Daemon

### Protocol

XVC v1.0 over TCP (default port 2542). Three commands:

| Command | Wire format | Response |
|---|---|---|
| `getinfo:` | 8 ASCII bytes | `xvcServer_v1.0:<max_vector_len>\n` |
| `settck:<period>` | 7 bytes + 4-byte LE uint32 (ns) | 4-byte LE uint32 (actual ns) |
| `shift:<n><tms><tdi>` | 6 bytes + 4-byte LE uint32 + 2 * ceil(n/8) bytes | ceil(n/8) bytes TDO |

All integer fields are little-endian uint32. Bit vectors are LSB-first
byte arrays (bit 0 of byte 0 shifts first), matching librp1jtag's convention.

### Architecture

Single C source file (`xvc/xvc_server.c`), ~200-300 lines. Links against
`librp1jtag`. Single-threaded, single-client, blocking I/O.

```
Client (Vivado)  ---TCP--->  rp1-xvc  ---librp1jtag--->  PIO/DMA  --->  FPGA
```

The XVC shift command maps directly to `rp1_jtag_shift()`. The settck command
converts nanosecond period to Hz and calls `rp1_jtag_set_freq()`.

### Command line

```
rp1-xvc --pins TDI:TDO:TCK:TMS [--port 2542] [--freq 6000000] [--verbose]
```

Pin format matches openFPGALoader convention. For NeTV2:
`rp1-xvc --pins 27:22:4:17`

### Design decisions

- **Single-client**: Vivado connects, sends commands sequentially, disconnects.
  Matches Xilinx's reference XVC server. Multi-client support is a future
  enhancement.
- **No threading**: Blocking read/write on the socket. The XVC protocol is
  inherently sequential (request-response), so async/threading adds complexity
  without benefit.
- **Max vector length**: 32768 bytes (256 Kbit). Matches common XVC server
  implementations and is large enough for any practical JTAG operation.
- **XVC v1.0 only**: The v1.1 memory read/write extensions (mrd/mwr) are not
  needed for JTAG-only use. Future enhancement if AXI debug access is desired.

### Future enhancements (documented in README)

- Multi-client support (accept multiple connections, serve sequentially or
  with mutex)
- XVC v1.1 protocol (mrd/mwr memory access commands)
- Systemd service file for auto-start
- IPv6 support

## Benchmarks

### Configurations to measure

| # | Platform | GPIO method | Tool | Adapter driver |
|---|---|---|---|---|
| 1 | RPi 3B+ | bcm2835gpio (mmap) | OpenOCD 0.10 | bcm2835gpio |
| 2 | RPi 5 | sysfsgpio (sysfs) | OpenOCD 0.12 | sysfsgpio |
| 3 | RPi 5 | rp1-jtag word-by-word | openFPGALoader | rp1pio |
| 4 | RPi 5 | rp1-jtag DMA+fast PIO | openFPGALoader | rp1pio |

### Hardware

- **RPi 3B+ (rpi3-netv2)**: `pi@ipv4.eth0.rpi3-netv2.iot.welland.mithis.com`,
  Raspbian 4.14, OpenOCD 0.10 with bcm2835gpio. NeTV2 may be XC7A35T or
  XC7A100T (check IDCODE).
- **RPi 5 (rpi5-netv2)**: `tim@ipv4.eth0.rpi5-netv2.iot.welland.mithis.com`,
  Debian 6.12, OpenOCD 0.12 with sysfsgpio. NeTV2 is XC7A100T (confirmed
  IDCODE 0x13631093).

### Methodology

1. Verify IDCODE on each board to confirm FPGA variant
2. Load bitstream appropriate to each variant (user-35.bit or user-100.bit)
3. Time bitstream load 3 times per configuration, report average
4. Report kB/s throughput (bitstream size / wall time)
5. Compute speedup relative to slowest baseline

### RPi 3 setup

Existing OpenOCD + bcm2835gpio configuration at
`/home/pi/code/netv2mvp-scripts/alphamax-rpi.cfg` (10 MHz adapter_khz).
Use `fpga-jtag.cfg` to load bitstream.

### RPi 5 sysfsgpio setup

Create OpenOCD config using `sysfsgpio` adapter. The RPi 5 uses RP1 for GPIO
with sysfs base offset at gpiochip512. GPIO BCM numbers need the offset added
(e.g. BCM4 = sysfs 516).

Pins: TCK=4, TMS=17, TDI=27, TDO=22 (same physical wiring as rp1-jtag).

### RPi 5 rp1-jtag

Already working via openFPGALoader. Two configurations:
- Phase 1 (word-by-word): disable DMA in library, or use pre-Phase-3 library
- Phase 3 (DMA+fast PIO): current library version

Since the library now always uses DMA when available, measuring Phase 1
separately requires either:
(a) A benchmark flag to disable DMA/fast program, or
(b) Using the previously recorded ~88 kB/s number from Phase 1

Approach (b) is simpler and the number is well-established from multiple runs.

### Output

Results table in the project README, plus a standalone benchmark script that
can reproduce the measurements.
