# Phase 2: XVC Daemon + Benchmarks — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add an XVC v1.0 daemon for Vivado connectivity and benchmark all JTAG methods across RPi 3B+ and RPi 5.

**Architecture:** Single-threaded TCP server (`rp1-xvc`) linking librp1jtag, implementing the three XVC v1.0 commands. Benchmarks compare bcm2835gpio (RPi 3), sysfsgpio (RPi 5), and rp1-jtag (RPi 5) by timing bitstream loads.

**Tech Stack:** C11, POSIX sockets, librp1jtag, CMake, OpenOCD (for baselines), openFPGALoader

---

## Key References

- **XVC protocol**: [github.com/Xilinx/XilinxVirtualCable](https://github.com/Xilinx/XilinxVirtualCable)
  - `getinfo:` → `xvcServer_v1.0:<max_vector_len>\n`
  - `settck:<4-byte LE ns>` → `<4-byte LE actual_ns>`
  - `shift:<4-byte LE num_bits><tms_bytes><tdi_bytes>` → `<tdo_bytes>`
  - All integers little-endian uint32, vectors LSB-first byte arrays
- **librp1jtag public API**: `lib/include/rp1_jtag.h`
- **openFPGALoader driver** (reference consumer): `drivers/openfpgaloader/rp1PioJtag.cpp`
- **RPi 5 sysfs GPIO**: gpiochip571 (label `pinctrl-rp1`), base 571, 54 pins. BCM N = sysfs 571+N.
- **RPi 3 OpenOCD config**: `/home/pi/code/netv2mvp-scripts/alphamax-rpi.cfg` on rpi3-netv2
- **Bitstreams**: RPi 5 has `/home/tim/netv2/user-100.bit`, RPi 3 has `/home/pi/code/netv2-fpga/production-images/user-100.bit` and `user-35.bit`

## SSH Rules

When running commands on remote RPis:
- **One simple command per SSH invocation. No compound statements.**
- Never use `&&`, `||`, pipes, semicolons, heredocs, subshells, loops, or command substitution
- For anything complex: write a script locally, `scp` it, then `ssh host "bash /path/to/script.sh"`
- RPi 5: `ssh tim@ipv4.eth0.rpi5-netv2.iot.welland.mithis.com "<cmd>"`
- RPi 3: `ssh pi@ipv4.eth0.rpi3-netv2.iot.welland.mithis.com "<cmd>"`

---

### Task 1: XVC daemon — CMake and skeleton

**Files:**
- Create: `xvc/xvc_server.c`
- Create: `xvc/CMakeLists.txt`
- Modify: `CMakeLists.txt` (top-level, add `add_subdirectory(xvc)`)

**Step 1: Create xvc/CMakeLists.txt**

```cmake
# XVC daemon (requires PIOLib for librp1jtag hardware support)
add_executable(rp1-xvc xvc_server.c)
target_link_libraries(rp1-xvc rp1jtag_static)
install(TARGETS rp1-xvc RUNTIME DESTINATION bin)
```

**Step 2: Add xvc subdirectory to top-level CMakeLists.txt**

After the `add_subdirectory(examples)` block inside the `if(HAVE_PIOLIB)` guard:
```cmake
    add_subdirectory(xvc)
```

**Step 3: Create xvc/xvc_server.c with argument parsing only**

Write the `main()` function with `getopt_long` parsing for `--pins`, `--port`, `--freq`, `--verbose`, `--help`. Parse pins as `TDI:TDO:TCK:TMS` (colon-separated, matching openFPGALoader convention). Print parsed values and exit. No socket code yet.

The pin parsing: split the `--pins` argument on `:` characters. Expect exactly 4 values: TDI, TDO, TCK, TMS (in that order). Convert each to int with `strtol`, validate 0-27.

**Step 4: Build locally to verify CMake integration**

Build will only succeed on RPi 5 (needs PIOLib). On local machine, verify the CMake configuration doesn't break:

Run: `cmake -B build && cmake --build build`
Expected: Builds (xvc target skipped if no PIOLib)

**Step 5: Build on RPi 5**

Push changes to RPi 5 (git push + pull, or scp files), build, run `./build/xvc/rp1-xvc --help` to verify arg parsing.

**Step 6: Commit**

```
git add xvc/CMakeLists.txt xvc/xvc_server.c CMakeLists.txt
git commit -m "Add XVC daemon skeleton with argument parsing"
```

---

### Task 2: XVC daemon — TCP server and protocol

**Files:**
- Modify: `xvc/xvc_server.c`

**Step 1: Add TCP server setup**

After argument parsing, add:
- Create TCP socket (`socket(AF_INET, SOCK_STREAM, 0)`)
- Set `SO_REUSEADDR`
- Bind to `0.0.0.0:port` (default 2542)
- Listen with backlog 1
- Print "Listening on port N" to stderr
- Accept loop: accept one client, handle it, close client socket, repeat

**Step 2: Add protocol message reader**

Add `read_exact(int fd, void *buf, size_t len)` helper that reads exactly `len` bytes from the socket (looping on partial reads, handling EINTR). Returns 0 on success, -1 on error/disconnect.

**Step 3: Add command dispatch loop**

Inside the client handler, read bytes from the socket and dispatch:

1. Read one byte at a time looking for command prefixes
2. Match `getinfo:` (8 bytes): respond with `xvcServer_v1.0:32768\n`
3. Match `settck:` (7 bytes): read 4-byte LE uint32 period_ns, convert to freq_hz = 1000000000 / period_ns, call `rp1_jtag_set_freq()`, respond with 4-byte LE actual_period_ns
4. Match `shift:` (6 bytes): read 4-byte LE uint32 num_bits, compute byte_count = (num_bits + 7) / 8, read byte_count bytes TMS, read byte_count bytes TDI, call `rp1_jtag_shift()`, write byte_count bytes TDO

For the command matching: use a simple approach — try to read and accumulate bytes into a command buffer. Compare against the three known prefixes. This avoids needing a state machine.

Simpler approach: the three commands start with `g`, `s` (two options: `se` for settck, `sh` for shift). Read bytes to distinguish:
- `g` → read 7 more bytes, expect `etinfo:`
- `s` → read next byte: `e` → `ttck:` (read 4 more), `h` → `ift:` (read 3 more)

**Step 4: Add JTAG initialization**

Before the accept loop:
- Call `rp1_jtag_init(&pins)` with parsed pin config
- If verbose, print pin configuration and frequency
- Set initial frequency via `rp1_jtag_set_freq()`

After the accept loop (on SIGINT or error):
- Call `rp1_jtag_close()`

**Step 5: Add signal handling**

Add `SIGINT` / `SIGTERM` handler that sets a `volatile sig_atomic_t running = 0` flag. The accept loop checks `running` to exit cleanly.

**Step 6: Build and test on RPi 5**

Build on RPi 5. Start the daemon: `sudo ./build/xvc/rp1-xvc --pins 27:22:4:17 --verbose`

Test with a simple netcat probe from another terminal:
`echo -n "getinfo:" | nc localhost 2542`
Expected response: `xvcServer_v1.0:32768`

**Step 7: Commit**

```
git commit -m "Implement XVC v1.0 protocol server"
```

---

### Task 3: XVC daemon — test with Vivado or IDCODE verification

**Files:**
- None (testing only)

**Step 1: Write a simple XVC IDCODE test client**

Create `xvc/test_xvc_idcode.py` — a Python script that connects to the XVC server, sends the TAP reset sequence (5x TMS=1), then shifts 32 bits of DR to read IDCODE.

XVC IDCODE test protocol:
1. Connect to localhost:2542
2. Send `getinfo:` — verify response
3. Send `settck:` with 166 ns period (6 MHz) — verify response
4. Send `shift:` with 5 bits, TMS=0x1f (all 1s), TDI=0x00 — TAP reset to Test-Logic-Reset
5. Send `shift:` with 1 bit, TMS=0x00, TDI=0x00 — move to Run-Test/Idle
6. Send `shift:` with 1 bit, TMS=0x01, TDI=0x00 — move to Select-DR-Scan
7. Send `shift:` with 1 bit, TMS=0x00, TDI=0x00 — move to Capture-DR
8. Send `shift:` with 1 bit, TMS=0x00, TDI=0x00 — move to Shift-DR
9. Send `shift:` with 32 bits, TMS=0x00000000, TDI=0x00000000 — read IDCODE (TDO)
10. Print TDO as hex IDCODE
11. Verify against expected 0x13631093 (XC7A100T)

Actually, steps 5-8 can be combined into one shift of 4 bits with TMS=0b0110. And the IDCODE read in step 9 should set TMS=0 for bits 0-30 and TMS=1 for bit 31 (to exit Shift-DR). So: shift 36 bits total — 4 bits TAP navigation + 32 bits IDCODE.

Simpler approach: after TAP reset (5x TMS=1), shift to Shift-DR (TMS pattern: 0,1,0,0 = 4 bits), then shift 32 bits of DR data.

The Python script uses `socket` and `struct.pack('<I', ...)` for the LE uint32 fields.

**Step 2: Test on RPi 5**

Start daemon in one terminal: `sudo ./build/xvc/rp1-xvc --pins 27:22:4:17 --verbose`
Run test script in another: `python3 xvc/test_xvc_idcode.py`
Expected: prints IDCODE 0x13631093

**Step 3: Commit**

```
git add xvc/test_xvc_idcode.py
git commit -m "Add XVC IDCODE verification test script"
```

---

### Task 4: Benchmark — verify RPi 3 IDCODE and time bitstream load

**Files:**
- Create: `benchmarks/bench_rpi3_bcm2835gpio.sh` (local script to scp and run on RPi 3)

**Step 1: Check IDCODE on RPi 3**

Run on RPi 3:
```
sudo openocd -f /home/pi/code/netv2mvp-scripts/idcode.cfg
```

Look for IDCODE in output. Expected: either 0x0362d093 (35T) or 0x13631093 (100T).

**Step 2: Time bitstream load on RPi 3**

The RPi 3's OpenOCD uses bcm2835gpio. Create a timing script locally at `benchmarks/bench_rpi3_bcm2835gpio.sh`:

```bash
#!/bin/bash
# Benchmark: RPi 3B+ bcm2835gpio bitstream load via OpenOCD
# Run on rpi3-netv2 as root

SCRIPTS=/home/pi/code/netv2mvp-scripts
IMAGES=/home/pi/code/netv2-fpga/production-images

# Detect FPGA variant
echo "=== Detecting FPGA ==="
sudo openocd -f "$SCRIPTS/idcode.cfg" 2>&1

# Time bitstream load (3 runs)
# Use user-100.bit if 100T, user-35.bit if 35T
# Adjust BITFILE below based on IDCODE result
BITFILE="$IMAGES/user-100.bit"

for i in 1 2 3; do
    echo "=== Run $i ==="
    time sudo openocd \
        -f "$SCRIPTS/alphamax-rpi.cfg" \
        -c "source [find cpld/xilinx-xc7.cfg]" \
        -c "init" \
        -c "pld load 0 $BITFILE" \
        -c "exit"
done
```

scp this script to RPi 3, run it, record results.

**Step 3: Record results**

Note the FPGA variant, bitstream file size, and wall time for each run. Calculate kB/s.

**Step 4: Commit script**

```
mkdir -p benchmarks
git add benchmarks/bench_rpi3_bcm2835gpio.sh
git commit -m "Add RPi 3 bcm2835gpio benchmark script"
```

---

### Task 5: Benchmark — RPi 5 sysfsgpio via OpenOCD

**Files:**
- Create: `benchmarks/rpi5-sysfsgpio.cfg` (OpenOCD config for RPi 5 sysfsgpio)
- Create: `benchmarks/bench_rpi5_sysfsgpio.sh`

**Step 1: Create OpenOCD sysfsgpio config for RPi 5**

The RPi 5's RP1 GPIO pins are at sysfs base 571 (gpiochip571, label `pinctrl-rp1`).
BCM pin N = sysfs number 571 + N.

- TCK = BCM 4 → sysfs 575
- TMS = BCM 17 → sysfs 588
- TDI = BCM 27 → sysfs 598
- TDO = BCM 22 → sysfs 593

Create `benchmarks/rpi5-sysfsgpio.cfg`:
```tcl
# OpenOCD sysfsgpio config for RPi 5 + NeTV2
# RPi 5 RP1 GPIO: gpiochip571 (base 571), BCM N = sysfs 571+N

adapter driver sysfsgpio
transport select jtag

sysfsgpio_tck_num  575
sysfsgpio_tms_num  588
sysfsgpio_tdi_num  598
sysfsgpio_tdo_num  593

adapter speed 1000
```

Note: sysfsgpio on RPi 5 will be very slow (each GPIO toggle is an RP1 PCIe round-trip via sysfs). Start with 1000 kHz and see if it even works — may need to reduce.

**Step 2: Create benchmark script**

Create `benchmarks/bench_rpi5_sysfsgpio.sh`:
```bash
#!/bin/bash
# Benchmark: RPi 5 sysfsgpio bitstream load via OpenOCD
# Run on rpi5-netv2 as root

CFG=/home/tim/github/mithro/rp1-jtag/benchmarks/rpi5-sysfsgpio.cfg
BITFILE=/home/tim/netv2/user-100.bit

echo "=== Detecting FPGA ==="
sudo openocd -f "$CFG" \
    -c "source [find cpld/xilinx-xc7.cfg]" \
    -c "init" \
    -c "scan_chain" \
    -c "exit" 2>&1

for i in 1 2 3; do
    echo "=== Run $i ==="
    time sudo openocd -f "$CFG" \
        -c "source [find cpld/xilinx-xc7.cfg]" \
        -c "init" \
        -c "pld load 0 $BITFILE" \
        -c "exit"
done
```

**Step 3: Test sysfsgpio on RPi 5**

scp the config and script to the RPi 5. Run the IDCODE detection first to verify sysfsgpio works with the RP1 GPIO offset. If it doesn't work, investigate and fix the sysfs numbers.

**Important**: sysfsgpio may be extremely slow on RPi 5 (each GPIO write goes through sysfs → RP1 PCIe). A single bitstream load might take 10+ minutes. Consider running only 1 timed run if each takes >5 minutes.

**Step 4: Record results**

Note wall time, calculate kB/s. If sysfsgpio fails entirely on RPi 5, document the failure (this is a legitimate finding — it demonstrates why PIO is needed).

**Step 5: Commit**

```
git add benchmarks/rpi5-sysfsgpio.cfg benchmarks/bench_rpi5_sysfsgpio.sh
git commit -m "Add RPi 5 sysfsgpio benchmark config and script"
```

---

### Task 6: Benchmark — RPi 5 rp1-jtag via openFPGALoader

**Files:**
- Create: `benchmarks/bench_rpi5_rp1jtag.sh`

**Step 1: Create benchmark script**

Create `benchmarks/bench_rpi5_rp1jtag.sh`:
```bash
#!/bin/bash
# Benchmark: RPi 5 rp1-jtag (DMA + fast PIO) bitstream load via openFPGALoader
# Run on rpi5-netv2 as root

OFPGAL=/home/tim/github/mithro/rp1-jtag/tmp/openFPGALoader/build/openFPGALoader
BITFILE=/home/tim/netv2/user-100.bit
PINS="27:22:4:17"

echo "=== Detecting FPGA ==="
sudo "$OFPGAL" -c rp1pio --pins "$PINS" --detect

for i in 1 2 3; do
    echo "=== Run $i ==="
    time sudo "$OFPGAL" -c rp1pio --pins "$PINS" "$BITFILE"
done
```

**Step 2: Run on RPi 5**

scp and run. Record results. Expected: ~6.5s per run (~571 kB/s) based on Phase 3 measurements.

**Step 3: Commit**

```
git add benchmarks/bench_rpi5_rp1jtag.sh
git commit -m "Add RPi 5 rp1-jtag benchmark script"
```

---

### Task 7: Compile results and update README

**Files:**
- Modify: `README.md`
- Create: `benchmarks/README.md`

**Step 1: Compile all benchmark results**

Gather results from Tasks 4-6. Calculate:
- kB/s for each configuration (bitstream_size_bytes / wall_time_seconds / 1024)
- Speedup relative to slowest method

**Step 2: Update README.md Performance section**

Replace the estimated Performance table with measured results:

```markdown
## Performance

Measured bitstream load times (average of 3 runs):

| Platform | Method | Tool | Bitstream | Time | Throughput | vs slowest |
|----------|--------|------|-----------|------|------------|------------|
| RPi 3B+  | bcm2835gpio | OpenOCD 0.10 | user-Xt.bit (X MB) | Xs | X kB/s | Xx |
| RPi 5    | sysfsgpio   | OpenOCD 0.12 | user-100t.bit (3.8 MB) | Xs | X kB/s | Xx |
| RPi 5    | rp1-jtag (PIO word-by-word) | openFPGALoader | user-100t.bit (3.8 MB) | ~42s | ~88 kB/s | Xx |
| RPi 5    | rp1-jtag (PIO+DMA) | openFPGALoader | user-100t.bit (3.8 MB) | ~6.5s | ~571 kB/s | Xx |
```

Fill in actual measured values.

**Step 3: Create benchmarks/README.md**

Document the benchmark methodology, hardware setup, how to reproduce, and the full results table.

**Step 4: Commit**

```
git add README.md benchmarks/README.md
git commit -m "Add benchmark results: bcm2835gpio, sysfsgpio, rp1-jtag comparison"
```

---

### Task 8: XVC README section

**Files:**
- Create: `xvc/README.md`
- Modify: `README.md` (add XVC quick-start to Components section)

**Step 1: Create xvc/README.md**

Document:
- What XVC is and what it's for (Vivado Hardware Manager remote debug)
- Usage: `sudo rp1-xvc --pins 27:22:4:17`
- Vivado connection: Open Hardware Manager → Open Target → Add Xilinx Virtual Cable → host:2542
- Supported protocol: XVC v1.0 (getinfo, settck, shift)
- Limitations: single client, v1.0 only, no SRST/TRST over XVC
- Future enhancements: multi-client, v1.1 (mrd/mwr), systemd service, IPv6

**Step 2: Update top-level README.md**

Add XVC usage example to the "Program an FPGA" section:
```markdown
# Using XVC (Xilinx Virtual Cable) for Vivado
sudo rp1-xvc --pins 27:22:4:17
# Then in Vivado: Open Hardware Manager → Add Virtual Cable → localhost:2542
```

**Step 3: Commit**

```
git add xvc/README.md README.md
git commit -m "Add XVC daemon documentation and README updates"
```
