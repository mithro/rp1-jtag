# openFPGALoader Integration

Cable driver for openFPGALoader using RP1 PIO JTAG on Raspberry Pi 5.

## Files

- `rp1PioJtag.hpp` - Header (inherits JtagInterface)
- `rp1PioJtag.cpp` - Implementation
- `integrate.py` - Automated integration script

## Quick Start

```bash
# 1. Install librp1jtag (from the rp1-jtag repo)
cd /path/to/rp1-jtag
cmake -B build -S . && cmake --build build
sudo cmake --install build
sudo ldconfig

# 2. Install openFPGALoader build dependencies
sudo apt install libfdt-dev libftdi1-dev libhidapi-dev pkg-config

# 3. Clone and patch openFPGALoader
git clone https://github.com/trabucayre/openFPGALoader.git
python3 integrate.py openFPGALoader

# 4. Build
cmake -DENABLE_RP1_PIO=ON -B openFPGALoader/build -S openFPGALoader
cmake --build openFPGALoader/build

# 5. Use (requires sudo for /dev/pio0 access)
# Default pins are NeTV2 wiring (TCK=4, TMS=17, TDI=27, TDO=22)
sudo openFPGALoader/build/openFPGALoader -c rp1pio --detect
sudo openFPGALoader/build/openFPGALoader -c rp1pio bitstream.bit
# Or with explicit pins (format: --pins TDI:TDO:TCK:TMS)
sudo openFPGALoader/build/openFPGALoader -c rp1pio --pins 27:22:4:17 --detect
```

## Pin Format

openFPGALoader's `--pins` takes positional colon-separated GPIO numbers: `TDI:TDO:TCK:TMS`

If `--pins` is omitted, the driver defaults to NeTV2 wiring (TCK=4, TMS=17, TDI=27, TDO=22).

For NeTV2 (TCK=4, TMS=17, TDI=27, TDO=22):
```
--pins 27:22:4:17
```

## Integration Script

`integrate.py` automates all source patches:

```bash
python3 integrate.py /path/to/openFPGALoader
```

It patches: `cable.hpp` (enum + cable_list), `jtag.cpp` (include + factory case),
and `CMakeLists.txt` (option, sources, link). The script is idempotent.

## Method Mapping

| openFPGALoader | librp1jtag | Notes |
|---|---|---|
| `writeTMS(tms, len, flush, tdi)` | `rp1_jtag_shift(len, tms, tdi_vec, NULL)` | tdi_vec from tdi param |
| `writeTDI(tx, rx, len, end)` | `rp1_jtag_shift(len, tms_vec, tx, rx)` | TMS: all 0, last=end |
| `writeTMSTDI(tms, tdi, tdo, len)` | `rp1_jtag_shift(len, tms, tdi, tdo)` | Direct passthrough |
| `toggleClk(tms, tdi, clk_len)` | `rp1_jtag_toggle_clk(clk_len, tms, tdi)` | |
| `setClkFreq(freq)` | `rp1_jtag_set_freq(freq)` | |

## Performance

With word-by-word PIO interleaving (Phase 1):
- ~88 kB/s throughput (~18x faster than sysfsgpio)
- 3.6 MB bitstream (XC7A100T): ~42 seconds
