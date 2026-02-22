# openFPGALoader Integration

Cable driver for openFPGALoader using RP1 PIO JTAG on Raspberry Pi 5.

## Files

- `rp1PioJtag.hpp` - Header
- `rp1PioJtag.cpp` - Implementation

## Integration into openFPGALoader

1. Copy `rp1PioJtag.hpp` and `rp1PioJtag.cpp` to openFPGALoader's `src/` directory
2. Add to `CMakeLists.txt`:
   ```cmake
   option(ENABLE_RP1_PIO "Enable RP1 PIO JTAG support" OFF)
   if(ENABLE_RP1_PIO)
       find_library(RP1JTAG_LIB rp1jtag REQUIRED)
       list(APPEND CABLE_SRCS src/rp1PioJtag.cpp)
       target_compile_definitions(openFPGALoader PRIVATE ENABLE_RP1_PIO)
       target_link_libraries(openFPGALoader PRIVATE ${RP1JTAG_LIB})
   endif()
   ```
3. Add cable registration to `src/cable.cpp` cable_list
4. Add to `src/jtag.cpp` factory

## Usage

```bash
# Build openFPGALoader with RP1 PIO support
cmake -DENABLE_RP1_PIO=ON ..

# Detect JTAG chain
openFPGALoader -c rp1pio --detect

# Program FPGA
openFPGALoader -c rp1pio bitstream.bit

# Custom pins
openFPGALoader -c rp1pio --pins tck=4:tms=17:tdi=27:tdo=22 bitstream.bit
```

## Method Mapping

| openFPGALoader | librp1jtag | Notes |
|---|---|---|
| `writeTMS(tms, len)` | `rp1_jtag_shift(len, tms, 0xFF, NULL)` | TDI high |
| `writeTDI(tx, rx, len, end)` | `rp1_jtag_shift(len, tms_vec, tx, rx)` | TMS: all 0, last=end |
| `writeTMSTDI(tms, tx, rx, len)` | `rp1_jtag_shift(len, tms, tx, rx)` | Direct passthrough |
| `toggleClk(num)` | `rp1_jtag_toggle_clk(num, 0, 1)` | TMS=0, TDI=1 |
| `setClkFreq(freq)` | `rp1_jtag_set_freq(freq)` | |
