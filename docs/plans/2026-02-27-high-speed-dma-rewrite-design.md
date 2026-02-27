# Design: High-Speed DMA Rewrite of librp1jtag

## Problem

The current librp1jtag uses word-by-word blocking FIFO calls
(`pio_sm_put_blocking`/`pio_sm_get_blocking`), each round-tripping through
PIOLib's mailbox RPC over PCIe (~10 us per word). This yields ~400 KB/s JTAG
throughput.

The [rpi5-rp1-pio-bench](https://github.com/mithro/rpi5-rp1-pio-bench)
benchmark demonstrates that bulk DMA with concurrent TX/RX pthreads achieves
~42 MB/s — a 168x improvement over blocking FIFO calls.

## Goals

1. Rewrite librp1jtag core to use DMA with concurrent pthreads
2. Eliminate TMS splitting by using two PIO state machines (SM0 for
   TDI/TDO/TCK, SM1 for TMS synchronized via GPIO wait)
3. Maintain constant TCK frequency throughout the entire shift operation
4. Test on real hardware: PIO loopback for max speed, then Artix-7 FPGA
   programming at reduced TCK
5. Demonstrate speedup over RPi 3 libgpiod method

## Architecture

```
librp1jtag (rewritten)
+-- rp1_jtag.h          Public API (preserved from phase1)
+-- rp1_jtag.c           Core: DMA + two SMs + pthreads
+-- pio/
|   +-- jtag_shift.pio   SM0: counted, 4 instr/bit (TDI/TDO/TCK)
|   +-- jtag_tms.pio     SM1: TMS sync (wait for TCK, output TMS)
|   +-- jtag_loopback.pio  Benchmark: 2 instr/word (pure DMA speed)
+-- No backend abstraction (direct PIOLib calls)
+-- pthreads for concurrent DMA (hidden inside rp1_jtag_shift)
```

### Key design changes from phase1

- **No pio_backend.h vtable / mock backend**: Direct PIOLib calls. Testing
  happens on real hardware, plus a PIO simulator for instruction verification.
- **No TMS splitting**: TMS bit vector goes directly to SM1 via DMA. SM1
  sets GPIO17 in sync with SM0's TCK, so each JTAG bit gets its own TMS value.
- **No program switching**: SM0 always runs jtag_shift, SM1 always runs
  jtag_tms. Only loopback mode uses a different program.
- **DMA with concurrent threads**: Three pthreads for SM0-TX, SM0-RX,
  SM1-TX. Follows the benchmark's proven pattern.
- **4 instr/bit PIO program**: The extra `nop` provides TMS setup time
  from SM1. 50 MHz max TCK vs old 5 instr/bit counted (40 MHz) or
  2 instr/bit fast (100 MHz, but couldn't handle arbitrary bit counts).

## PIO Programs

### SM0: jtag_shift.pio (4 instructions/bit)

```
.program jtag_shift
.side_set 1

    pull block       side 0    ; Get bit count - 1
    out x, 32        side 0    ; X = counter
loop:
    out pins, 1      side 0    ; TDI from TX FIFO, TCK LOW (setup)
    nop              side 0    ; Extra cycle: TMS settles (from SM1)
    in pins, 1       side 1    ; TDO to RX FIFO, TCK HIGH (sample)
    jmp x-- loop     side 0    ; TCK LOW (falling edge), loop
    push             side 0    ; Flush partial TDO word
    ; Stalls on next pull -> TCK stays LOW
```

Pin mapping:
- OUT base = TDI (GPIO27), 1 pin
- IN base = TDO (GPIO22), 1 pin
- SIDESET base = TCK (GPIO4), 1 pin

Shift config:
- Out shift: right, autopull at 32 (LSB-first TDI)
- In shift: right, autopush at 32 (LSB-first TDO)

Performance:
- 200 MHz / 4 = 50 MHz max TCK
- Divider 1.5 -> 33 MHz TCK (Artix-7 max)
- Divider 4.0 -> 12.5 MHz TCK
- Divider 10.0 -> 5 MHz TCK

FIFO protocol:
- TX: [count-1, tdi_word_0, tdi_word_1, ..., tdi_word_N]
  where N = ceil(num_bits / 32)
- RX: [tdo_word_0, tdo_word_1, ..., tdo_word_K]
  where K = ceil(num_bits / 32)
  (plus possible spurious word from explicit push on aligned transfers)

### SM1: jtag_tms.pio (3 instructions/bit, GPIO-synced)

```
.program jtag_tms

.wrap_target
    wait 0 gpio 4             ; Wait for TCK LOW (from SM0)
    out pins, 1               ; TMS bit from TX FIFO -> GPIO17
    wait 1 gpio 4             ; Wait for TCK HIGH
.wrap
```

Pin mapping:
- OUT base = TMS (GPIO17), 1 pin
- GPIO4 = TCK (read via wait instruction, driven by SM0)

Shift config:
- Out shift: right, autopull at 32 (LSB-first TMS)

Behaviour:
- Synchronizes to SM0's TCK via GPIO wait instructions
- Outputs one TMS bit per TCK cycle
- Naturally stalls when SM0 stops toggling TCK (count expired)
- No count word needed — SM1 processes bits until TCK stops

### Loopback: jtag_loopback.pio (2 instructions/word)

```
.program jtag_loopback

.wrap_target
    out x, 32                 ; TX FIFO -> scratch X (autopull)
    in x, 32                  ; X -> RX FIFO (autopush)
.wrap
```

Purpose: benchmark raw DMA throughput (PIO processes 100 Mwords/s,
DMA-limited to ~42 MB/s).

## DMA and Threading Model

Following the benchmark pattern (concurrent pthreads for blocking
`pio_sm_xfer_data` calls):

```
rp1_jtag_shift(num_bits, tms, tdi, tdo):
  1. Pack TDI -> SM0 TX buffer: [count-1, tdi_word_0, ..., tdi_word_N]
  2. Pack TMS -> SM1 TX buffer: [tms_word_0, ..., tms_word_M]
  3. Allocate SM0 RX buffer: [tdo_word_0, ..., tdo_word_K]

  4. Configure DMA:
     pio_sm_config_xfer(pio, sm0, PIO_DIR_TO_SM,   sm0_tx_size, 1)
     pio_sm_config_xfer(pio, sm0, PIO_DIR_FROM_SM,  sm0_rx_size, 1)
     pio_sm_config_xfer(pio, sm1, PIO_DIR_TO_SM,    sm1_tx_size, 1)

  5. Set FIFO thresholds for heavy DMA channels:
     pio_sm_set_dmactrl(pio, sm0, true,  0xC0000108)  // TX
     pio_sm_set_dmactrl(pio, sm0, false, 0xC0000108)  // RX
     pio_sm_set_dmactrl(pio, sm1, true,  0xC0000108)  // TMS TX

  6. Enable both SMs

  7. Launch 3 pthreads concurrently:
     Thread 1: pio_sm_xfer_data(pio, sm0, PIO_DIR_TO_SM,   sm0_tx_size, tx_buf)
     Thread 2: pio_sm_xfer_data(pio, sm0, PIO_DIR_FROM_SM, sm0_rx_size, rx_buf)
     Thread 3: pio_sm_xfer_data(pio, sm1, PIO_DIR_TO_SM,   sm1_tx_size, tms_buf)

  8. pthread_join all three

  9. Disable both SMs
 10. Drain any spurious RX FIFO words
 11. Fix partial-word alignment (right-shift last word)
 12. Unpack TDO -> output buffer
```

Buffer sizes:
- SM0 TX: (1 + ceil(num_bits/32)) * 4 bytes (count word + TDI data)
- SM0 RX: ceil(num_bits/32) * 4 bytes (TDO data)
- SM1 TX: ceil(num_bits/32) * 4 bytes (TMS data)

## Public API

Preserved from phase1:

```c
typedef struct rp1_jtag rp1_jtag_t;

typedef struct {
    int tck, tms, tdi, tdo;       // Main JTAG pins (BCM GPIO numbers)
    int srst, trst;               // Optional resets (-1 if unused)
} rp1_jtag_pins_t;

rp1_jtag_t *rp1_jtag_init(const rp1_jtag_pins_t *pins);
rp1_jtag_t *rp1_jtag_init_loopback(void);
void rp1_jtag_close(rp1_jtag_t *jtag);
int rp1_jtag_set_freq(rp1_jtag_t *jtag, uint32_t freq_hz);
uint32_t rp1_jtag_get_freq(rp1_jtag_t *jtag);
int rp1_jtag_shift(rp1_jtag_t *jtag, uint32_t num_bits,
                    const uint8_t *tms, const uint8_t *tdi, uint8_t *tdo);
int rp1_jtag_toggle_clk(rp1_jtag_t *jtag, uint32_t num_clocks,
                         bool tms, bool tdi);
int rp1_jtag_reset(rp1_jtag_t *jtag, int srst, int trst);
```

`rp1_jtag_init_loopback()` uses jtag_loopback.pio (single SM, no GPIO)
for pure DMA throughput benchmarking.

## Performance Targets

| Mode                              | Max TCK  | Throughput  | vs current   |
|-----------------------------------|----------|-------------|--------------|
| PIO loopback (DMA benchmark)      | N/A      | ~42 MB/s    | ~100x        |
| JTAG loopback (SM0+SM1, no FPGA)  | 50 MHz   | ~6.25 MB/s  | ~15x         |
| JTAG real (Artix-7 at 33 MHz)     | 33 MHz   | ~4.1 MB/s   | ~10x         |
| RPi 3 libgpiod (comparison)       | ~1 MHz   | ~125 KB/s   | baseline     |

## Testing Strategy

1. **PIO simulator tests**: Verify jtag_shift.pio and jtag_tms.pio
   instruction sequences produce correct TCK/TDI/TMS/TDO waveforms
2. **Hardware PIO loopback**: jtag_loopback.pio DMA throughput benchmark
   (no GPIO, pure DMA speed)
3. **Hardware JTAG loopback**: SM0+SM1 with TDI->TDO jumper wire —
   verify data integrity at various TCK frequencies
4. **Hardware IDCODE test**: Read NeTV2 FPGA IDCODE (XC7A100T, 0x13631093)
   via two-SM JTAG at 33 MHz TCK
5. **Performance benchmark**: Measure actual TCK frequency and throughput,
   compare with RPi 3 libgpiod performance

## Build System

Keep CMake. Changes:
- Remove pio_backend_mock.c, pio_backend.h (no more backend abstraction)
- Remove pio_backend_rp1.c (direct PIOLib calls in rp1_jtag.c)
- Link libpthread
- New PIO programs (jtag_shift, jtag_tms, jtag_loopback)
- PIO simulator tests still build without PIOLib (no hardware needed)

## Files to Create/Modify

| File | Action |
|------|--------|
| lib/src/rp1_jtag.c | Rewrite (DMA + two SMs + pthreads) |
| lib/include/rp1_jtag.h | Minor updates (keep API, update comments) |
| lib/src/rp1_jtag_internal.h | Rewrite (new internal types) |
| lib/src/pio_backend.h | Delete |
| lib/src/pio_backend_rp1.c | Delete |
| lib/src/pio_backend_mock.c | Delete |
| lib/src/pio/jtag_shift.pio | Rewrite (4 instr/bit counted) |
| lib/src/pio/jtag_shift_fast.pio | Delete |
| lib/src/pio/jtag_tms.pio | New (SM1 TMS sync program) |
| lib/src/pio/jtag_loopback.pio | Rewrite (benchmark loopback) |
| lib/src/pio/jtag_target.pio | Keep (hardware test utility) |
| lib/CMakeLists.txt | Update sources, add pthread |
| tests/ | Update for new internals |
| drivers/openfpgaloader/ | Update to match any API changes |
