# High-Speed DMA Rewrite Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Rewrite librp1jtag to use bulk DMA with concurrent pthreads and two PIO state machines (SM0 for TDI/TDO/TCK, SM1 for TMS), achieving ~10x speedup over the current word-by-word FIFO approach.

**Architecture:** SM0 runs a 4 instr/bit counted JTAG program that reads a count word then shifts TDI/TDO bits with TCK via sideset. SM1 runs a 3-instruction TMS synchronization program that reads TMS bits from its own TX FIFO and outputs them to the TMS GPIO pin, synchronized to SM0's TCK via `wait gpio`. Three concurrent pthreads handle SM0-TX, SM0-RX, and SM1-TX DMA transfers. The public API (`rp1_jtag_shift()`) remains synchronous with threading hidden inside.

**Tech Stack:** C11, PIOLib (RP1 PIO), pthreads, CMake, CTest

**Design doc:** `docs/plans/2026-02-27-high-speed-dma-rewrite-design.md`

**Test hardware:**
- RPi 5: `tim@10.1.10.14` (rpi5-netv2), NeTV2 with XC7A100T (IDCODE 0x13631093)
- RPi 3: `pi@rpi3-netv2.iot.welland.mithis.com`, NeTV2 with XC7A35T
- JTAG pins: TCK=GPIO4, TMS=GPIO17, TDI=GPIO27, TDO=GPIO22

---

## Task 1: Write new PIO programs

**Files:**
- Rewrite: `lib/src/pio/jtag_shift.pio`
- Create: `lib/src/pio/jtag_tms.pio`
- Rewrite: `lib/src/pio/jtag_loopback.pio`
- Delete: `lib/src/pio/jtag_shift_fast.pio`
- Regenerate: `lib/src/pio/generated/*.pio.h`

### Step 1: Write jtag_shift.pio (SM0: 4 instr/bit counted JTAG)

Replace contents of `lib/src/pio/jtag_shift.pio` with:

```asm
;
; jtag_shift.pio - Counted JTAG shift with DMA (Phase 2)
;
; 4 instructions per bit in the loop. Used with SM1 (jtag_tms.pio)
; for concurrent TMS control.
;
; Pin assignment:
;   - SIDESET pin 0: TCK (toggled automatically by every instruction)
;   - OUT pin 0:     TDI (data shifted out from TX FIFO)
;   - IN pin 0:      TDO (data shifted in to RX FIFO)
;   - TMS:           Controlled by SM1 via jtag_tms.pio (not this SM)
;
; TX FIFO protocol (fed by DMA):
;   Word 0:    bit_count - 1 (loop counter, loaded via pull + out x)
;   Word 1..N: TDI data, LSB-first, 32 bits per word (autopull)
;
; RX FIFO output (drained by DMA):
;   TDO data, LSB-first, 32 bits per word (autopush).
;   Final partial word flushed by explicit push.
;
; JTAG timing (per bit, 4 cycles):
;   Cycle 0: out pins, 1  side 0 -- TDI output, TCK LOW (setup begins)
;   Cycle 1: nop           side 0 -- TMS setup time (SM1 sets TMS here)
;   Cycle 2: in pins, 1   side 1 -- TCK HIGH (rising edge), sample TDO
;   Cycle 3: jmp x--      side 0 -- TCK LOW (falling edge), loop
;
; At 200 MHz PIO clock (5 ns/cycle):
;   - TDI setup before rising edge: 2 cycles = 10 ns
;   - TMS setup before rising edge: 1 cycle = 5 ns (set by SM1)
;   - TDO sample after rising edge: same cycle
;   - TCK period: 4 cycles = 20 ns = 50 MHz max
;   - With divider 1.5: 33.3 MHz (Artix-7 max)
;

.program jtag_shift
.side_set 1

    pull            side 0      ; Get bit count - 1 from TX FIFO
    out x, 32       side 0      ; Load into X as loop counter
loop:
    out pins, 1     side 0      ; Drive TDI, TCK LOW (setup phase)
    nop             side 0      ; Extra cycle for TMS setup from SM1
    in pins, 1      side 1      ; Sample TDO, TCK HIGH (rising edge)
    jmp x-- loop    side 0      ; TCK LOW (falling edge), loop
    push            side 0      ; Flush remaining TDO bits to RX FIFO
```

### Step 2: Write jtag_tms.pio (SM1: TMS synchronized to TCK)

Create new file `lib/src/pio/jtag_tms.pio`:

```asm
;
; jtag_tms.pio - TMS synchronization to TCK (Phase 2)
;
; Runs on SM1 alongside jtag_shift.pio on SM0. Reads TMS bit
; vector from TX FIFO and outputs one bit per TCK cycle,
; synchronized to SM0's TCK output via wait gpio.
;
; Pin assignment:
;   - OUT pin 0:  TMS (GPIO17, controlled by this SM)
;   - GPIO wait:  TCK (GPIO4, driven by SM0's sideset)
;
; TX FIFO protocol (fed by DMA):
;   Word 0..M: TMS data, LSB-first, 32 bits per word (autopull)
;   No count word needed -- SM1 runs until TCK stops toggling.
;
; Synchronization:
;   SM1 waits for TCK LOW, outputs TMS, then waits for TCK HIGH.
;   When SM0's counter expires, TCK stops toggling and SM1 stalls
;   on the next wait instruction. No explicit stop signal needed.
;
; IMPORTANT: The TCK gpio number is hardcoded in the wait instructions.
;   The host must patch it at load time if TCK is not on GPIO4.
;   Use pio_encode_wait_gpio(polarity, gpio) to patch.
;

.program jtag_tms

.wrap_target
    wait 0 gpio 4               ; Wait for TCK LOW (SM0 setup phase)
    out pins, 1                  ; TMS bit from TX FIFO to GPIO17
    wait 1 gpio 4               ; Wait for TCK HIGH (SM0 sample phase)
.wrap
```

### Step 3: Write jtag_loopback.pio (benchmark: pure DMA throughput)

Replace contents of `lib/src/pio/jtag_loopback.pio` with:

```asm
;
; jtag_loopback.pio - Pure DMA throughput benchmark (Phase 2)
;
; Reads 32-bit words from TX FIFO via autopull, passes them
; through unchanged (identity), writes to RX FIFO via autopush.
; No GPIO pins used. Single state machine.
;
; At 200 MHz with 2 instructions per word:
;   Internal throughput = 200 MHz / 2 = 100 Mwords/s = 400 MB/s
;   (DMA is the bottleneck, not PIO -- expect ~42 MB/s practical)
;

.program jtag_loopback

.wrap_target
    out x, 32                    ; autopull: TX FIFO -> OSR -> scratch X
    in x, 32                     ; autopush: X -> ISR -> RX FIFO
.wrap
```

### Step 4: Delete jtag_shift_fast.pio

Remove `lib/src/pio/jtag_shift_fast.pio` and `lib/src/pio/generated/jtag_shift_fast.pio.h`.

### Step 5: Generate PIO headers

Run pioasm to generate headers. If pioasm is not available locally, hand-assemble
the instructions using the RP2040/RP1 PIO instruction encoding reference and write
the `.pio.h` files directly. The instruction encoding is:

For `jtag_shift.pio` (sideset 1 bit, no opt):
```
; Instruction encoding with 1-bit sideset (bit 12):
; side 0 = bit 12 clear, side 1 = bit 12 set
;
; 0: pull block        side 0 = 0x80a0
; 1: out x, 32         side 0 = 0x6020
; 2: out pins, 1       side 0 = 0x6001
; 3: nop               side 0 = 0xa042
; 4: in pins, 1        side 1 = 0x5001
; 5: jmp x--, 2        side 0 = 0x0042
; 6: push block         side 0 = 0x8020
```

For `jtag_tms.pio` (no sideset):
```
; 0: wait 0 gpio 4     = 0x2004
; 1: out pins, 1       = 0x6001
; 2: wait 1 gpio 4     = 0x2084
```

For `jtag_loopback.pio` (no sideset):
```
; 0: out x, 32         = 0x6020
; 1: in x, 32          = 0x4020
```

Write the generated headers to `lib/src/pio/generated/`:
- `jtag_shift.pio.h`
- `jtag_tms.pio.h`
- `jtag_loopback.pio.h`

### Step 6: Commit

```
git add lib/src/pio/
git rm lib/src/pio/jtag_shift_fast.pio lib/src/pio/generated/jtag_shift_fast.pio.h
git commit -m "Rewrite PIO programs for two-SM DMA architecture

New jtag_shift.pio: 4 instr/bit counted loop (was 5), with nop
for SM1 TMS setup time. New jtag_tms.pio: SM1 program that reads
TMS bits from TX FIFO synchronized to SM0's TCK via wait gpio.
New jtag_loopback.pio: 2 instr/word identity loopback for DMA
benchmarking. Removed jtag_shift_fast.pio (no longer needed)."
```

---

## Task 2: Rewrite core library for DMA + two SMs

**Files:**
- Rewrite: `lib/src/rp1_jtag.c`
- Rewrite: `lib/src/rp1_jtag_internal.h`
- Minor update: `lib/include/rp1_jtag.h`
- Delete: `lib/src/pio_backend.h`
- Delete: `lib/src/pio_backend_rp1.c`
- Delete: `lib/src/pio_backend_mock.c`
- Update: `lib/CMakeLists.txt`

### Step 1: Update rp1_jtag.h

Update the public header with revised comments. The API itself stays the same.
Just update the file comment block to mention the two-SM DMA architecture.

### Step 2: Write rp1_jtag_internal.h

Replace contents with new internal types for the two-SM architecture:

```c
/* rp1_jtag_internal.h - Internal types for librp1jtag (Phase 2 DMA) */
#ifndef RP1_JTAG_INTERNAL_H
#define RP1_JTAG_INTERNAL_H

#include "rp1_jtag.h"
#include <stdbool.h>
#include <stdint.h>

/* RP1 PIO clock: 200 MHz */
#define RP1_PIO_CLK_HZ          200000000

/* Default JTAG clock: 10 MHz */
#define DEFAULT_FREQ_HZ         10000000

/* BCM GPIO range on RPi 5 */
#define MAX_GPIO_PIN            27

/* Bits per FIFO word */
#define BITS_PER_WORD           32

/* Instructions per bit in the jtag_shift loop */
#define INSTR_PER_BIT           4

/* FIFO threshold for heavy DMA channels (DREQ enable + threshold=8) */
#define DMA_FIFO_THRESHOLD      0xC0000108

/* Operating modes */
typedef enum {
    MODE_JTAG,          /* Normal JTAG: SM0 + SM1, GPIO pins */
    MODE_LOOPBACK,      /* DMA benchmark: single SM, jtag_loopback.pio */
} rp1_jtag_mode_t;

/* Main context */
struct rp1_jtag {
    rp1_jtag_pins_t pins;
    rp1_jtag_mode_t mode;
    uint32_t freq_hz;

    /* PIOLib handles (set by init, used by all operations) */
    void *pio;              /* PIO instance (PIO type from piolib) */
    int sm0;                /* State machine 0: TDI/TDO/TCK */
    int sm1;                /* State machine 1: TMS (-1 in loopback) */
    unsigned int offset0;   /* SM0 program offset */
    unsigned int offset1;   /* SM1 program offset */
};

/* Utility: number of 32-bit words needed for N bits */
static inline uint32_t bits_to_word_count(uint32_t num_bits)
{
    return (num_bits + BITS_PER_WORD - 1) / BITS_PER_WORD;
}

#endif /* RP1_JTAG_INTERNAL_H */
```

### Step 3: Write rp1_jtag.c

Rewrite the core implementation. Structure:

```c
/* rp1_jtag.c - High-speed JTAG via RP1 PIO with DMA (Phase 2)
 *
 * Uses two PIO state machines for constant-TCK JTAG:
 *   SM0: jtag_shift.pio (TDI/TDO/TCK, 4 instr/bit, counted loop)
 *   SM1: jtag_tms.pio (TMS, synchronized to SM0's TCK via wait gpio)
 *
 * Data transfer via concurrent DMA with three pthreads:
 *   Thread 1: SM0 TX (count word + TDI data)
 *   Thread 2: SM0 RX (TDO data)
 *   Thread 3: SM1 TX (TMS data)
 */

#define _GNU_SOURCE
#include "rp1_jtag_internal.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef HAVE_PIOLIB
#include "piolib.h"
#include "pio/generated/jtag_shift.pio.h"
#include "pio/generated/jtag_tms.pio.h"
#include "pio/generated/jtag_loopback.pio.h"
#endif
```

Key functions to implement (in order):

1. **Bit vector utilities** (same as phase1):
   - `static uint8_t bit_get(const uint8_t *vec, uint32_t bit)`
   - `static void bit_set(uint8_t *vec, uint32_t bit, bool val)`
   - `static void bits_to_words(const uint8_t *bits, uint32_t start_bit, uint32_t num_bits, uint32_t *words)`
   - `static void words_to_bits(const uint32_t *words, uint32_t num_words, uint8_t *bits, uint32_t start_bit, uint32_t num_bits)`

2. **DMA thread wrapper** (from benchmark):
   ```c
   typedef struct {
       PIO pio;
       uint sm;
       enum pio_xfer_dir dir;
       size_t size;
       void *buf;
       int ret;
   } xfer_args_t;

   static void *xfer_thread(void *arg)
   {
       xfer_args_t *a = (xfer_args_t *)arg;
       a->ret = pio_sm_xfer_data(a->pio, a->sm, a->dir, a->size, a->buf);
       return NULL;
   }
   ```

3. **Core shift operation** (`jtag_shift_dma`):
   - Pack TDI into SM0 TX buffer: [count-1, tdi_word_0, ..., tdi_word_N]
   - Pack TMS into SM1 TX buffer: [tms_word_0, ..., tms_word_M]
   - Allocate SM0 RX buffer
   - Configure DMA for all three transfers
   - Set FIFO thresholds (`pio_sm_set_dmactrl`)
   - Enable both SMs
   - Launch 3 pthreads, join all
   - Disable both SMs
   - Drain spurious RX FIFO words
   - Fix partial-word alignment
   - Unpack TDO

4. **Loopback shift** (`loopback_shift_dma`):
   - Single SM, no TMS
   - TX buffer = input words, RX buffer = output words
   - 2 concurrent threads (TX + RX)

5. **Public API**:
   - `rp1_jtag_init()`: Open PIO, claim 2 SMs, load programs, configure pins
   - `rp1_jtag_init_loopback()`: Single SM, loopback program
   - `rp1_jtag_close()`: Disable SMs, remove programs, unclaim, close PIO
   - `rp1_jtag_set_freq()`: Calculate divider = PIO_CLK / (freq_hz * INSTR_PER_BIT)
   - `rp1_jtag_get_freq()`: Return stored freq_hz
   - `rp1_jtag_shift()`: Call jtag_shift_dma (JTAG mode) or loopback_shift_dma
   - `rp1_jtag_toggle_clk()`: Use SM0 shift with constant TDI/TMS
   - `rp1_jtag_reset()`: GPIO control for SRST/TRST

6. **Non-PIOLIB stub**:
   ```c
   #else /* !HAVE_PIOLIB */
   rp1_jtag_t *rp1_jtag_init(const rp1_jtag_pins_t *pins) {
       (void)pins;
       fprintf(stderr, "rp1_jtag: compiled without PIOLib support\n");
       return NULL;
   }
   /* ... stubs for all functions ... */
   #endif
   ```

### Step 4: Delete backend abstraction files

Remove:
- `lib/src/pio_backend.h`
- `lib/src/pio_backend_rp1.c`
- `lib/src/pio_backend_mock.c`

### Step 5: Update lib/CMakeLists.txt

Update source list:
```cmake
set(LIB_SOURCES
    src/rp1_jtag.c
)
```

Remove references to pio_backend_*.c. Add pthread dependency:
```cmake
if(HAVE_PIOLIB)
    target_link_libraries(rp1jtag PRIVATE ${PIOLIB_LIBRARY} Threads::Threads)
endif()
```

### Step 6: Commit

```
git rm lib/src/pio_backend.h lib/src/pio_backend_rp1.c lib/src/pio_backend_mock.c
git add lib/
git commit -m "Rewrite core library for DMA + two-SM architecture

Replace word-by-word FIFO interleaving with bulk DMA using three
concurrent pthreads. SM0 handles TDI/TDO/TCK with a 4 instr/bit
counted program. SM1 handles TMS synchronized to SM0's TCK.
Removes backend abstraction layer (direct PIOLib calls).
Loopback mode uses single SM for DMA throughput benchmarking."
```

---

## Task 3: Update PIO simulator tests

The PIO simulator tests verify PIO program behavior without hardware. Update
them for the new programs.

**Files:**
- Rewrite: `tests/pio_sim/test_jtag_shift.c`
- Create: `tests/pio_sim/test_jtag_tms.c`
- Delete: `tests/pio_sim/test_jtag_shift_fast.c`
- Update: `tests/pio_sim/CMakeLists.txt`

### Step 1: Update test_jtag_shift.c

Update tests for the new 4 instr/bit program:
- Verify TCK waveform: LOW-LOW-HIGH-LOW per bit (was LOW-LOW-HIGH-HIGH-LOW)
- Verify TDI setup timing: TDI set 2 cycles before TCK rise
- Verify TDO sampling: on TCK rising edge (cycle 2 of 4)
- Verify count-based termination and push

### Step 2: Create test_jtag_tms.c

Test the jtag_tms.pio program via PIO simulator:
- Verify it reads TMS bits from TX FIFO
- Verify it outputs to OUT pin (TMS)
- Verify synchronization via GPIO wait (simulate TCK toggling)
- Verify it stalls when TCK stops toggling

### Step 3: Remove test_jtag_shift_fast.c

Delete `tests/pio_sim/test_jtag_shift_fast.c` (program no longer exists).

### Step 4: Update CMakeLists.txt

Replace `test_jtag_shift_fast` with `test_jtag_tms` in the test list.

### Step 5: Run tests

```bash
cmake -B build && cmake --build build
ctest --test-dir build -R pio_sim -V
```

Expected: All PIO simulator tests pass.

### Step 6: Commit

```
git add tests/pio_sim/
git rm tests/pio_sim/test_jtag_shift_fast.c
git commit -m "Update PIO simulator tests for new two-SM programs

New test_jtag_tms.c verifies SM1 TMS synchronization via GPIO wait.
Updated test_jtag_shift.c for 4 instr/bit loop (was 5). Removed
test_jtag_shift_fast.c (program no longer exists)."
```

---

## Task 4: Update unit tests

The unit tests verify the library's data handling without hardware. Since we
removed the mock backend, we need to restructure these tests.

**Files:**
- Keep and update: `tests/unit/test_data_packing.c`
- Rewrite: `tests/unit/test_tms_splitting.c` (remove — TMS splitting no longer exists)
- Rewrite: `tests/unit/test_fifo_interleave.c` (remove — FIFO interleaving no longer exists)
- Create: `tests/unit/test_dma_buffer_packing.c`
- Update: `tests/unit/CMakeLists.txt`

### Step 1: Keep test_data_packing.c

The `bits_to_words()` and `words_to_bits()` functions are preserved. These tests
should still pass. Verify the functions are still exported or accessible from the
test. May need to extract them to a separate utility file or make them non-static.

### Step 2: Create test_dma_buffer_packing.c

Test the DMA buffer packing logic:
- SM0 TX buffer: verify count word is `num_bits - 1`, followed by TDI words
- SM1 TX buffer: verify TMS words are packed correctly
- SM0 RX buffer: verify unpacking with partial-word alignment fix
- Various bit counts: 1, 8, 32, 33, 64, 100, 1024

### Step 3: Remove old tests that no longer apply

- `test_tms_splitting.c` (TMS splitting removed — SM1 handles TMS directly)
- `test_fifo_interleave.c` (FIFO interleaving removed — DMA handles transfer)

### Step 4: Update CMakeLists.txt

Replace old test targets with new ones. Since the mock backend is gone,
unit tests that need library internals should either:
- Test standalone utility functions (data packing)
- Test via the public API with `rp1_jtag_init_loopback()` on real hardware

### Step 5: Run tests

```bash
cmake -B build && cmake --build build
ctest --test-dir build -R unit -V
```

### Step 6: Commit

```
git add tests/unit/
git rm tests/unit/test_tms_splitting.c tests/unit/test_fifo_interleave.c
git commit -m "Update unit tests for DMA architecture

New test_dma_buffer_packing.c verifies SM0/SM1 TX buffer layout and
RX unpacking. Removed test_tms_splitting.c (no more TMS splitting)
and test_fifo_interleave.c (no more FIFO interleaving). Kept
test_data_packing.c (bit vector utilities unchanged)."
```

---

## Task 5: Update hardware tests

**Files:**
- Update: `tests/hardware/test_pio_loopback.c` (use DMA loopback)
- Update: `tests/hardware/test_gpio_loopback.c` (use two-SM JTAG)
- Update: `tests/hardware/test_idcode.c` (use two-SM JTAG)
- Create: `tests/hardware/bench_dma_loopback.c` (DMA throughput benchmark)
- Update: `tests/hardware/CMakeLists.txt`
- Update: `tests/hardware/test_dma_explore.c` (may keep for reference)

### Step 1: Rewrite test_pio_loopback.c

Use `rp1_jtag_init_loopback()` for DMA throughput test. Verify:
- Data integrity (send known pattern, receive identical pattern)
- Throughput measurement (time the transfer, report MB/s)
- Various transfer sizes (1 KB, 64 KB, 256 KB)

### Step 2: Create bench_dma_loopback.c

Full DMA throughput benchmark following the rpi5-rp1-pio-bench style:
- Configurable transfer size and iterations
- Warmup iterations
- Statistical report (min/max/mean/median/stddev)
- Data verification
- Pass/fail threshold

### Step 3: Update test_gpio_loopback.c

Use `rp1_jtag_init()` and `rp1_jtag_shift()` with the two-SM architecture.
Test with TDI→TDO jumper wire (GPIO27→GPIO22):
- Send known TDI pattern with TMS=0
- Verify received TDO matches TDI (shifted by 1 bit due to JTAG scan chain)
- Test at various frequencies (1 MHz, 10 MHz, 33 MHz)

### Step 4: Update test_idcode.c

Read NeTV2 FPGA IDCODE using the new two-SM JTAG:
- Navigate TAP to Shift-DR via TMS
- Read 32-bit IDCODE
- Verify: 0x13631093 (XC7A100T)

### Step 5: Update CMakeLists.txt

Add bench_dma_loopback, remove test_dma_explore if not needed.

### Step 6: Commit

```
git add tests/hardware/
git commit -m "Update hardware tests for DMA two-SM architecture

New bench_dma_loopback.c for DMA throughput benchmarking.
Updated test_pio_loopback.c for DMA loopback mode.
Updated test_gpio_loopback.c and test_idcode.c for two-SM JTAG."
```

---

## Task 6: Update openFPGALoader driver

**Files:**
- Update: `drivers/openfpgaloader/rp1PioJtag.cpp`
- Update: `drivers/openfpgaloader/rp1PioJtag.hpp` (if needed)

### Step 1: Verify driver compatibility

The openFPGALoader driver uses only the public API (`rp1_jtag_init`, `rp1_jtag_shift`,
etc.). Since the API is preserved, the driver should work without changes.

Review the driver to confirm no internal dependencies were broken.

### Step 2: Minor cleanup if needed

If the driver references any removed internal headers or functions, update it.

### Step 3: Commit (only if changes needed)

```
git add drivers/
git commit -m "Update openFPGALoader driver for DMA architecture"
```

---

## Task 7: Build and verify on x86 (no hardware)

### Step 1: Build without PIOLib

```bash
cmake -B build -DHAVE_PIOLIB=OFF
cmake --build build
```

Should compile cleanly with stub functions.

### Step 2: Run simulator and unit tests

```bash
ctest --test-dir build -V
```

All PIO simulator and data packing tests should pass.

### Step 3: Fix any build/test issues

### Step 4: Commit any fixes

---

## Task 8: Test on RPi 5 hardware (loopback)

### Step 1: Copy code and build on RPi 5

```bash
scp -r . tim@10.1.10.14:~/rp1-jtag-dma/
ssh tim@10.1.10.14
cd ~/rp1-jtag-dma
cmake -B build && cmake --build build
```

### Step 2: Run PIO loopback benchmark

```bash
sudo ./build/tests/hardware/bench_dma_loopback --size=262144 --iterations=20
```

Expected: ~42 MB/s aggregate throughput (matching rpi5-rp1-pio-bench).

### Step 3: Run GPIO loopback test (1 jumper: TDI→TDO)

```bash
sudo ./build/tests/hardware/test_gpio_loopback
```

Expected: Data integrity pass at various TCK frequencies.

### Step 4: Document results and fix issues

---

## Task 9: Test on RPi 5 hardware (real FPGA)

### Step 1: Run IDCODE test

```bash
sudo ./build/tests/hardware/test_idcode
```

Expected: IDCODE = 0x13631093 (XC7A100T).

### Step 2: Test with openFPGALoader

Build a static openFPGALoader binary with the new librp1jtag and test:

```bash
sudo openFPGALoader --cable rp1pio --pins 27:22:4:17 --detect
```

Expected: Detects XC7A100T.

### Step 3: Program FPGA via openFPGALoader

```bash
sudo openFPGALoader --cable rp1pio --pins 27:22:4:17 --write-sram bitstream.bit
```

Expected: Programs successfully.

### Step 4: Compare performance with RPi 3

Run the same programming operation on RPi 3 using libgpiod and time both:

```bash
# RPi 5 (rp1pio cable, DMA)
time sudo openFPGALoader --cable rp1pio --pins 27:22:4:17 --write-sram bitstream.bit

# RPi 3 (libgpiod cable, GPIO bitbang)
time sudo openFPGALoader --cable libgpiod --pins 27:22:4:17 --write-sram bitstream.bit
```

Expected: RPi 5 is significantly faster (10x+).

### Step 5: Commit test results

```
git add benchmarks/
git commit -m "Add DMA benchmark results: RPi 5 vs RPi 3 performance"
```

---

## Task 10: Update CI workflow

**Files:**
- Update: `.github/workflows/static-openfpgaloader.yml`

### Step 1: Verify CI still builds

The static build workflow builds librp1jtag and links it into openFPGALoader.
Push the branch and verify all CI jobs pass (arm64, armv7, armv6).

### Step 2: Fix any CI issues

The main concern: pthread linking in the static build. Alpine musl provides
pthreads in libc, so `-lpthread` should work. Verify.

### Step 3: Commit any CI fixes

---

## Summary

| Task | Description | Tests | Hardware needed? |
|------|-------------|-------|-----------------|
| 1 | Write new PIO programs | Assemble + verify headers | No |
| 2 | Rewrite core library | Compile check | No |
| 3 | Update PIO simulator tests | `ctest -R pio_sim` | No |
| 4 | Update unit tests | `ctest -R unit` | No |
| 5 | Update hardware tests | Manual on RPi 5 | Yes |
| 6 | Update openFPGALoader driver | Build check | No |
| 7 | Build + test on x86 | `ctest` | No |
| 8 | Test on RPi 5 (loopback) | sudo on RPi 5 | Yes |
| 9 | Test on RPi 5 (real FPGA) | sudo on RPi 5 | Yes |
| 10 | Update CI workflow | GitHub Actions | No |
