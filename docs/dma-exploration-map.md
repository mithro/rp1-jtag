# Map of the fast-JTAG exploration branches

Written 2026-10-04. `main` ships the slow, correct path (`use_dma = false`,
word-by-word FIFO access). Everything that tried to go faster lives on the
branches below. This page says what each branch is, what it claimed when it
was written, and what has been re-measured since.

## Topology

```
main
 ├── feature/xvc-daemon                      Xilinx Virtual Cable server (independent of the rest)
 └── feature/jtag-benchmarks                 benchmark scripts + benchmarks/RESULTS.md baseline
      └── perf/01-fast-shift-tuning
           └── perf/02-dma-bidi-threaded
                └── perf/03-dma-two-sm
                     └── perf/04-dma-bulk-single-sm
                          └── perf/05-dma-sm0-only
                               └── perf/06-dma-sm-reset-fix
                                    └── perf/07-dma-continuous-concurrent
                                         └── perf/08-fifo-write-count-word
                                              └── perf/09-dma-hybrid-streaming
                                                   └── perf/10-dma-interleaved-streaming
                                                        └── perf/11-dma-txonly-streaming
                                                             └── perf/12-jtag-stream-joined-fifo
```

The `perf/*` branches are **stacked, not independent**. Each attempt rewrote
the same shift functions in `lib/src/rp1_jtag.c` on top of the one before it,
and each appended its measurements to `benchmarks/RESULTS.md`, which the
benchmarks branch creates. A branch therefore contains every earlier attempt;
`git diff perf/NN-1..perf/NN` shows exactly what attempt NN changed.

## The attempts

"Claimed" is what the commit messages and `benchmarks/RESULTS.md` had
recorded about the attempt by the end of the line of work (February and
March 2026, Pi 5 `rpi5-netv2`, NeTV2 XC7A100T, 3.8 MB `user-100.bit`, on a
kernel carrying raspberrypi/linux PR #7190, which was still open on
2026-10-04). "Re-measured" is from 2026-09-22 on the same
board with stock kernel 6.12.47 unless it says otherwise. An empty cell means
nobody has re-measured that state.

| Branch | Idea | Claimed | Re-measured |
|---|---|---|---|
| `perf/01-fast-shift-tuning` | Keep the state machine running across chunks, 32 KB chunks, 10 MHz default | About 6.2 s (best 6.21 s, from 6.53 s before) | |
| `perf/02-dma-bidi-threaded` | TX and RX DMA from two threads | Failed: timeouts; left switched off | |
| `perf/03-dma-two-sm` | Rewrite: TDI/TDO/TCK on SM0, TMS on SM1, DMA on both | Works after three fixes | |
| `perf/04-dma-bulk-single-sm` | Constant-TMS shifts on SM0 only, chunked with a restart per chunk | 3.5 s at 10 MHz | |
| `perf/05-dma-sm0-only` | Drop SM1; DMACTRL threshold 4; 30720-bit chunks | 0.96 s at 20 MHz, DONE=1. **Later withdrawn**: the BYPASS loopback integrity test fails, DONE was a false positive | **Never reaches DONE** (status `0x4000190c`), at any TCK from 1 to 20 MHz, through openFPGALoader and OpenOCD, on kernels 6.12.47 and 6.18.50 |
| `perf/06-dma-sm-reset-fix` | Sequential TX then RX, 224-bit chunks | 14.4 s, DONE=1, 29/29 BYPASS sizes | 14.6 s DONE=1 (6.12.47); 11.1 s DONE=1 (6.18.50) |
| `perf/07-dma-continuous-concurrent` | Continuous state machine, concurrent TX+RX DMA | DONE=0, data corruption | |
| `perf/08-fifo-write-count-word` | No DMA: count word plus `pio_sm_put_blocking` per word | 9.42 s at any TCK, DONE=1 | |
| `perf/09-dma-hybrid-streaming` | Streaming DMA for whole words, FIFO write for the tail | 6.1 s, DONE=1 | |
| `perf/10-dma-interleaved-streaming` | TX then RX DMA in 32-byte chunks | 5.9 to 6.1 s, DONE=1, 29/29 BYPASS | 5.9 s DONE=1 (6.12.47); **DONE=0 on 6.18.50** |
| `perf/11-dma-txonly-streaming` | TX-only DMA with autopush off; 32-bit alignment buffer in the openFPGALoader driver; chunks of 16 then 28 bytes | 3.58 s at 10 MHz, DONE=1, 29/29 BYPASS | |
| `perf/12-jtag-stream-joined-fifo` | Two-instruction `jtag_stream` PIO program, joined 16-word TX FIFO, 4096-byte DMA chunks | 1.75 s at 20 MHz, 3.27 s at 10 MHz, DONE=1, 29/29 BYPASS | |

## What is not explained yet

1. **Concurrent TX+RX DMA fails in this library** (timeouts in attempt 02,
   corrupted data in attempts 05 and 07) while `rpi5-rp1-pio-bench` runs
   concurrent DMA at 42 MB/s on the same board. The difference between the
   two has never been found. Treat it as a bug in this library until proven
   otherwise.
2. **Attempt 11 states that the RP1 DMA controller ignores DREQ pacing on PIO
   TX writes** and works around it by keeping each chunk (16, then 28 bytes)
   smaller than the 8-word FIFO. Attempt 12 then sends 4096-byte chunks into
   a 64-byte joined FIFO and reports DONE=1 with the integrity test passing,
   and the reference benchmark sends 256 KB per call. Both contradict the
   attempt 11 statement, so what really went wrong with larger chunks in
   attempt 11 is unknown.
3. **Kernel 6.18.50 fails the reference benchmark's own data check** and
   turns attempt 10's DONE=1 into DONE=0. Whether that is a kernel
   regression or a configuration difference has not been established.
4. **IDCODE reads fail above 21 MHz TCK** with a clean one-bit shift, which
   points at the TDO sampling instant in the PIO program.
5. The state machine was measured running at twice the TCK the divider
   formula predicts.

## How to measure a branch

1. Characterise the kernel first with `rpi5-rp1-pio-bench`.
2. Run the BYPASS loopback test (`tests/hardware/test_bypass_loopback`); a
   branch that fails it is wrong, whatever DONE says.
3. Load a bitstream and read DONE from openFPGALoader's status output, not
   from its exit code.
4. Record kernel version, TCK, wall time and DONE in `benchmarks/RESULTS.md`
   on the branch measured.

## Where the branches came from

They are a split, made on 2026-10-04, of one 65-commit line of work that sat
in a local branch and on the test board. Each commit was cherry-picked
(`-x`, so every commit names its original) onto `main`; at every `perf/*` tip
`lib/src` and `tests` are byte-identical to the original line at the same
point. (`feature/xvc-daemon` and `feature/jtag-benchmarks` differ from it
only by `main`'s `use_dma = false` change, which the original line never
had.) The unsplit original is kept as
`wip/2026-10-04-desktop-streaming-dma-lineage`. Three older preservation
branches remain: `wip/2026-10-03-rpi5-netv2-jtag-stream` and
`wip/2026-10-03-rpi5-netv2-stash` (uncommitted states found on the test
board; the first is an earlier copy of what attempts 11 and 12 commit), and
`wip/2026-02-24-bidi-dma-exploration` (uncommitted experiments behind
attempt 02).

Five commits of the original line are not on any split branch because they
were not about JTAG speed: two that added working-method notes to
`CLAUDE.md`, one `.gitignore` entry, and an `hw.md` change together with its
revert.
