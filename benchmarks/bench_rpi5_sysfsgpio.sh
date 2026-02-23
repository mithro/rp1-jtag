#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Benchmark: RPi 5 sysfsgpio bitstream load via OpenOCD
# Run on rpi5-netv2 as root
#
# Usage: sudo bash bench_rpi5_sysfsgpio.sh

CFG=/home/tim/github/mithro/rp1-jtag/benchmarks/rpi5-sysfsgpio.cfg
BITFILE=/home/tim/netv2/user-100.bit

echo "=== Detecting FPGA ==="
sudo openocd -f "$CFG" \
    -c "source [find cpld/xilinx-xc7.cfg]" \
    -c "init" \
    -c "scan_chain" \
    -c "exit" 2>&1

echo ""
echo "=== Loading bitstream: $BITFILE ==="
ls -l "$BITFILE"

echo ""
echo "NOTE: sysfsgpio on RPi 5 is extremely slow (~5 kB/s)."
echo "A 3.8 MB bitstream may take 10+ minutes per run."
echo ""

for i in 1 2 3; do
    echo "=== Run $i ==="
    time sudo openocd -f "$CFG" \
        -c "source [find cpld/xilinx-xc7.cfg]" \
        -c "init" \
        -c "pld load 0 $BITFILE" \
        -c "exit"
    echo ""
done
