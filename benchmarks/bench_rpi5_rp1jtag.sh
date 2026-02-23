#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Benchmark: RPi 5 rp1-jtag (DMA + fast PIO) bitstream load via openFPGALoader
# Run on rpi5-netv2 as root
#
# Usage: sudo bash bench_rpi5_rp1jtag.sh

OFPGAL=/home/tim/github/mithro/rp1-jtag/tmp/openFPGALoader/build/openFPGALoader
BITFILE=/home/tim/netv2/user-100.bit
PINS="27:22:4:17"

echo "=== Detecting FPGA ==="
sudo "$OFPGAL" -c rp1pio --pins "$PINS" --detect

echo ""
echo "=== Loading bitstream: $BITFILE ==="
ls -l "$BITFILE"
echo ""

for i in 1 2 3; do
    echo "=== Run $i ==="
    time sudo "$OFPGAL" -c rp1pio --pins "$PINS" "$BITFILE"
    echo ""
done
