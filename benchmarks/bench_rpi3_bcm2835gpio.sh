#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Benchmark: RPi 3B+ bcm2835gpio bitstream load via OpenOCD
# Run on rpi3-netv2 as root
#
# Usage: sudo bash bench_rpi3_bcm2835gpio.sh

SCRIPTS=/home/pi/code/netv2mvp-scripts
IMAGES=/home/pi/code/netv2-fpga/production-images

echo "=== Detecting FPGA ==="
sudo openocd -f "$SCRIPTS/idcode.cfg" 2>&1

# Determine FPGA variant and select bitstream
# XC7A100T IDCODE: 0x13631093, XC7A35T IDCODE: 0x0362d093
BITFILE="$IMAGES/user-35.bit"
echo ""
echo "=== Loading bitstream: $BITFILE ==="
ls -l "$BITFILE"

for i in 1 2 3; do
    echo ""
    echo "=== Run $i ==="
    time sudo openocd \
        -f "$SCRIPTS/alphamax-rpi.cfg" \
        -c "source [find cpld/xilinx-xc7.cfg]" \
        -c "init" \
        -c "pld load 0 $BITFILE" \
        -c "exit"
done
