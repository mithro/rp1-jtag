#!/bin/bash
# netv2_spiflash.sh — SPI flash utility for NeTV2 via RP1 PIO JTAG
#
# Scans for SPI flash using bscan_spi proxy bitstream, then optionally
# writes a bitstream to flash at offset 0 and reboots the FPGA.
#
# Usage:
#   sudo ./netv2_spiflash.sh scan
#   sudo ./netv2_spiflash.sh flash <bitstream.bin>
#   sudo ./netv2_spiflash.sh read  <output.bin> [offset] [size]
#
# Requires: OpenOCD built with --enable-rp1-pio-jtag, bscan_spi_xc7a35t.bit

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# --- Configuration ---
OPENOCD="${OPENOCD:-/home/k8s/openocd/src/openocd}"
OPENOCD_TCL="${OPENOCD_TCL:-/home/k8s/openocd/tcl}"
BSCAN_BIT="${BSCAN_BIT:-/home/k8s/netv2mvp-scripts/bscan_spi_xc7a35t.bit}"
CFG="${SCRIPT_DIR}/netv2_35t_spi.cfg"

# --- Helpers ---
die()  { echo "ERROR: $*" >&2; exit 1; }
info() { echo ">>> $*"; }

cleanup_pio() {
    local pids
    pids="$(sudo lsof -t /dev/pio0 2>/dev/null || true)"
    if [ -n "$pids" ]; then
        info "Killing stale PIO processes: $pids"
        sudo kill -9 $pids 2>/dev/null || true
        sleep 0.3
    fi
}

run_openocd() {
    # Run OpenOCD with the SPI config and arbitrary -c commands
    sudo "$OPENOCD" -s "$OPENOCD_TCL" -f "$CFG" "$@" 2>&1
}

# --- Preflight checks ---
[ -x "$OPENOCD" ]  || die "OpenOCD not found at $OPENOCD"
[ -f "$BSCAN_BIT" ] || die "bscan_spi bitstream not found at $BSCAN_BIT"
[ -f "$CFG" ]       || die "Config file not found at $CFG"
[ "$(id -u)" -eq 0 ] || die "Must run as root (sudo)"

# --- Commands ---
cmd_scan() {
    info "Scanning SPI flash on NeTV2 (XC7A35T) via RP1 PIO JTAG..."
    cleanup_pio
    run_openocd \
        -c "init" \
        -c "jtagspi_init xc7.pld $BSCAN_BIT" \
        -c "flash info 0" \
        -c "flash list" \
        -c "shutdown"
}

cmd_flash() {
    local bitstream="$1"
    [ -f "$bitstream" ] || die "Bitstream file not found: $bitstream"

    local size
    size="$(stat -c%s "$bitstream")"
    info "Flashing $bitstream ($size bytes) to SPI flash at offset 0..."
    info "  bscan_spi: $BSCAN_BIT"

    cleanup_pio
    run_openocd \
        -c "init" \
        -c "jtagspi_init xc7.pld $BSCAN_BIT" \
        -c "jtagspi_program $bitstream 0" \
        -c "virtex2 refresh xc7.pld" \
        -c "shutdown"

    info "Flash complete. FPGA rebooted from SPI."
}

cmd_read() {
    local output="$1"
    local offset="${2:-0}"
    local size="${3:-256}"

    info "Reading $size bytes from SPI flash at offset $offset..."
    cleanup_pio
    run_openocd \
        -c "init" \
        -c "jtagspi_init xc7.pld $BSCAN_BIT" \
        -c "flash read_bank 0 $output $offset $size" \
        -c "shutdown"

    info "Read complete: $output"
}

# --- Main ---
case "${1:-}" in
    scan)
        cmd_scan
        ;;
    flash)
        [ -n "${2:-}" ] || die "Usage: $0 flash <bitstream.bin>"
        cmd_flash "$2"
        ;;
    read)
        [ -n "${2:-}" ] || die "Usage: $0 read <output.bin> [offset] [size]"
        cmd_read "$2" "${3:-0}" "${4:-256}"
        ;;
    *)
        echo "Usage: $0 {scan|flash|read} [args...]"
        echo ""
        echo "Commands:"
        echo "  scan                          Detect SPI flash via JTAG"
        echo "  flash <bitstream.bin>         Erase+write+verify bitstream to SPI"
        echo "  read  <out.bin> [off] [size]  Read bytes from SPI flash"
        exit 1
        ;;
esac
