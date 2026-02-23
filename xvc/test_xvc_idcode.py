#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
XVC IDCODE verification test.

Connects to an rp1-xvc server, resets the JTAG TAP, navigates to
Shift-DR, reads 32 bits of IDCODE, and verifies the result.

Usage:
    # Start the XVC server first:
    sudo ./build/xvc/rp1-xvc --pins 27:22:4:17 --verbose

    # Then run this test:
    python3 xvc/test_xvc_idcode.py [host] [port]
"""

import socket
import struct
import sys


def xvc_getinfo(sock):
    """Send getinfo: command, return response string."""
    sock.sendall(b"getinfo:")
    resp = b""
    while not resp.endswith(b"\n"):
        chunk = sock.recv(256)
        if not chunk:
            raise ConnectionError("Disconnected during getinfo")
        resp += chunk
    return resp.decode().strip()


def xvc_settck(sock, period_ns):
    """Send settck: command, return actual period in nanoseconds."""
    sock.sendall(b"settck:" + struct.pack("<I", period_ns))
    resp = sock.recv(4)
    if len(resp) != 4:
        raise ConnectionError("Short settck response")
    return struct.unpack("<I", resp)[0]


def xvc_shift(sock, num_bits, tms_bytes, tdi_bytes):
    """Send shift: command, return TDO bytes."""
    byte_count = (num_bits + 7) // 8
    assert len(tms_bytes) == byte_count
    assert len(tdi_bytes) == byte_count
    sock.sendall(b"shift:" + struct.pack("<I", num_bits) + tms_bytes + tdi_bytes)

    tdo = b""
    while len(tdo) < byte_count:
        chunk = sock.recv(byte_count - len(tdo))
        if not chunk:
            raise ConnectionError("Disconnected during shift response")
        tdo += chunk
    return tdo


def bits_to_bytes(bits):
    """Convert a list of bit values (LSB-first) to bytes."""
    byte_count = (len(bits) + 7) // 8
    result = bytearray(byte_count)
    for i, bit in enumerate(bits):
        if bit:
            result[i // 8] |= 1 << (i % 8)
    return bytes(result)


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "localhost"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 2542

    print(f"Connecting to {host}:{port}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))

    try:
        # Step 1: getinfo
        info = xvc_getinfo(sock)
        print(f"  getinfo: {info}")
        assert info.startswith("xvcServer_v1.0:"), f"Unexpected: {info}"

        # Step 2: settck (166 ns = ~6 MHz)
        actual_ns = xvc_settck(sock, 166)
        actual_mhz = 1000.0 / actual_ns if actual_ns > 0 else 0
        print(f"  settck: requested 166 ns, got {actual_ns} ns ({actual_mhz:.1f} MHz)")

        # Step 3: TAP reset (5 clocks with TMS=1)
        tms = bits_to_bytes([1, 1, 1, 1, 1])
        tdi = bits_to_bytes([0, 0, 0, 0, 0])
        xvc_shift(sock, 5, tms, tdi)
        print("  TAP reset (5x TMS=1)")

        # Step 4: Navigate to Shift-DR
        # From Test-Logic-Reset:
        #   TMS=0 -> Run-Test/Idle
        #   TMS=1 -> Select-DR-Scan
        #   TMS=0 -> Capture-DR
        #   TMS=0 -> Shift-DR
        tms = bits_to_bytes([0, 1, 0, 0])
        tdi = bits_to_bytes([0, 0, 0, 0])
        xvc_shift(sock, 4, tms, tdi)
        print("  Navigate to Shift-DR (TMS=0,1,0,0)")

        # Step 5: Read 32 bits of IDCODE (TMS=0 for all, stay in Shift-DR)
        tms = bytes(4)  # 32 bits of TMS=0
        tdi = bytes(4)  # 32 bits of TDI=0 (don't care)
        tdo = xvc_shift(sock, 32, tms, tdi)

        idcode = struct.unpack("<I", tdo)[0]
        print(f"  IDCODE: 0x{idcode:08x}")

        # Verify known IDCODEs
        known = {
            0x13631093: "XC7A100T",
            0x0362D093: "XC7A35T",
        }
        if idcode in known:
            print(f"  PASS: {known[idcode]}")
        else:
            print(f"  UNKNOWN IDCODE (expected XC7A100T=0x13631093 or XC7A35T=0x0362d093)")
            sys.exit(1)

    finally:
        sock.close()


if __name__ == "__main__":
    main()
