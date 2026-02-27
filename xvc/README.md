# rp1-xvc — Xilinx Virtual Cable Daemon

XVC (Xilinx Virtual Cable) is a TCP-based protocol that allows Vivado Hardware Manager
to communicate with JTAG devices over a network. `rp1-xvc` implements an XVC v1.0
server using librp1jtag for high-speed JTAG via the Raspberry Pi 5 RP1 PIO.

## Usage

```bash
# Start the XVC server (requires root for /dev/pio0)
sudo ./build/xvc/rp1-xvc --pins 27:22:4:17

# With custom port and frequency
sudo ./build/xvc/rp1-xvc --pins 27:22:4:17 --port 2542 --freq 6000000

# Verbose mode (log each XVC command)
sudo ./build/xvc/rp1-xvc --pins 27:22:4:17 --verbose
```

Pin format: `TDI:TDO:TCK:TMS` (BCM GPIO numbers, matching openFPGALoader convention).

## Connecting from Vivado

1. Start `rp1-xvc` on the Raspberry Pi 5
2. In Vivado: **Open Hardware Manager** -> **Open Target** -> **Open New Target**
3. Select **Add Xilinx Virtual Cable**
4. Enter the RPi 5 hostname or IP and port (default: 2542)
5. The FPGA should appear in the hardware tree

## Protocol

Implements XVC v1.0 with three commands:

| Command | Request | Response |
|---------|---------|----------|
| `getinfo:` | (none) | `xvcServer_v1.0:<max_vector_len>\n` |
| `settck:` | 4-byte LE period (ns) | 4-byte LE actual period (ns) |
| `shift:` | 4-byte LE num_bits + TMS bytes + TDI bytes | TDO bytes |

All integers are little-endian uint32. Bit vectors are LSB-first byte arrays.

Maximum vector length: 262144 bits (32768 bytes).

## Limitations

- Single client only (one connection at a time)
- XVC v1.0 protocol (no `mrd`/`mwr` memory access commands)
- No SRST/TRST pin support over XVC
- Requires root access for PIO hardware

## Verification Test

A Python test script verifies basic XVC functionality by reading the FPGA IDCODE:

```bash
# Start the server in one terminal
sudo ./build/xvc/rp1-xvc --pins 27:22:4:17

# Run the test in another terminal
python3 xvc/test_xvc_idcode.py
```

## Future Enhancements

- **Multi-client support**: Allow multiple simultaneous Vivado connections with JTAG access serialisation
- **XVC v1.1**: Add `mrd`/`mwr` commands for direct memory-mapped register access
- **systemd service**: Unit file for automatic startup
- **IPv6 support**: Listen on both IPv4 and IPv6
- **Access control**: Bind address restriction, connection logging
