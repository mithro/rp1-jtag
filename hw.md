# Hardware Setup

There are two Raspberry Pi devices that you are able to access:

---

# rpi5-netv2.iot.welland.mithis.com

Raspberry Pi 5 Model B Rev 1.0 with a NeTV2 FPGA board connected via JTAG.

Login: `ssh tim@rpi5-netv2.iot.welland.mithis.com` (or `tim@10.1.10.14`)

You need to ssh into the device as the user `tim` and you have root access via
`sudo`.

| Property         | Value                                               |
|------------------|-----------------------------------------------------|
| SoC              | BCM2712 (4x Cortex-A76 @ 2.4 GHz)                  |
| RAM              | 4 GB LPDDR4X                                        |
| OS               | Debian GNU/Linux 13 (trixie)                        |
| Kernel           | 6.12.47+rpt-rpi-2712 (aarch64)                     |
| GPIO controller  | RP1 southbridge (via PCIe 2.0 x4)                   |
| PIO              | 1 instance (`/dev/pio0`), 4 state machines, 200 MHz |
| Serial           | `/dev/ttyAMA10`                                     |
| SPI              | `/dev/spidev10.0`                                   |
| I2C              | `/dev/i2c-13`, `/dev/i2c-14`                        |
| OpenOCD          | 0.12.0+dev-snapshot (2025-07-16)                    |
| openFPGALoader   | `/home/tim/openFPGALoader-src/build/openFPGALoader` |
| FPGA             | Xilinx Artix-7 XC7A100T (IDCODE `0x13631093`)      |
| Bitstream        | `~/rp1-jtag/tmp/user-100.bit` (3.8 MB)             |
| librp1jtag       | `/usr/local/lib/librp1jtag.so.0`                    |

## JTAG Pin Mapping

| JTAG Signal | BCM GPIO | RPi 40-pin Header |
|-------------|----------|--------------------|
| TCK         | GPIO4    | Pin 7              |
| TMS         | GPIO17   | Pin 11             |
| TDI         | GPIO27   | Pin 13             |
| TDO         | GPIO22   | Pin 15             |

## JTAG Tool Usage

### openFPGALoader (rp1-jtag PIO driver)

```bash
# Install/update library before testing
sudo cp ~/rp1-jtag/build/lib/librp1jtag.so.0 /usr/local/lib/ && sudo ldconfig

# Reset PIO module (always do this before testing)
sudo rmmod rp1_pio && sudo modprobe rp1_pio

# Program bitstream (--pins format: TDI:TDO:TCK:TMS)
sudo /home/tim/openFPGALoader-src/build/openFPGALoader \
    -c rp1pio --pins 27:22:4:17 --freq 10000000 \
    --write-sram ~/rp1-jtag/tmp/user-100.bit
```

### OpenOCD (sysfsgpio)

```bash
sudo openocd \
    -f interface/sysfsgpio-raspberrypi.cfg \
    -c "sysfsgpio_tck_num 4; sysfsgpio_tms_num 17; sysfsgpio_tdi_num 27; sysfsgpio_tdo_num 22" \
    -c "source [find cpld/xilinx-xc7.cfg]" \
    -c "init" -c "scan_chain" -c "exit"
```

## Hardware Test Commands

```bash
# Hardware tests (need sudo for /dev/pio0)
sudo ~/rp1-jtag/build/tests/hardware/test_pio_loopback     # No wiring needed
sudo ~/rp1-jtag/build/tests/hardware/test_gpio_loopback     # 1 jumper: TDI->TDO
sudo ~/rp1-jtag/build/tests/hardware/test_target_loopback   # 4 jumpers
sudo ~/rp1-jtag/build/tests/hardware/test_idcode            # Needs NeTV2
sudo ~/rp1-jtag/build/tests/hardware/test_bypass_loopback   # Needs NeTV2 (gold standard)
```

---

# rpi3-netv2.iot.welland.mithis.com

Raspberry Pi 3 Model B Plus Rev 1.3 running the stock NeTV2 firmware image.

Login: `ssh pi@ipv4.eth0.rpi3-netv2.iot.welland.mithis.com`

You need to ssh into the device as the user `pi` and you have root access via
`sudo`.

| Property         | Value                                               |
|------------------|-----------------------------------------------------|
| SoC              | BCM2837B0 (4x Cortex-A53 @ 1.4 GHz)                |
| RAM              | 927 MB LPDDR2                                       |
| OS               | Raspbian GNU/Linux 9 (stretch)                      |
| Kernel           | 4.14.71-v7+ (armv7l)                                |
| GPIO controller  | BCM2837 (direct memory-mapped, base `0x3F000000`)   |
| PIO              | None (BCM2837 has no PIO block)                     |
| Serial           | `/dev/ttyAMA0`                                      |
| SPI              | None enabled                                        |
| I2C              | None enabled                                        |
| OpenOCD          | 0.10.0 (2018-10-26, Alphamax fork)                  |
| FPGA             | Xilinx Artix-7 XC7A35T (per config; check IDCODE)   |
| Bitstream        | `/home/pi/code/netv2-fpga/production-images/user-35.bit` |
| OpenOCD configs  | `/home/pi/code/netv2mvp-scripts/`                   |

## JTAG Pin Mapping

Same pin assignment as rpi5-netv2 (both connected to NeTV2 board):

| JTAG Signal | BCM GPIO | RPi 40-pin Header |
|-------------|----------|--------------------|
| TCK         | GPIO4    | Pin 7              |
| TMS         | GPIO17   | Pin 11             |
| TDI         | GPIO27   | Pin 13             |
| TDO         | GPIO22   | Pin 15             |

## JTAG Tool Usage

### OpenOCD (bcm2835gpio, 10 MHz)

```bash
# Read IDCODE
sudo openocd -f /home/pi/code/netv2mvp-scripts/idcode.cfg

# Program bitstream
sudo openocd \
    -f /home/pi/code/netv2mvp-scripts/alphamax-rpi.cfg \
    -c "source [find cpld/xilinx-xc7.cfg]" \
    -c "init" \
    -c "pld load 0 /home/pi/code/netv2-fpga/production-images/user-35.bit" \
    -c "exit"
```

### OpenOCD Config Details

The `alphamax-rpi.cfg` config uses:
- `interface bcm2835gpio` — direct memory-mapped GPIO (not sysfs)
- `bcm2835gpio_peripheral_base 0x3F000000` — RPi 2/3 peripheral base
- `bcm2835gpio_speed_coeffs 100000 5` — tuned for 10 MHz (oscilloscope-verified)
- `bcm2835gpio_jtag_nums 4 17 27 22` — TCK TMS TDI TDO
- `bcm2835gpio_srst_num 24` — reset pin
- `adapter_khz 10000` — 10 MHz target clock speed

Note: The Alphamax fork of OpenOCD has a GPIO drive strength patch
(`pads_base[...] = 0x5a000008 + 4` for 10 mA drive) to avoid signal
integrity issues at higher clock speeds.

---

# NeTV2 FPGA Board

Both Raspberry Pi devices are connected to a NeTV2 FPGA board via JTAG.
The NeTV2 is a Crowd Supply-funded video overlay board designed by bunnie
(Andrew Huang) at Alphamax.

## FPGA Variants

| Board     | FPGA Variant  | IDCODE         | Bitstream Size |
|-----------|---------------|----------------|----------------|
| rpi5-netv2| XC7A100T      | `0x13631093`   | ~3.8 MB        |
| rpi3-netv2| XC7A35T       | `0x0362d093`   | ~1.0 MB        |

The Artix-7 JTAG interface supports up to 66 MHz TCK. At the current
wiring lengths, reliable operation has been verified up to ~33 MHz
requested (which is ~66 MHz actual due to the 2x RP1 PIO clock factor).

## IDCODE Reference

| Part          | IDCODE         |
|---------------|----------------|
| XC7A35T       | `0x0362d093`   |
| XC7A100T      | `0x13631093`   |

---

# Performance Baselines

Measured bitstream programming times for the XC7A100T (3.8 MB) on rpi5-netv2:

| Method                              | Time   | Throughput | Notes                    |
|-------------------------------------|--------|------------|--------------------------|
| rp1-jtag TX-only streaming DMA      | 6.13s  | ~620 kB/s  | 16B DMA chunks, DONE=1   |
| rp1-jtag FIFO write (put_blocking)  | 9.4s   | ~404 kB/s  | ~957K ioctls, DONE=1     |
| rp1-jtag sequential DMA             | 14.4s  | ~264 kB/s  | 224-bit chunks, DONE=1   |
| OpenOCD sysfsgpio (RPi 5)           | 39s    | ~96 kB/s   | Kernel sysfs overhead    |

All PIO-based methods are ioctl-limited (~25 us per ioctl), not TCK-limited.
