# Hardware Setup

There are four Raspberry Pi devices available for testing. The two NeTV2 devices
(rpi5-netv2, rpi3-netv2) are the **primary hardware** for JTAG development and
testing. The two Pmod devices (rpi5-pmod, rpi4-pmod) are **secondary hardware**
used for testing general IO performance.

Each device has a set of interfaces (eth0 and wlan0) with both IPv4 and IPv6.
To use a specific interface and IP you can use:

 * ipv4.eth0.\<hostname\> -- IPv4 address on eth0 interface.
 * ipv6.wlan0.\<hostname\> -- IPv6 address on wlan0 interface.
 * eth0.\<hostname\> -- Both IPv4 and IPv6 addresses.
 * \<hostname\> -- All IPv4 and IPv6 addresses.

You need to ssh into the devices as the user `tim` and you have root access via
`sudo`. The exception is rpi3-netv2, which uses the user `pi`.

---

# PRIMARY: rpi5-netv2.iot.welland.mithis.com

Raspberry Pi 5 Model B Rev 1.0 with a NeTV2 FPGA board connected via JTAG.

Login: `ssh tim@rpi5-netv2.iot.welland.mithis.com` (or `tim@10.1.10.14`)

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

# PRIMARY: rpi3-netv2.iot.welland.mithis.com

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
- `interface bcm2835gpio` -- direct memory-mapped GPIO (not sysfs)
- `bcm2835gpio_peripheral_base 0x3F000000` -- RPi 2/3 peripheral base
- `bcm2835gpio_speed_coeffs 100000 5` -- tuned for 10 MHz (oscilloscope-verified)
- `bcm2835gpio_jtag_nums 4 17 27 22` -- TCK TMS TDI TDO
- `bcm2835gpio_srst_num 24` -- reset pin
- `adapter_khz 10000` -- 10 MHz target clock speed

Note: The Alphamax fork of OpenOCD has a GPIO drive strength patch
(`pads_base[...] = 0x5a000008 + 4` for 10 mA drive) to avoid signal
integrity issues at higher clock speeds.

---

# NeTV2 FPGA Board

Both rpi5-netv2 and rpi3-netv2 are connected to a NeTV2 FPGA board via JTAG.
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

---

# SECONDARY: Other Available Hardware (IO Testing)

The following Pmod-based devices are primarily used for testing general IO
performance rather than JTAG. They provide a controlled environment for
benchmarking GPIO, SPI, I2C, UART, and other bus protocols between two
Raspberry Pi devices, with a Glasgow Interface Explorer available for
passive signal analysis.

---

## rpi5-pmod.iot.welland.mithis.com

Raspberry Pi 5 Model B Rev 1.1 with a Digilent Pmod HAT Adapter and a Glasgow
Interface Explorer (revC3) connected via USB.

Login: `ssh tim@rpi5-pmod.iot.welland.mithis.com`

| Property        | Value                                            |
|-----------------|--------------------------------------------------|
| SoC             | BCM2712 (4x Cortex-A76 @ 2.4 GHz)               |
| RAM             | 8 GB LPDDR4X                                     |
| OS              | Debian GNU/Linux 13 (trixie)                     |
| Kernel          | 6.12.47+rpt-rpi-2712 (aarch64)                  |
| GPIO controller | RP1 southbridge (via PCIe 2.0 x4)               |
| PIO             | 1 instance, 4 state machines, 200 MHz            |
| Serial          | `/dev/ttyAMA10`                                  |
| SPI             | `/dev/spidev10.0`                                |
| I2C             | `/dev/i2c-13`, `/dev/i2c-14`                     |
| HAT EEPROM      | Vendor: Digilent, Product: Pmod HAT Adaptor      |

### Glasgow Interface Explorer

A Glasgow revC3 (serial `C3-20230729T212534Z`) is connected via USB to the
rpi5-pmod. The Glasgow's 16 I/O pins (ports A and B, 8 pins each) are connected
to the RPi5 GPIO header via jumper wires.

| Property        | Value                                            |
|-----------------|--------------------------------------------------|
| Revision        | revC3                                            |
| Serial          | C3-20230729T212534Z                              |
| USB ID          | 20b7:9db1 (Qi Hardware)                          |
| Firmware        | API level 5                                      |
| I/O Ports       | A (8 pins), B (8 pins)                           |
| I/O Voltage     | Programmable, set to 3.3V for RPi GPIO           |
| Software        | Glasgow 0.1.dev2829 (installed via `uv tool`)    |

#### Glasgow CLI Usage

```bash
# Glasgow is installed as a uv tool
export PATH=$HOME/.local/bin:$PATH

# List connected devices
glasgow list

# Read all pins (high-impedance input)
glasgow run control-gpio -V 3.3 \
    --pins 'A0,A1,A2,A3,A4,A5,A6,A7,B0,B1,B2,B3,B4,B5,B6,B7' \
    A0 A1 A2 A3 A4 A5 A6 A7 B0 B1 B2 B3 B4 B5 B6 B7

# Drive a pin high (strong drive)
glasgow run control-gpio -V 3.3 --pins 'A0' A0=1

# Drive a pin low (strong drive)
glasgow run control-gpio -V 3.3 --pins 'A0' A0=0

# Weak pull-up / pull-down
glasgow run control-gpio -V 3.3 --pins 'A0' A0=H   # weak pull-up
glasgow run control-gpio -V 3.3 --pins 'A0' A0=L   # weak pull-down

# Capture logic analyser traces
glasgow run analyzer ...
```

**Note:** Glasgow commands require root access (`sudo`) or udev rules. A udev
rule is installed at `/etc/udev/rules.d/90-glasgow.rules` that grants access
to the `plugdev` group (which `tim` is a member of).

#### Glasgow-to-RPi GPIO Pin Mapping

The Glasgow I/O pins are connected to the RPi5 GPIO header via jumper wires.
The connections were discovered programmatically using `discover_glasgow_pins.py`.

##### Port A

| Glasgow Pin | BCM GPIO | RPi Header Pin | Pmod Pin  | Pmod Function              |
|-------------|----------|----------------|-----------|----------------------------|
| A0          | GPIO16   | Pin 36         | JC1       | UART0_CTS / SPI1_CE2      |
| A1          | GPIO20   | Pin 38         | JA9       | PCM_DIN / SPI1_MOSI       |
| A2          | GPIO21   | Pin 40         | JA8       | PCM_DOUT / SPI1_SCLK      |
| A3          | GPIO26   | Pin 37         | JB7       | GPIO                       |
| A4          | GPIO19   | Pin 35         | JA7       | PCM_FS / PWM1 / SPI1_MISO |
| A5          | GPIO13   | Pin 33         | JB8       | PWM1                       |
| A6          | GPIO6    | Pin 31         | JC10      | GPCLK2                     |
| A7          | GPIO5    | Pin 29         | JC9       | GPCLK1                     |

##### Port B

| Glasgow Pin | BCM GPIO | RPi Header Pin | Pmod Pin  | Pmod Function              |
|-------------|----------|----------------|-----------|----------------------------|
| B0          | GPIO14   | Pin 8          | JC2       | UART0_TXD                  |
| B1          | GPIO15   | Pin 10         | JC3       | UART0_RXD                  |
| B2          | GPIO18   | Pin 12         | JA10      | PCM_CLK / PWM0 / SPI1_CE0 |
| B3          | GPIO25   | Pin 22         | --        | (not on Pmod HAT)          |
| B4          | GPIO8    | Pin 24         | JA1       | SPI0_CE0                   |
| B5          | GPIO7    | Pin 26         | JB1       | SPI0_CE1                   |
| B6          | GPIO3    | Pin 5          | JB9       | I2C1_SCL (1.8kΩ pull-up)  |
| B7          | GPIO2    | Pin 3          | JB10      | I2C1_SDA (1.8kΩ pull-up)  |

##### Sorted by RPi Header Pin (physical wiring order)

| RPi Header Pin | BCM GPIO | Glasgow Pin | Pmod Pin  |
|----------------|----------|-------------|-----------|
| Pin 3          | GPIO2    | B7          | JB10      |
| Pin 5          | GPIO3    | B6          | JB9       |
| Pin 8          | GPIO14   | B0          | JC2       |
| Pin 10         | GPIO15   | B1          | JC3       |
| Pin 12         | GPIO18   | B2          | JA10      |
| Pin 22         | GPIO25   | B3          | --        |
| Pin 24         | GPIO8    | B4          | JA1       |
| Pin 26         | GPIO7    | B5          | JB1       |
| Pin 29         | GPIO5    | A7          | JC9       |
| Pin 31         | GPIO6    | A6          | JC10      |
| Pin 33         | GPIO13   | A5          | JB8       |
| Pin 35         | GPIO19   | A4          | JA7       |
| Pin 36         | GPIO16   | A0          | JC1       |
| Pin 37         | GPIO26   | A3          | JB7       |
| Pin 38         | GPIO20   | A1          | JA9       |
| Pin 40         | GPIO21   | A2          | JA8       |

##### Notes

- **GPIO25 (B3)** is connected to RPi header pin 22 but is **not** routed to
  any Pmod connector on the Pmod HAT. It is one of five "unused" GPIOs that
  the Pmod HAT leaves unconnected. The other four (GPIO22↔GPIO27 and
  GPIO23↔GPIO24) are wired as loopback pairs on the RPi5 header.
- **15 of the 16 Glasgow pins** are tapping signals that also go through the
  Pmod connectors to the inter-RPi jumper cables (to rpi4-pmod).
- **GPIO2/GPIO3 (B7/B6)** have 1.8kΩ hardware pull-up resistors on the RPi
  board itself (I2C1 bus). The Glasgow will see these as weakly pulled high
  when not actively driven.

---

## rpi4-pmod.iot.welland.mithis.com

Raspberry Pi 4 Model B Rev 1.5 with a Digilent Pmod HAT Adapter.

Login: `ssh tim@rpi4-pmod.iot.welland.mithis.com`

| Property        | Value                                            |
|-----------------|--------------------------------------------------|
| SoC             | BCM2711 (4x Cortex-A72 @ 1.8 GHz)               |
| RAM             | 4 GB LPDDR4                                      |
| OS              | Debian GNU/Linux 13 (trixie)                     |
| Kernel          | 6.12.47+rpt-rpi-v8 (aarch64)                    |
| GPIO controller | BCM2711 (direct memory-mapped)                   |
| PIO             | None (BCM2711 has no PIO block)                  |
| Serial          | None enabled (UART available via config.txt)     |
| SPI             | None enabled (SPI available via config.txt)      |
| I2C             | `/dev/i2c-20`, `/dev/i2c-21`                     |
| HAT EEPROM      | Vendor: Digilent, Product: Pmod HAT Adaptor      |

---

## Digilent Pmod HAT Adapter

The Digilent Pmod HAT Adapter (also branded as DesignSpark Pmod HAT, RS part 144-8419)
plugs into the Raspberry Pi 40-pin GPIO header and provides three 2x6-pin Pmod host
connectors: **JA**, **JB**, and **JC**.

Reference manual: https://digilent.com/reference/add-ons/pmod-hat/reference-manual

Key specifications:
- 3 x 2x6-pin (12-pin) Pmod host connectors
- 3.3V I/O on all Pmod pins (directly connected to RPi GPIOs, which are 3.3V)
- 16 mA current limit per Pmod GPIO
- 5V barrel jack for external power supply
- VCC on Pmod pins is 3.3V from the RPi's 3.3V rail

### GPIO Pin Mapping

The pin mapping below is from the
[DesignSpark.Pmod HAT.py source code](https://github.com/DesignSparkRS/DesignSpark.Pmod/blob/master/DesignSpark/Pmod/HAT.py),
which is the authoritative software definition. All GPIO numbers use BCM numbering.

#### Pmod Connector JA (SPI + GPIO)

JA top row (pins 1-6) carries SPI0 with chip select CE0.
JA bottom row (pins 7-12) carries PCM/PWM signals.

| Pmod Pin | BCM GPIO | RPi 40-pin Header | Alternate Function(s)          | Glasgow |
|----------|----------|--------------------|--------------------------------|---------|
| JA1      | GPIO8    | Pin 24             | **SPI0_CE0**                   | B4      |
| JA2      | GPIO10   | Pin 19             | **SPI0_MOSI**                  | --      |
| JA3      | GPIO9    | Pin 21             | **SPI0_MISO**                  | --      |
| JA4      | GPIO11   | Pin 23             | **SPI0_SCLK**                  | --      |
| JA5      | GND      | Pin 25 (GND)       | Ground                         | --      |
| JA6      | VCC      | 3.3V               | Power (3.3V)                   | --      |
| JA7      | GPIO19   | Pin 35             | PCM_FS / PWM1 / SPI1_MISO     | A4      |
| JA8      | GPIO21   | Pin 40             | PCM_DOUT / GPCLK1 / SPI1_SCLK | A2      |
| JA9      | GPIO20   | Pin 38             | PCM_DIN / GPCLK0 / SPI1_MOSI  | A1      |
| JA10     | GPIO18   | Pin 12             | PCM_CLK / PWM0 / SPI1_CE0     | B2      |
| JA11     | GND      | Pin 39 (GND)       | Ground                         | --      |
| JA12     | VCC      | 3.3V               | Power (3.3V)                   | --      |

#### Pmod Connector JB (SPI + I2C + GPIO)

JB top row (pins 1-6) carries SPI0 with chip select CE1 (shares MOSI/MISO/CLK with JA).
JB bottom row (pins 7-12) carries I2C1 on pins 9-10.

| Pmod Pin | BCM GPIO | RPi 40-pin Header | Alternate Function(s)          | Glasgow |
|----------|----------|--------------------|--------------------------------|---------|
| JB1      | GPIO7    | Pin 26             | **SPI0_CE1**                   | B5      |
| JB2      | GPIO10   | Pin 19             | **SPI0_MOSI** (shared with JA) | --      |
| JB3      | GPIO9    | Pin 21             | **SPI0_MISO** (shared with JA) | --      |
| JB4      | GPIO11   | Pin 23             | **SPI0_SCLK** (shared with JA) | --      |
| JB5      | GND      | Pin 25 (GND)       | Ground                         | --      |
| JB6      | VCC      | 3.3V               | Power (3.3V)                   | --      |
| JB7      | GPIO26   | Pin 37             | GPIO only                      | A3      |
| JB8      | GPIO13   | Pin 33             | PWM1                           | A5      |
| JB9      | GPIO3    | Pin 5              | **I2C1_SCL** (1.8kΩ pull-up)  | B6      |
| JB10     | GPIO2    | Pin 3              | **I2C1_SDA** (1.8kΩ pull-up)  | B7      |
| JB11     | GND      | Pin 39 (GND)       | Ground                         | --      |
| JB12     | VCC      | 3.3V               | Power (3.3V)                   | --      |

#### Pmod Connector JC (UART + GPIO)

JC top row (pins 1-6) carries UART0.
JC bottom row (pins 7-12) carries GPCLK/PWM signals.

| Pmod Pin | BCM GPIO | RPi 40-pin Header | Alternate Function(s)          | Glasgow |
|----------|----------|--------------------|--------------------------------|---------|
| JC1      | GPIO16   | Pin 36             | **UART0_CTS** / SPI1_CE2      | A0      |
| JC2      | GPIO14   | Pin 8              | **UART0_TXD**                  | B0      |
| JC3      | GPIO15   | Pin 10             | **UART0_RXD**                  | B1      |
| JC4      | GPIO17   | Pin 11             | **UART0_RTS** / SPI1_CE1      | --      |
| JC5      | GND      | Pin 25 (GND)       | Ground                         | --      |
| JC6      | VCC      | 3.3V               | Power (3.3V)                   | --      |
| JC7      | GPIO4    | Pin 7              | GPCLK0                         | --      |
| JC8      | GPIO12   | Pin 32             | PWM0                           | --      |
| JC9      | GPIO5    | Pin 29             | GPCLK1                         | A7      |
| JC10     | GPIO6    | Pin 31             | GPCLK2                         | A6      |
| JC11     | GND      | Pin 39 (GND)       | Ground                         | --      |
| JC12     | VCC      | 3.3V               | Power (3.3V)                   | --      |

#### Unused GPIO Pins

Five RPi GPIO pins are **not connected** to any Pmod connector: **GPIO22**,
**GPIO23**, **GPIO24**, **GPIO25**, **GPIO27**.

Of these, **GPIO25** is connected to Glasgow pin **B3** (RPi header pin 22).

The remaining four are connected in two loopback pairs via jumper wires on the
RPi5 GPIO header (discovered using `discover_gpio_pairs.py`):

| Pair | Pin A  | Header Pin | Pin B  | Header Pin |
|------|--------|------------|--------|------------|
| 1    | GPIO22 | Pin 15     | GPIO27 | Pin 13     |
| 2    | GPIO23 | Pin 16     | GPIO24 | Pin 18     |

These loopback pairs can be used for self-test purposes (driving one pin and
reading the other to verify GPIO functionality without external hardware).

#### Shared Bus Constraints

**SPI0 bus sharing:** JA (pins 1-4) and JB (pins 1-4) share the SPI0 MOSI, MISO, and
SCLK lines (GPIO10, GPIO9, GPIO11). They differ only in chip select: JA uses CE0
(GPIO8) and JB uses CE1 (GPIO7). This means:
- Two SPI devices can be used simultaneously (one on JA, one on JB) using different
  chip selects
- If JA or JB is used for non-SPI GPIO purposes, the shared pins conflict and the
  other port's top row becomes unusable

**I2C1 bus:** JB pins 9-10 carry I2C1 (GPIO2/SDA, GPIO3/SCL). These pins have
hardware 1.8kΩ pull-up resistors to 3.3V on the RPi board itself.

**UART0:** JC pins 1-4 carry the primary UART. On RPi4, this is the PL011 UART
(typically `/dev/ttyAMA0` when enabled). On RPi5, UART routing differs due to RP1.

---

## Pmod Interface Types

The Pmod standard defines several interface types that specify the signal assignments
on the 6-pin (1x6) or 12-pin (2x6) connectors. The specification is maintained by
Digilent:
https://digilent.com/reference/_media/reference/pmod/pmod-interface-specification-1_3_1.pdf

### Physical Connector Layout

```
  12-pin (2x6) Pmod connector        6-pin (1x6) Pmod connector
  (active-low nSS side = pin 1)

  ┌─────────────────────────┐         ┌─────────────────────────┐
  │ Pin6  Pin5  Pin4  Pin3  │         │ Pin6 Pin5 Pin4 Pin3     │
  │ (VCC) (GND)             │         │(VCC)(GND)               │
  │ Pin12 Pin11 Pin10 Pin9  │         │ Pin2 Pin1               │
  │ (VCC) (GND)             │         └─────────────────────────┘
  │ Pin2  Pin1              │
  └─────────────────────────┘

  Physical pin numbering (active-low nSS end = pin 1):

  Top row:     Pin 1 | Pin 2 | Pin 3 | Pin 4 | Pin 5 (GND) | Pin 6 (VCC)
  Bottom row:  Pin 7 | Pin 8 | Pin 9 | Pin10 | Pin11 (GND) | Pin12 (VCC)
```

### Type 1 -- GPIO

General-purpose bidirectional I/O. All signal pins are directly connected to
host GPIOs with no protocol requirement.

| Pin | Signal | Direction      |
|-----|--------|----------------|
| 1   | IO1    | Bidirectional  |
| 2   | IO2    | Bidirectional  |
| 3   | IO3    | Bidirectional  |
| 4   | IO4    | Bidirectional  |
| 5   | GND    | Power          |
| 6   | VCC    | Power          |

**Type 1A (expanded GPIO):** Uses the full 12-pin connector. Pins 7-10 are
additional GPIO (IO5-IO8), with pins 11 (GND) and 12 (VCC).

### Type 2 -- SPI

Serial Peripheral Interface. The host acts as SPI master.

| Pin | Signal | Direction      | Description                    |
|-----|--------|----------------|--------------------------------|
| 1   | CS     | Host -> Pmod   | Chip select (active low)       |
| 2   | MOSI   | Host -> Pmod   | Master Out, Slave In           |
| 3   | MISO   | Pmod -> Host   | Master In, Slave Out           |
| 4   | SCK    | Host -> Pmod   | Serial clock                   |
| 5   | GND    | Power          |                                |
| 6   | VCC    | Power          |                                |

**Type 2A (expanded SPI):** Uses the full 12-pin connector. Bottom row provides
additional signals:

| Pin | Signal | Direction      | Description                    |
|-----|--------|----------------|--------------------------------|
| 7   | INT    | Pmod -> Host   | Interrupt (active low), optional|
| 8   | RESET  | Host -> Pmod   | Reset (active low), optional   |
| 9   | CS2    | Host -> Pmod   | Second chip select, optional   |
| 10  | CS3    | Host -> Pmod   | Third chip select, optional    |
| 11  | GND    | Power          |                                |
| 12  | VCC    | Power          |                                |

Pins 7-10 may also be used as general-purpose GPIO when the optional SPI
signals are not needed.

### Type 3 -- UART

Universal Asynchronous Receiver/Transmitter.

| Pin | Signal | Direction      | Description                    |
|-----|--------|----------------|--------------------------------|
| 1   | CTS    | Pmod -> Host   | Clear To Send (flow control)   |
| 2   | TXD    | Host -> Pmod   | Transmit data                  |
| 3   | RXD    | Pmod -> Host   | Receive data                   |
| 4   | RTS    | Host -> Pmod   | Request To Send (flow control) |
| 5   | GND    | Power          |                                |
| 6   | VCC    | Power          |                                |

**Type 3A (expanded UART):** Uses the full 12-pin connector. Bottom row provides
additional signals (typically GPIO or a second UART channel).

Note: CTS/RTS flow control signals (pins 1 and 4) are optional. A minimal UART
Pmod only needs TXD (pin 2) and RXD (pin 3).

### Type 4 -- H-Bridge

Motor driver (single H-bridge). Note: Type 4 is deprecated.

| Pin | Signal | Direction      | Description                    |
|-----|--------|----------------|--------------------------------|
| 1   | DIR    | Host -> Pmod   | Direction control              |
| 2   | EN     | Host -> Pmod   | Enable (PWM input)             |
| 3   | SA     | Pmod -> Host   | Sensor A (encoder feedback)    |
| 4   | SB     | Pmod -> Host   | Sensor B (encoder feedback)    |
| 5   | GND    | Power          |                                |
| 6   | VCC    | Power          |                                |

### Type 5 -- Dual H-Bridge

Dual motor driver. Note: Type 5 is deprecated.

Uses the full 12-pin connector with two H-bridge channels.

### Type 6 -- I2C

Inter-Integrated Circuit bus.

| Pin | Signal    | Direction      | Description                    |
|-----|-----------|----------------|--------------------------------|
| 1   | INT/GPIO  | Pmod -> Host   | Interrupt (optional) or GPIO   |
| 2   | RESET/GPIO| Host -> Pmod   | Reset (optional) or GPIO       |
| 3   | SCL       | Host -> Pmod   | I2C clock                      |
| 4   | SDA       | Bidirectional  | I2C data                       |
| 5   | GND       | Power          |                                |
| 6   | VCC       | Power          |                                |

**Type 6A (expanded I2C):** Uses the full 12-pin connector. Bottom row provides
additional signals (typically GPIO, second I2C channel, or SPI for dual-interface
devices).

---

## Pmod HAT Connector-to-Type Compatibility

Based on the GPIO mappings and alternate functions, each Pmod connector on the HAT
supports specific Pmod interface types:

| Connector | Top Row (pins 1-4)        | Bottom Row (pins 7-10)     | Supported Pmod Types           |
|-----------|---------------------------|----------------------------|--------------------------------|
| **JA**    | SPI0 (CE0, MOSI, MISO, CLK) | PCM / PWM / SPI1        | Type 2 (SPI), Type 1 (GPIO)   |
| **JB**    | SPI0 (CE1, MOSI, MISO, CLK) | GPIO, PWM1, I2C1 (SCL, SDA) | Type 2 (SPI), Type 6 (I2C on bottom), Type 1 (GPIO) |
| **JC**    | UART0 (CTS, TXD, RXD, RTS)  | GPCLK, PWM0             | Type 3 (UART), Type 1 (GPIO)  |

### Type 2 (SPI) on JA

| Pmod Type 2 Pin | Signal | JA Pin | BCM GPIO | RPi Function   |
|-----------------|--------|--------|----------|----------------|
| Pin 1 (CS)      | CS     | JA1    | GPIO8    | SPI0_CE0       |
| Pin 2 (MOSI)    | MOSI   | JA2    | GPIO10   | SPI0_MOSI      |
| Pin 3 (MISO)    | MISO   | JA3    | GPIO9    | SPI0_MISO      |
| Pin 4 (SCK)     | SCK    | JA4    | GPIO11   | SPI0_SCLK      |

### Type 2 (SPI) on JB

| Pmod Type 2 Pin | Signal | JB Pin | BCM GPIO | RPi Function   |
|-----------------|--------|--------|----------|----------------|
| Pin 1 (CS)      | CS     | JB1    | GPIO7    | SPI0_CE1       |
| Pin 2 (MOSI)    | MOSI   | JB2    | GPIO10   | SPI0_MOSI      |
| Pin 3 (MISO)    | MISO   | JB3    | GPIO9    | SPI0_MISO      |
| Pin 4 (SCK)     | SCK    | JB4    | GPIO11   | SPI0_SCLK      |

### Type 3 (UART) on JC

| Pmod Type 3 Pin | Signal | JC Pin | BCM GPIO | RPi Function   |
|-----------------|--------|--------|----------|----------------|
| Pin 1 (CTS)     | CTS    | JC1    | GPIO16   | UART0_CTS      |
| Pin 2 (TXD)     | TXD    | JC2    | GPIO14   | UART0_TXD      |
| Pin 3 (RXD)     | RXD    | JC3    | GPIO15   | UART0_RXD      |
| Pin 4 (RTS)     | RTS    | JC4    | GPIO17   | UART0_RTS      |

### Type 6 (I2C) on JB bottom row

| Pmod Type 6 Pin | Signal    | JB Pin | BCM GPIO | RPi Function   |
|-----------------|-----------|--------|----------|----------------|
| Pin 1 (INT)     | INT/GPIO  | JB7    | GPIO26   | GPIO           |
| Pin 2 (RESET)   | RESET/GPIO| JB8    | GPIO13   | PWM1 / GPIO    |
| Pin 3 (SCL)     | SCL       | JB9    | GPIO3    | I2C1_SCL       |
| Pin 4 (SDA)     | SDA       | JB10   | GPIO2    | I2C1_SDA       |

---

## Inter-RPi Jumper Cable Connections

The two RPi Pmod devices (rpi5-pmod and rpi4-pmod) are connected via jumper cables
between their Pmod connectors. All three ports are connected straight-through:
JA-to-JA, JB-to-JB, and JC-to-JC.

Since both RPi devices use the same Pmod HAT Adapter, the **same BCM GPIO number**
on each side is connected together. For example, RPi5 GPIO8 (JA pin 1) is wired
to RPi4 GPIO8 (JA pin 1).

### JA connections (RPi5 <-> RPi4)

| Pmod Pin | Signal      | RPi5 GPIO | RPi4 GPIO | Glasgow | Notes                    |
|----------|-------------|-----------|-----------|---------|--------------------------|
| JA1      | SPI0_CE0    | GPIO8     | GPIO8     | B4      | SPI chip select 0        |
| JA2      | SPI0_MOSI   | GPIO10    | GPIO10    | --      | Shared with JB2          |
| JA3      | SPI0_MISO   | GPIO9     | GPIO9     | --      | Shared with JB3          |
| JA4      | SPI0_SCLK   | GPIO11    | GPIO11    | --      | Shared with JB4          |
| JA5      | GND         | --        | --        | --      | Ground                   |
| JA6      | VCC         | --        | --        | --      | 3.3V (DO NOT jumper VCC) |
| JA7      | PCM_FS/PWM1 | GPIO19    | GPIO19    | A4      |                          |
| JA8      | PCM_DOUT    | GPIO21    | GPIO21    | A2      |                          |
| JA9      | PCM_DIN     | GPIO20    | GPIO20    | A1      |                          |
| JA10     | PCM_CLK/PWM0| GPIO18    | GPIO18    | B2      |                          |
| JA11     | GND         | --        | --        | --      | Ground                   |
| JA12     | VCC         | --        | --        | --      | 3.3V (DO NOT jumper VCC) |

### JB connections (RPi5 <-> RPi4)

| Pmod Pin | Signal      | RPi5 GPIO | RPi4 GPIO | Glasgow | Notes                    |
|----------|-------------|-----------|-----------|---------|--------------------------|
| JB1      | SPI0_CE1    | GPIO7     | GPIO7     | B5      | SPI chip select 1        |
| JB2      | SPI0_MOSI   | GPIO10    | GPIO10    | --      | Shared with JA2          |
| JB3      | SPI0_MISO   | GPIO9     | GPIO9     | --      | Shared with JA3          |
| JB4      | SPI0_SCLK   | GPIO11    | GPIO11    | --      | Shared with JA4          |
| JB5      | GND         | --        | --        | --      | Ground                   |
| JB6      | VCC         | --        | --        | --      | 3.3V (DO NOT jumper VCC) |
| JB7      | GPIO        | GPIO26    | GPIO26    | A3      |                          |
| JB8      | PWM1        | GPIO13    | GPIO13    | A5      |                          |
| JB9      | I2C1_SCL    | GPIO3     | GPIO3     | B6      | Has 1.8kΩ pull-up        |
| JB10     | I2C1_SDA    | GPIO2     | GPIO2     | B7      | Has 1.8kΩ pull-up        |
| JB11     | GND         | --        | --        | --      | Ground                   |
| JB12     | VCC         | --        | --        | --      | 3.3V (DO NOT jumper VCC) |

### JC connections (RPi5 <-> RPi4)

| Pmod Pin | Signal      | RPi5 GPIO | RPi4 GPIO | Glasgow | Notes                    |
|----------|-------------|-----------|-----------|---------|--------------------------|
| JC1      | UART0_CTS   | GPIO16    | GPIO16    | A0      |                          |
| JC2      | UART0_TXD   | GPIO14    | GPIO14    | B0      |                          |
| JC3      | UART0_RXD   | GPIO15    | GPIO15    | B1      |                          |
| JC4      | UART0_RTS   | GPIO17    | GPIO17    | --      |                          |
| JC5      | GND         | --        | --        | --      | Ground                   |
| JC6      | VCC         | --        | --        | --      | 3.3V (DO NOT jumper VCC) |
| JC7      | GPCLK0      | GPIO4     | GPIO4     | --      |                          |
| JC8      | PWM0        | GPIO12    | GPIO12    | --      |                          |
| JC9      | GPCLK1      | GPIO5     | GPIO5     | A7      |                          |
| JC10     | GPCLK2      | GPIO6     | GPIO6     | A6      |                          |
| JC11     | GND         | --        | --        | --      | Ground                   |
| JC12     | VCC         | --        | --        | --      | 3.3V (DO NOT jumper VCC) |

### Unique GPIO connections (excluding shared SPI bus pins)

After de-duplicating the shared SPI0 lines (GPIO9, GPIO10, GPIO11), the complete
set of unique GPIO-to-GPIO connections between the two RPi devices is:

| BCM GPIO | Pmod Pin(s)   | Function                | Glasgow |
|----------|---------------|-------------------------|---------|
| GPIO2    | JB10          | I2C1_SDA                | B7      |
| GPIO3    | JB9           | I2C1_SCL                | B6      |
| GPIO4    | JC7           | GPCLK0                  | --      |
| GPIO5    | JC9           | GPCLK1                  | A7      |
| GPIO6    | JC10          | GPCLK2                  | A6      |
| GPIO7    | JB1           | SPI0_CE1                | B5      |
| GPIO8    | JA1           | SPI0_CE0                | B4      |
| GPIO9    | JA3, JB3      | SPI0_MISO               | --      |
| GPIO10   | JA2, JB2      | SPI0_MOSI               | --      |
| GPIO11   | JA4, JB4      | SPI0_SCLK               | --      |
| GPIO12   | JC8           | PWM0                    | --      |
| GPIO13   | JB8           | PWM1                    | A5      |
| GPIO14   | JC2           | UART0_TXD               | B0      |
| GPIO15   | JC3           | UART0_RXD               | B1      |
| GPIO16   | JC1           | UART0_CTS / SPI1_CE2    | A0      |
| GPIO17   | JC4           | UART0_RTS / SPI1_CE1    | --      |
| GPIO18   | JA10          | PCM_CLK / PWM0 / SPI1_CE0 | B2   |
| GPIO19   | JA7           | PCM_FS / PWM1 / SPI1_MISO | A4   |
| GPIO20   | JA9           | PCM_DIN / SPI1_MOSI     | A1      |
| GPIO21   | JA8           | PCM_DOUT / SPI1_SCLK    | A2      |
| GPIO26   | JB7           | GPIO                    | A3      |

That is **21 unique GPIO connections** between the two RPi devices (or 24 Pmod
signal pins total, with 3 duplicated via the shared SPI0 bus).

Of these 21 inter-RPi GPIO connections, **15 are also tapped by the Glasgow**.
The 6 GPIOs that are connected between RPis but **not** tapped by Glasgow are:
GPIO4, GPIO9, GPIO10, GPIO11, GPIO12, GPIO17.

The Glasgow also taps **GPIO25** which is **not** part of the inter-RPi connection
(it is not routed through any Pmod connector).

---

## Using Glasgow as a Signal Analyser

The Glasgow can passively observe signals between the rpi5-pmod and rpi4-pmod
using its `analyzer` applet. Since 15 of the 16 Glasgow pins are tapped into
the inter-RPi signal lines, the Glasgow can capture traffic on most of the
buses simultaneously.

### Capturable Buses

#### UART (JC top row) -- Glasgow A0, B0, B1

| Signal    | GPIO   | Glasgow | Direction (RPi5 perspective) |
|-----------|--------|---------|------------------------------|
| UART0_CTS | GPIO16 | A0      | Input (from RPi4)            |
| UART0_TXD | GPIO14 | B0      | Output (to RPi4)             |
| UART0_RXD | GPIO15 | B1      | Input (from RPi4)            |

UART0_RTS (GPIO17) is **not** tapped -- but RTS/CTS are optional flow control.

#### I2C (JB bottom row) -- Glasgow B6, B7

| Signal    | GPIO   | Glasgow | Note                         |
|-----------|--------|---------|------------------------------|
| I2C1_SCL  | GPIO3  | B6      | 1.8kΩ pull-up to 3.3V       |
| I2C1_SDA  | GPIO2  | B7      | 1.8kΩ pull-up to 3.3V       |

#### SPI0 chip selects -- Glasgow B4, B5

| Signal    | GPIO   | Glasgow | Note                         |
|-----------|--------|---------|------------------------------|
| SPI0_CE0  | GPIO8  | B4      | JA chip select               |
| SPI0_CE1  | GPIO7  | B5      | JB chip select               |

SPI0 MOSI/MISO/SCLK (GPIO9, GPIO10, GPIO11) are **not** tapped by Glasgow.

#### PCM / SPI1 (JA bottom row) -- Glasgow A1, A2, A4, B2

| Signal         | GPIO   | Glasgow |
|----------------|--------|---------|
| PCM_CLK/SPI1_CE0  | GPIO18 | B2  |
| PCM_FS/SPI1_MISO  | GPIO19 | A4  |
| PCM_DIN/SPI1_MOSI | GPIO20 | A1  |
| PCM_DOUT/SPI1_SCLK| GPIO21 | A2  |

All 4 PCM/SPI1 signals are tapped -- this bus is fully observable.

#### GPCLK / PWM / GPIO -- Glasgow A3, A5, A6, A7

| Signal   | GPIO   | Glasgow |
|----------|--------|---------|
| GPCLK1   | GPIO5  | A7      |
| GPCLK2   | GPIO6  | A6      |
| PWM1     | GPIO13 | A5      |
| GPIO     | GPIO26 | A3      |

### Glasgow Analyzer Usage

```bash
# Capture all 16 Glasgow pins as a logic trace (VCD format)
glasgow run analyzer -V 3.3 \
    --pins 'A0,A1,A2,A3,A4,A5,A6,A7,B0,B1,B2,B3,B4,B5,B6,B7' \
    --pin-set-data 'A0,A1,A2,A3,A4,A5,A6,A7,B0,B1,B2,B3,B4,B5,B6,B7' \
    -f <sample_rate> \
    -o trace.vcd

# View in GTKWave or PulseView
gtkwave trace.vcd
pulseview -I vcd -i trace.vcd
```

### Unmonitored Signals

The following inter-RPi signals are **not** observable by the Glasgow:

| Signal    | GPIO   | Pmod Pin | Reason              |
|-----------|--------|----------|----------------------|
| GPCLK0    | GPIO4  | JC7      | No Glasgow wire      |
| SPI0_MISO | GPIO9  | JA3/JB3  | No Glasgow wire      |
| SPI0_MOSI | GPIO10 | JA2/JB2  | No Glasgow wire      |
| SPI0_SCLK | GPIO11 | JA4/JB4  | No Glasgow wire      |
| PWM0      | GPIO12 | JC8      | No Glasgow wire      |
| UART0_RTS | GPIO17 | JC4      | No Glasgow wire      |

---

# Power Cycling

## NeTV2 Devices (Tasmota Smart Plugs)

The rpi5-netv2 and rpi3-netv2 are powered via Tasmota-based smart plugs
(Athom Plug V3, ESP32-C3). Each plug is controllable via HTTP.

| Device     | Plug       | Plug Hostname                            |
|------------|------------|------------------------------------------|
| rpi5-netv2 | au-plug-22 | au-plug-22.iot.welland.mithis.com        |
| rpi3-netv2 | au-plug-18 | au-plug-18.iot.welland.mithis.com        |

### Tasmota HTTP API

```bash
# Check power status
curl "http://au-plug-22.iot.welland.mithis.com/cm?cmnd=Power"
# Returns: {"POWER":"ON"} or {"POWER":"OFF"}

# Power off
curl "http://au-plug-22.iot.welland.mithis.com/cm?cmnd=Power%20Off"

# Power on
curl "http://au-plug-22.iot.welland.mithis.com/cm?cmnd=Power%20On"

# Toggle power
curl "http://au-plug-22.iot.welland.mithis.com/cm?cmnd=Power%20Toggle"
```

### Power Cycle Script

A power cycle script is available at
`~/github/mithro/fpgas-online-test-designs/tmp/power_cycle.py`:

```bash
# Power cycle rpi5-netv2 (off for 5 seconds, then on)
python3 ~/github/mithro/fpgas-online-test-designs/tmp/power_cycle.py au-plug-22

# Power cycle rpi3-netv2
python3 ~/github/mithro/fpgas-online-test-designs/tmp/power_cycle.py au-plug-18
```

After power cycling, wait approximately 30 seconds for the device to boot.

## Pmod Devices (PoE)

The rpi5-pmod and rpi4-pmod are powered via Power over Ethernet (PoE) from a
Netgear GSM7252PS switch.

| Device    | Switch                     | Port  | Management IP |
|-----------|----------------------------|-------|---------------|
| rpi5-pmod | sw-netgear-gsm7252ps-s1    | 1/0/1 | 10.1.5.23     |
| rpi4-pmod | sw-netgear-gsm7252ps-s1    | 1/0/2 | 10.1.5.23     |

The switch is a Netgear GSM7252PS 48-Port GE L2+ Managed Stackable PoE Switch
with 2 10GE SFP+ ports. Management IP: `10.1.5.23`.

### SNMP PoE Control

PoE is controlled via SNMP using the POWER-ETHERNET-MIB (RFC 3621). The
`pethPsePortAdminEnable` object is at column 3 of the `pethPsePortTable`
(columns 1-2 are not-accessible index columns).

```
OID: 1.3.6.1.2.1.105.1.1.1.3.<groupIndex>.<portIndex>
     ^^^^^^^^^^^^^^^^^^^^^^^^^^^
     pethPsePortAdminEnable

Values: 1 = true (PoE enabled), 2 = false (PoE disabled)

Community strings:
  Read:  public
  Write: private
```

```bash
# Install SNMP tools (already installed on rpi5-pmod)
sudo apt install snmp

# Query PoE status for port 1/0/1 (rpi5-pmod)
snmpget -v2c -c public 10.1.5.23 1.3.6.1.2.1.105.1.1.1.3.1.1

# Query PoE status for port 1/0/2 (rpi4-pmod)
snmpget -v2c -c public 10.1.5.23 1.3.6.1.2.1.105.1.1.1.3.1.2

# Disable PoE on port 1/0/1 (power off rpi5-pmod)
snmpset -v2c -c private 10.1.5.23 1.3.6.1.2.1.105.1.1.1.3.1.1 i 2

# Enable PoE on port 1/0/1 (power on rpi5-pmod)
snmpset -v2c -c private 10.1.5.23 1.3.6.1.2.1.105.1.1.1.3.1.1 i 1

# Disable PoE on port 1/0/2 (power off rpi4-pmod)
snmpset -v2c -c private 10.1.5.23 1.3.6.1.2.1.105.1.1.1.3.1.2 i 2

# Enable PoE on port 1/0/2 (power on rpi4-pmod)
snmpset -v2c -c private 10.1.5.23 1.3.6.1.2.1.105.1.1.1.3.1.2 i 1
```

After disabling PoE, the device powers off immediately. After re-enabling,
allow ~30 seconds for the device to boot.

---

# Verification

Run `verify_pmod_connections.py` from the local machine to verify all jumper cable
connections between the two RPi Pmod devices. The script tests each GPIO connection
in both directions (RPi5 -> RPi4 and RPi4 -> RPi5) by driving a pin high/low on one
side and reading it on the other.

```
uv run verify_pmod_connections.py
```

See `verify_pmod_connections.py` for details.

Run `discover_glasgow_pins.py` on the rpi5-pmod (as root) to re-discover the
Glasgow-to-RPi GPIO pin mapping:

```bash
# On rpi5-pmod:
sudo python3 ~/discover_glasgow_pins.py
```

Run `discover_gpio_pairs.py` on the rpi5-pmod (as root) to re-discover the
loopback pairs among unused GPIO pins:

```bash
# On rpi5-pmod:
sudo python3 ~/discover_gpio_pairs.py
```
