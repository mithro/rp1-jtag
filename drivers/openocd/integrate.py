#!/usr/bin/env python3
"""
Integrate rp1_pio_jtag adapter driver into an OpenOCD source tree.

Usage:
    python3 integrate.py /path/to/openocd

This script:
  1. Copies rp1_pio_jtag.c into src/jtag/drivers/
  2. Patches src/jtag/interfaces.c to register the driver
  3. Patches src/jtag/interfaces.h (or interface.h) to add the extern declaration
  4. Patches configure.ac to add --enable-rp1-pio-jtag option
  5. Patches src/jtag/drivers/Makefile.am to build the driver

SPDX-License-Identifier: Apache-2.0
"""

import sys
import shutil
from pathlib import Path

ALREADY_APPLIED = "rp1_pio_jtag"


def insert_before(content: str, anchor: str, block: str, marker: str) -> str:
    """Insert block BEFORE the first occurrence of anchor."""
    if ALREADY_APPLIED in content and ALREADY_APPLIED in block:
        # Check if this specific patch is already applied
        lines = block.strip().split('\n')
        key_line = lines[1] if len(lines) > 1 else lines[0]
        if key_line.strip() in content:
            print(f"  {marker}: already applied, skipping")
            return content
    idx = content.find(anchor)
    if idx < 0:
        print(f"  ERROR: {marker}: anchor not found")
        print(f"         Looking for: {anchor[:80]!r}...")
        sys.exit(1)
    content = content[:idx] + block + content[idx:]
    print(f"  {marker}: applied")
    return content


def insert_after(content: str, anchor: str, block: str, marker: str) -> str:
    """Insert block AFTER the first occurrence of anchor."""
    if block.strip().split('\n')[0].strip() in content:
        print(f"  {marker}: already applied, skipping")
        return content
    idx = content.find(anchor)
    if idx < 0:
        print(f"  ERROR: {marker}: anchor not found")
        print(f"         Looking for: {anchor[:80]!r}...")
        sys.exit(1)
    insert_pos = idx + len(anchor)
    content = content[:insert_pos] + block + content[insert_pos:]
    print(f"  {marker}: applied")
    return content


def replace_once(content: str, old: str, new: str, marker: str) -> str:
    """Replace first occurrence of old with new."""
    if new in content:
        print(f"  {marker}: already applied, skipping")
        return content
    if old not in content:
        print(f"  ERROR: {marker}: text not found")
        print(f"         Looking for: {old[:80]!r}...")
        sys.exit(1)
    content = content.replace(old, new, 1)
    print(f"  {marker}: applied")
    return content


def main() -> None:
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} /path/to/openocd")
        sys.exit(1)

    ocd_dir = Path(sys.argv[1])
    drivers_dir = ocd_dir / "src" / "jtag" / "drivers"
    script_dir = Path(__file__).parent

    if not drivers_dir.exists():
        print(f"Error: {drivers_dir} not found. Is this an OpenOCD source tree?")
        sys.exit(1)

    print("Integrating rp1_pio_jtag adapter driver into OpenOCD...")

    # 1. Copy driver file
    src_file = "rp1_pio_jtag.c"
    shutil.copy2(script_dir / src_file, drivers_dir / src_file)
    print(f"  Copied {src_file} -> src/jtag/drivers/{src_file}")

    # 2. Patch interfaces.c — add to driver list
    print("\nPatching src/jtag/interfaces.c...")
    interfaces_c = ocd_dir / "src" / "jtag" / "interfaces.c"
    ic = interfaces_c.read_text()

    # Add extern declaration near other externs
    # Look for the linuxgpiod extern as anchor
    ic = insert_before(
        ic,
        '#if BUILD_LINUXGPIOD == 1\n',
        '#if BUILD_RP1_PIO_JTAG == 1\n'
        'extern struct adapter_driver rp1_pio_jtag_adapter_driver;\n'
        '#endif\n',
        "interfaces.c extern",
    )

    # Add to adapter_drivers array
    ic = insert_before(
        ic,
        '#if BUILD_LINUXGPIOD == 1\n'
        '\t&linuxgpiod_adapter_driver,\n'
        '#endif\n',
        '#if BUILD_RP1_PIO_JTAG == 1\n'
        '\t&rp1_pio_jtag_adapter_driver,\n'
        '#endif\n',
        "interfaces.c array entry",
    )

    interfaces_c.write_text(ic)

    # 3. Patch configure.ac — add --enable-rp1-pio-jtag option
    print("\nPatching configure.ac...")
    configure_ac = ocd_dir / "configure.ac"
    ca = configure_ac.read_text()

    # Add AC_ARG_ENABLE and build conditional near linuxgpiod
    ca = insert_before(
        ca,
        'AC_ARG_ENABLE([linuxgpiod],',
        'AC_ARG_ENABLE([rp1_pio_jtag],\n'
        '  AS_HELP_STRING([--enable-rp1-pio-jtag],\n'
        '    [Enable RP1 PIO JTAG adapter driver for Raspberry Pi 5 (default: no)]),\n'
        '  [build_rp1_pio_jtag=$enableval], [build_rp1_pio_jtag=no])\n'
        '\n',
        "configure.ac AC_ARG_ENABLE",
    )

    # Add PKG_CHECK and AM_CONDITIONAL near linuxgpiod's
    # Find the linuxgpiod build conditional block
    linuxgpiod_conditional = 'AM_CONDITIONAL([LINUXGPIOD]'
    if linuxgpiod_conditional in ca:
        ca = insert_before(
            ca,
            linuxgpiod_conditional,
            'AS_IF([test "x$build_rp1_pio_jtag" = "xyes"], [\n'
            '  PKG_CHECK_MODULES([LIBRP1JTAG], [rp1jtag], [\n'
            '    AC_DEFINE([BUILD_RP1_PIO_JTAG], [1], [Build RP1 PIO JTAG driver])\n'
            '  ], [\n'
            '    PKG_CHECK_MODULES([LIBRP1JTAG], [librp1jtag], [\n'
            '      AC_DEFINE([BUILD_RP1_PIO_JTAG], [1], [Build RP1 PIO JTAG driver])\n'
            '    ], [\n'
            '      AC_MSG_ERROR([librp1jtag not found. Install from https://github.com/mithro/rp1-jtag])\n'
            '    ])\n'
            '  ])\n'
            '], [\n'
            '  build_rp1_pio_jtag=no\n'
            '])\n'
            'AM_CONDITIONAL([RP1_PIO_JTAG], [test "x$build_rp1_pio_jtag" = "xyes"])\n'
            '\n',
            "configure.ac AM_CONDITIONAL",
        )
    else:
        print("  WARNING: Could not find AM_CONDITIONAL anchor for linuxgpiod")
        print("  You may need to manually add the AM_CONDITIONAL for RP1_PIO_JTAG")

    configure_ac.write_text(ca)

    # 4. Patch src/jtag/drivers/Makefile.am — add source file
    print("\nPatching src/jtag/drivers/Makefile.am...")
    makefile_am = drivers_dir / "Makefile.am"
    mf = makefile_am.read_text()

    # Add conditional source near linuxgpiod
    mf = insert_before(
        mf,
        'if LINUXGPIOD\n',
        'if RP1_PIO_JTAG\n'
        'DRIVERFILES += %D%/rp1_pio_jtag.c\n'
        'endif\n\n',
        "Makefile.am DRIVERFILES",
    )

    # Add include/link flags if needed
    if 'RP1_PIO_JTAG' not in mf.split('DRIVERFILES')[0]:
        # Check if there's an AM_CFLAGS section we need to extend
        pass

    makefile_am.write_text(mf)

    print("\nDone! To build OpenOCD with rp1_pio_jtag support:")
    print(f"  cd {ocd_dir}")
    print("  ./bootstrap")
    print("  ./configure --enable-rp1-pio-jtag")
    print("  make -j$(nproc)")
    print()
    print("Example OpenOCD config (netv2.cfg):")
    print("  adapter driver rp1_pio_jtag")
    print("  rp1_pio_jtag jtag_nums 4 17 27 22")
    print("  adapter speed 1000")
    print("  transport select jtag")


if __name__ == "__main__":
    main()
