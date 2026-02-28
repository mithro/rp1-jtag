#!/usr/bin/env python3
"""
Integrate rp1_pio_jtag adapter driver into an OpenOCD source tree.

Usage:
    python3 integrate.py /path/to/openocd

This script:
  1. Copies rp1_pio_jtag.c into src/jtag/drivers/
  2. Patches configure.ac to add m4 adapter definition, PKG_CHECK, AC_ARG_ENABLE,
     PROCESS_ADAPTERS, USE_LIBRP1JTAG conditional, and summary
  3. Patches src/jtag/drivers/Makefile.am to build the driver
  4. Patches src/jtag/interface.h to add extern declaration
  5. Patches src/jtag/interfaces.c to register the driver

The script is idempotent — safe to run multiple times.

SPDX-License-Identifier: Apache-2.0
"""

import sys
import shutil
from pathlib import Path

MARKER = "rp1_pio_jtag"


def insert_before(content: str, anchor: str, block: str, marker: str) -> str:
    """Insert block BEFORE the first occurrence of anchor."""
    # Skip if block's key content is already present
    key_lines = [l.strip() for l in block.strip().split('\n') if l.strip()]
    if key_lines and any(l in content for l in key_lines[1:2]):
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
    key_lines = [l.strip() for l in block.strip().split('\n') if l.strip()]
    if key_lines and key_lines[0] in content:
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

    # ---- 2. Patch configure.ac ----
    print("\nPatching configure.ac...")
    configure_ac = ocd_dir / "configure.ac"
    ca = configure_ac.read_text()

    # 2a. Add m4_define for LIBRP1JTAG_ADAPTERS after LIBGPIOD_ADAPTERS
    ca = insert_after(
        ca,
        "m4_define([LIBGPIOD_ADAPTERS],\n"
        "\t[[[linuxgpiod], [Linux GPIO bitbang through libgpiod], [LINUXGPIOD]]])\n",
        "\nm4_define([LIBRP1JTAG_ADAPTERS],\n"
        "\t[[[rp1_pio_jtag], [RP1 PIO JTAG for Raspberry Pi 5], [RP1_PIO_JTAG]]])\n",
        "configure.ac m4_define",
    )

    # 2b. Add to AC_ARG_ADAPTERS (first group, default=auto)
    # Insert after LIBGPIOD_ADAPTERS in the AC_ARG_ADAPTERS call
    ca = insert_after(
        ca,
        "  LIBGPIOD_ADAPTERS,\n",
        "  LIBRP1JTAG_ADAPTERS,\n",
        "configure.ac AC_ARG_ADAPTERS",
    )

    # 2c. Add PKG_CHECK_MODULES for librp1jtag after libjaylink check
    ca = insert_after(
        ca,
        "PKG_CHECK_MODULES([LIBJAYLINK], [libjaylink >= 0.2],\n"
        "\t[use_libjaylink=yes], [use_libjaylink=no])\n",
        "\nPKG_CHECK_MODULES([LIBRP1JTAG], [rp1jtag],\n"
        "\t[use_librp1jtag=yes], [use_librp1jtag=no])\n",
        "configure.ac PKG_CHECK_MODULES",
    )

    # 2d. Add PROCESS_ADAPTERS after LIBGPIOD_ADAPTERS processing
    ca = insert_after(
        ca,
        "PROCESS_ADAPTERS([LIBGPIOD_ADAPTERS], "
        "[\"x$use_libgpiod\" = \"xyes\"], [Linux libgpiod])\n",
        "PROCESS_ADAPTERS([LIBRP1JTAG_ADAPTERS], "
        "[\"x$use_librp1jtag\" = \"xyes\"], [librp1jtag])\n",
        "configure.ac PROCESS_ADAPTERS",
    )

    # 2e. Add AM_CONDITIONAL for USE_LIBRP1JTAG after USE_LIBGPIOD
    ca = insert_after(
        ca,
        "AM_CONDITIONAL([USE_LIBGPIOD], [test \"x$use_libgpiod\" = \"xyes\"])\n",
        "AM_CONDITIONAL([USE_LIBRP1JTAG], [test \"x$use_librp1jtag\" = \"xyes\"])\n",
        "configure.ac AM_CONDITIONAL",
    )

    # 2f. Add to configuration summary after LIBGPIOD_ADAPTERS
    ca = insert_after(
        ca,
        "\tLIBGPIOD_ADAPTERS,\n",
        "\tLIBRP1JTAG_ADAPTERS,\n",
        "configure.ac summary",
    )

    configure_ac.write_text(ca)

    # ---- 3. Patch src/jtag/drivers/Makefile.am ----
    print("\nPatching src/jtag/drivers/Makefile.am...")
    makefile_am = drivers_dir / "Makefile.am"
    mf = makefile_am.read_text()

    # Add USE_LIBRP1JTAG CPPFLAGS/LIBADD after USE_LIBGPIOD block
    ca_block = (
        "if USE_LIBGPIOD\n"
        "%C%_libocdjtagdrivers_la_CPPFLAGS += $(LIBGPIOD_CFLAGS)\n"
        "%C%_libocdjtagdrivers_la_LIBADD += $(LIBGPIOD_LIBS)\n"
        "endif\n"
    )
    mf = insert_after(
        mf,
        ca_block,
        "\nif USE_LIBRP1JTAG\n"
        "%C%_libocdjtagdrivers_la_CPPFLAGS += $(LIBRP1JTAG_CFLAGS)\n"
        "%C%_libocdjtagdrivers_la_LIBADD += $(LIBRP1JTAG_LIBS)\n"
        "endif\n",
        "Makefile.am CPPFLAGS/LIBADD",
    )

    # Add conditional source file after LINUXGPIOD DRIVERFILES
    mf = insert_after(
        mf,
        "if LINUXGPIOD\n"
        "DRIVERFILES += %D%/linuxgpiod.c\n"
        "endif\n",
        "if RP1_PIO_JTAG\n"
        "DRIVERFILES += %D%/rp1_pio_jtag.c\n"
        "endif\n",
        "Makefile.am DRIVERFILES",
    )

    makefile_am.write_text(mf)

    # ---- 4. Patch src/jtag/interface.h ----
    print("\nPatching src/jtag/interface.h...")
    interface_h = ocd_dir / "src" / "jtag" / "interface.h"
    ih = interface_h.read_text()

    # Add extern declaration in alphabetical order (after remote_bitbang)
    ih = insert_after(
        ih,
        "extern struct adapter_driver remote_bitbang_adapter_driver;\n",
        "extern struct adapter_driver rp1_pio_jtag_adapter_driver;\n",
        "interface.h extern",
    )

    interface_h.write_text(ih)

    # ---- 5. Patch src/jtag/interfaces.c ----
    print("\nPatching src/jtag/interfaces.c...")
    interfaces_c = ocd_dir / "src" / "jtag" / "interfaces.c"
    ic = interfaces_c.read_text()

    # Add driver entry after remote_bitbang (alphabetical order)
    ic = insert_after(
        ic,
        "#if BUILD_REMOTE_BITBANG == 1\n"
        "\t\t&remote_bitbang_adapter_driver,\n"
        "#endif\n",
        "#if BUILD_RP1_PIO_JTAG == 1\n"
        "\t\t&rp1_pio_jtag_adapter_driver,\n"
        "#endif\n",
        "interfaces.c array entry",
    )

    interfaces_c.write_text(ic)

    print("\n" + "=" * 60)
    print("Integration complete!")
    print("=" * 60)
    print()
    print("To build OpenOCD with rp1_pio_jtag support:")
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
