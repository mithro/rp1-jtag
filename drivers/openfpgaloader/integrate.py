#!/usr/bin/env python3
"""
Integrate rp1pio cable driver into an openFPGALoader source tree.

Usage:
    python3 integrate.py /path/to/openFPGALoader

This script:
  1. Copies rp1PioJtag.{hpp,cpp} into src/
  2. Patches cable.hpp to add MODE_RP1_PIO and cable_list entry
  3. Patches jtag.cpp to add the factory case
  4. Patches CMakeLists.txt to add ENABLE_RP1_PIO option
"""

import sys
import shutil
from pathlib import Path

ALREADY_APPLIED = "ENABLE_RP1_PIO"


def insert_before(content: str, anchor: str, block: str, marker: str) -> str:
    """Insert block BEFORE the first occurrence of anchor."""
    if ALREADY_APPLIED in block and ALREADY_APPLIED in content:
        # Check if this specific patch is already applied
        if block.strip().split('\n')[1] in content:
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
    if block.strip().split('\n')[0] in content:
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
        print(f"Usage: {sys.argv[0]} /path/to/openFPGALoader")
        sys.exit(1)

    ofl_dir = Path(sys.argv[1])
    src_dir = ofl_dir / "src"
    script_dir = Path(__file__).parent

    if not (src_dir / "jtag.cpp").exists():
        print(f"Error: {src_dir}/jtag.cpp not found. Is this an openFPGALoader tree?")
        sys.exit(1)

    print("Integrating rp1pio cable driver into openFPGALoader...")

    # 1. Copy driver files
    for f in ["rp1PioJtag.hpp", "rp1PioJtag.cpp"]:
        shutil.copy2(script_dir / f, src_dir / f)
        print(f"  Copied {f} -> src/{f}")

    # 2. Patch cable.hpp
    print("\nPatching cable.hpp...")
    cable = (src_dir / "cable.hpp").read_text()

    # Add enum entry after MODE_GWU2X
    cable = insert_after(
        cable,
        "\tMODE_GWU2X,            /*! Gowin GWU2X JTAG mode */\n",
        "\tMODE_RP1_PIO,          /*! RP1 PIO JTAG mode (RPi 5) */\n",
        "enum MODE_RP1_PIO",
    )

    # Add cable_list entry before ENABLE_LIBGPIOD block
    cable = insert_before(
        cable,
        '#ifdef ENABLE_LIBGPIOD\n\t{"libgpiod"',
        '#ifdef ENABLE_RP1_PIO\n'
        '\t{"rp1pio",             CABLE_DEF(MODE_RP1_PIO, 0, 0x0000                  )},\n'
        '#endif\n',
        "cable_list rp1pio",
    )

    (src_dir / "cable.hpp").write_text(cable)

    # 3. Patch jtag.cpp
    print("\nPatching jtag.cpp...")
    jtag = (src_dir / "jtag.cpp").read_text()

    # Add include after LIBGPIOD include
    jtag = insert_after(
        jtag,
        '#ifdef ENABLE_LIBGPIOD\n#include "libgpiodJtagBitbang.hpp"\n#endif\n',
        '#ifdef ENABLE_RP1_PIO\n#include "rp1PioJtag.hpp"\n#endif\n',
        "include rp1PioJtag",
    )

    # Add factory case BEFORE the LIBGPIOD block (so it's at the same level)
    jtag = insert_before(
        jtag,
        '#ifdef ENABLE_LIBGPIOD\n\tcase MODE_LIBGPIOD_BITBANG:\n',
        '#ifdef ENABLE_RP1_PIO\n'
        '\tcase MODE_RP1_PIO:\n'
        '\t\t_jtag = new Rp1PioJtag(pin_conf, dev, clkHZ, verbose);\n'
        '\t\tbreak;\n'
        '#endif\n',
        "factory MODE_RP1_PIO",
    )

    (src_dir / "jtag.cpp").write_text(jtag)

    # 4. Patch CMakeLists.txt
    print("\nPatching CMakeLists.txt...")
    cmake = (ofl_dir / "CMakeLists.txt").read_text()

    # Add option before LIBGPIOD option
    cmake = insert_before(
        cmake,
        '# Libgpiod is only available on Linux OS.\n',
        '# RP1 PIO is only available on Linux (RPi 5).\n'
        'if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")\n'
        '\toption(ENABLE_RP1_PIO "enable RP1 PIO JTAG driver (requires librp1jtag)" OFF)\n'
        'else()\n'
        '\tset(ENABLE_RP1_PIO OFF)\n'
        'endif()\n\n',
        "CMake option",
    )

    # Add source files before libGPIOD source block
    cmake = insert_before(
        cmake,
        '# libGPIOD support\nif (ENABLE_LIBGPIOD)\nlist(APPEND OPENFPGALOADER_SOURCE  src/libgpiodJtagBitbang',
        '# RP1 PIO JTAG support\n'
        'if (ENABLE_RP1_PIO)\n'
        'list(APPEND OPENFPGALOADER_SOURCE  src/rp1PioJtag.cpp)\n'
        'list(APPEND OPENFPGALOADER_HEADERS src/rp1PioJtag.hpp)\n'
        'endif()\n\n',
        "CMake sources",
    )

    # Add USE_DEVICE_ARG support (so --pins/--device CLI args work)
    cmake = replace_once(
        cmake,
        'if (ENABLE_UDEV OR ENABLE_LIBGPIOD OR ENABLE_JETSONNANOGPIO)\n'
        '\tadd_definitions(-DUSE_DEVICE_ARG)',
        'if (ENABLE_UDEV OR ENABLE_LIBGPIOD OR ENABLE_JETSONNANOGPIO OR ENABLE_RP1_PIO)\n'
        '\tadd_definitions(-DUSE_DEVICE_ARG)',
        "CMake USE_DEVICE_ARG (if)",
    )
    cmake = replace_once(
        cmake,
        'endif(ENABLE_UDEV OR ENABLE_LIBGPIOD OR ENABLE_JETSONNANOGPIO)',
        'endif(ENABLE_UDEV OR ENABLE_LIBGPIOD OR ENABLE_JETSONNANOGPIO OR ENABLE_RP1_PIO)',
        "CMake USE_DEVICE_ARG (endif)",
    )

    # Add link libraries after the last LIBGPIOD endif block
    # The linking section is near the end of the file (after target_link_libraries)
    link_anchor = 'endif(ENABLE_LIBGPIOD)\n'
    link_block = (
        '\nif (ENABLE_RP1_PIO)\n'
        '\ttarget_link_libraries(openFPGALoader rp1jtag)\n'
        '\t# PIOLib is a transitive dependency of librp1jtag (needed for static linking)\n'
        '\tfind_library(PIOLIB_LIBRARY pio)\n'
        '\tif(PIOLIB_LIBRARY)\n'
        '\t\ttarget_link_libraries(openFPGALoader ${PIOLIB_LIBRARY})\n'
        '\tendif()\n'
        '\tadd_definitions(-DENABLE_RP1_PIO=1)\n'
        'endif(ENABLE_RP1_PIO)\n'
    )
    if 'target_link_libraries(openFPGALoader rp1jtag)' not in cmake:
        # Find the LAST occurrence (the linking section, not the sources section)
        idx = cmake.rfind(link_anchor)
        if idx >= 0:
            insert_pos = idx + len(link_anchor)
            cmake = cmake[:insert_pos] + link_block + cmake[insert_pos:]
            print("  CMake link libraries: applied")
        else:
            print("  ERROR: CMake link libraries: anchor not found")
            sys.exit(1)
    else:
        print("  CMake link libraries: already applied, skipping")

    (ofl_dir / "CMakeLists.txt").write_text(cmake)

    print("\nDone! Build with:")
    print(f"  cd {ofl_dir}")
    print("  mkdir -p build && cd build")
    print("  cmake -DENABLE_RP1_PIO=ON ..")
    print("  make -j$(nproc)")


if __name__ == "__main__":
    main()
