#!/usr/bin/env python3
"""Assemble the published static openFPGALoader files.

The same directory is published to GitHub Pages (next to the apt
repository, under static/) and as the assets of a GitHub release, so a
consumer gets identical names and checksums from either place.

Input: the static-openfpgaloader-<arch> artifacts of
.github/workflows/static-openfpgaloader.yml, each holding openFPGALoader,
openfpgaloader-commit.txt (the openFPGALoader commit it was built from) and
openfpgaloader-share/ (its data: the spiOverJtag bridges).

Output, in OUT_DIR:
    openFPGALoader-static-<arch>          the binary alone
    openFPGALoader-static-<arch>.tar.gz   openFPGALoader-static-<arch>/bin/openFPGALoader
                                          and .../share/openFPGALoader/ (the bridges)
    <each of the above>.sha256            "<sha256>  <name>", sha256sum -c format
    SHA256SUMS                            every file above, sha256sum -c format
    latest.json                           what was built, from what, and where

Every binary is checked to be a statically linked ELF for its architecture
that loads its data from /usr/local/share/openFPGALoader, every tarball to
hold the bridges of the boards it is used on, and all must be built from
the same openFPGALoader commit.

The binary alone needs the bridge given with --bridge, or a directory of
bridges in OPENFPGALOADER_SOJ_DIR, for -f / --flash-info. The tarball needs
nothing more once extracted with `tar -xzf <tarball> --strip-components=1
-C /usr/local` (bin/openFPGALoader, share/openFPGALoader/); extracted
anywhere else, set OPENFPGALOADER_SOJ_DIR=<dir>/share/openFPGALoader.

Usage:
    python3 packaging/static-site.py ARTIFACT_DIR OUT_DIR \\
        --version 0.0.post84 --commit <rp1-jtag sha> \\
        --openfpgaloader-repo URL --openfpgaloader-ref REF \\
        --release-tag TAG --repo mithro/rp1-jtag --pages-url URL
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import struct
import sys
import tarfile
from pathlib import Path

# artifact arch -> (ELF e_machine, ELF class, `uname -m` values it runs on)
ARCHES = {
    "arm64": (183, 2, ["aarch64"]),
    "armv7": (40, 1, ["armv7l", "aarch64"]),
    "armv6": (40, 1, ["armv6l", "armv7l", "aarch64"]),
}
PT_INTERP = 3
DATA_DIR = b"/usr/local/share/openFPGALoader"
# bridges of the boards these binaries are used on: Arty A7, NeTV2 35T/100T,
# Acorn CLE-101/215
BRIDGES = ["xc7a35tcsg324", "xc7a35tfgg484", "xc7a100tfgg484", "xc7a200tsbg484"]


def elf_check(path: Path, machine: int, elf_class: int) -> None:
    """Refuse anything that is not a static ELF for this machine."""
    data = path.read_bytes()
    if data[:4] != b"\x7fELF":
        sys.exit(f"ERROR: {path}: not an ELF file")
    if data[4] != elf_class or data[5] != 1:
        sys.exit(f"ERROR: {path}: ELF class {data[4]} / data {data[5]}, "
                 f"expected class {elf_class}, little endian")
    (e_machine,) = struct.unpack_from("<H", data, 18)
    if e_machine != machine:
        sys.exit(f"ERROR: {path}: e_machine {e_machine}, expected {machine}")
    if elf_class == 2:
        e_phoff, = struct.unpack_from("<Q", data, 32)
        e_phentsize, e_phnum = struct.unpack_from("<HH", data, 54)
    else:
        e_phoff, = struct.unpack_from("<I", data, 28)
        e_phentsize, e_phnum = struct.unpack_from("<HH", data, 42)
    for i in range(e_phnum):
        (p_type,) = struct.unpack_from("<I", data, e_phoff + i * e_phentsize)
        if p_type == PT_INTERP:
            sys.exit(f"ERROR: {path}: has a program interpreter, not static")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("artifact_dir", type=Path)
    ap.add_argument("out_dir", type=Path)
    ap.add_argument("--version", required=True)
    ap.add_argument("--commit", required=True)
    ap.add_argument("--openfpgaloader-repo", required=True)
    ap.add_argument("--openfpgaloader-ref", required=True)
    ap.add_argument("--release-tag", required=True)
    ap.add_argument("--repo", required=True)
    ap.add_argument("--pages-url", required=True)
    args = ap.parse_args()

    if args.out_dir.exists():
        sys.exit(f"ERROR: {args.out_dir} already exists")
    args.out_dir.mkdir(parents=True)

    files, sums, ofl_commits = [], [], set()
    for arch, (machine, elf_class, uname) in ARCHES.items():
        src = args.artifact_dir / f"static-openfpgaloader-{arch}"
        binary = src / "openFPGALoader"
        commit_file = src / "openfpgaloader-commit.txt"
        if not binary.is_file() or not commit_file.is_file():
            sys.exit(f"ERROR: {src}: missing openFPGALoader or openfpgaloader-commit.txt")
        elf_check(binary, machine, elf_class)
        if DATA_DIR + b"\0" not in binary.read_bytes():
            sys.exit(f"ERROR: {binary}: does not load its data from {DATA_DIR.decode()}")
        share = src / "openfpgaloader-share"
        for bridge in BRIDGES:
            if not (share / f"spiOverJtag_{bridge}.bit.gz").is_file():
                sys.exit(f"ERROR: {share}: spiOverJtag_{bridge}.bit.gz missing")
        ofl_commits.add(commit_file.read_text().strip())

        name = f"openFPGALoader-static-{arch}"
        dst = args.out_dir / name
        shutil.copyfile(binary, dst)
        os.chmod(dst, 0o755)

        tgz = args.out_dir / f"{name}.tar.gz"
        with tarfile.open(tgz, "w:gz") as t:
            def norm(ti: tarfile.TarInfo) -> tarfile.TarInfo:
                ti.uid = ti.gid = 0
                ti.uname = ti.gname = "root"
                ti.mode = 0o755 if ti.isdir() or ti.name.endswith("/bin/openFPGALoader") else 0o644
                return ti
            t.add(dst, arcname=f"{name}/bin/openFPGALoader", filter=norm)
            t.add(share, arcname=f"{name}/share/openFPGALoader", filter=norm)

        for kind, path in (("binary", dst), ("tarball", tgz)):
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            line = f"{digest}  {path.name}\n"
            (args.out_dir / f"{path.name}.sha256").write_text(line)
            sums.append(line)
            files.append({
                "arch": arch,
                "kind": kind,
                "uname_m": uname,
                "name": path.name,
                "sha256": digest,
                "size": path.stat().st_size,
                "url": {
                    "release": f"https://github.com/{args.repo}/releases/download/"
                               f"{args.release_tag}/{path.name}",
                    "release_latest": f"https://github.com/{args.repo}/releases/latest/"
                                      f"download/{path.name}",
                    "pages": f"{args.pages_url}/{path.name}",
                },
            })

    if len(ofl_commits) != 1:
        sys.exit(f"ERROR: binaries built from different openFPGALoader commits: "
                 f"{sorted(ofl_commits)}")
    (ofl_commit,) = ofl_commits

    (args.out_dir / "SHA256SUMS").write_text("".join(sums))
    latest = {
        "format": "rp1-jtag-static-openfpgaloader",
        "version": 1,
        "rp1_jtag": {"version": args.version, "commit": args.commit},
        "openfpgaloader": {
            "repo": args.openfpgaloader_repo,
            "ref": args.openfpgaloader_ref,
            "commit": ofl_commit,
        },
        "release_tag": args.release_tag,
        "data_dir": DATA_DIR.decode(),
        "files": files,
    }
    (args.out_dir / "latest.json").write_text(json.dumps(latest, indent=2) + "\n")
    for p in sorted(args.out_dir.iterdir()):
        print(f"{p.stat().st_size:>10}  {p.name}")


if __name__ == "__main__":
    main()
