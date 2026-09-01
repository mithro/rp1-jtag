#!/usr/bin/env python3
"""Derive the Debian package version from ``git describe``.

Series tags are two-component, ``vX.Y`` (``v0.0`` is the convention for the
root commit). At tag ``vX.Y`` the version is ``X.Y``; N commits later it is
``X.Y.postN``. With no matching tag it falls back to ``0.0.post<commit count>``.

The version increments on every commit, so each push to ``main`` publishes a
new, upgradeable package with no manual bump and no tag. All forms are valid
Debian versions verbatim, and match the scheme used by the other mithro apt
repositories (see mithro/apt-repo-action).

Usage:
    python3 packaging/deb-version.py                    # print the version
    python3 packaging/deb-version.py --write-changelog  # stamp debian/changelog
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from email.utils import formatdate
from pathlib import Path

_DEFAULT_REPO = Path(__file__).resolve().parent.parent
REPO = Path(os.environ.get("DEB_VERSION_REPO", _DEFAULT_REPO))
CHANGELOG = REPO / "debian" / "changelog"
SOURCE = "rp1-jtag"
MAINTAINER = "Tim 'mithro' Ansell <me@mith.ro>"

DESCRIBE_RE = re.compile(r"^v(\d+\.\d+)-(\d+)-g[0-9a-f]+$")


def _git(*args: str) -> str:
    return subprocess.run(
        ["git", "-C", str(REPO), *args],
        capture_output=True, text=True, check=True,
    ).stdout.strip()


def version() -> str:
    """``X.Y`` exactly at a series tag, ``X.Y.postN`` after it."""
    try:
        described = _git("describe", "--tags", "--long", "--match", "v[0-9]*.[0-9]*")
    except subprocess.CalledProcessError:
        # No series tag reachable (or a shallow clone with none fetched): fall
        # back to counting commits so the version still increases monotonically.
        try:
            count = _git("rev-list", "--count", "HEAD")
        except subprocess.CalledProcessError:
            print("ERROR: not a git repository, or no commits", file=sys.stderr)
            sys.exit(1)
        return f"0.0.post{count}"

    m = DESCRIBE_RE.match(described)
    if not m:
        print(f"ERROR: unexpected git describe output: {described!r}", file=sys.stderr)
        sys.exit(1)
    series, distance = m.group(1), int(m.group(2))
    return series if distance == 0 else f"{series}.post{distance}"


def write_changelog(ver: str) -> None:
    """Rewrite debian/changelog with this version.

    The timestamp comes from the HEAD commit so a rebuild of the same commit is
    byte-identical rather than churning on every run.
    """
    try:
        date = _git("log", "-1", "--format=%cd", "--date=rfc2822")
    except subprocess.CalledProcessError:
        date = formatdate(localtime=True)
    CHANGELOG.write_text(
        f"{SOURCE} ({ver}) unstable; urgency=medium\n"
        f"\n"
        f"  * Rolling build from git ({ver}).\n"
        f"\n"
        f" -- {MAINTAINER}  {date}\n"
    )


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--write-changelog", action="store_true",
                    help="stamp debian/changelog with the derived version")
    args = ap.parse_args()

    ver = version()
    if args.write_changelog:
        write_changelog(ver)
        print(f"wrote {CHANGELOG} ({ver})", file=sys.stderr)
    print(ver)


if __name__ == "__main__":
    main()
