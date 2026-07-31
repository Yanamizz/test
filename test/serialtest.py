#!/usr/bin/env python3

from __future__ import annotations

import pathlib
import subprocess
import sys


def main() -> int:
    repo_root = pathlib.Path(__file__).resolve().parents[1]
    binary = repo_root / "build" / "bin" / "radar"

    if not binary.exists():
        print(f"missing binary: {binary}")
        print("build with:")
        print("  cmake -S . -B build")
        print("  cmake --build build --target radar")
        return 1

    return subprocess.call([str(binary), *sys.argv[1:]], cwd=repo_root)


if __name__ == "__main__":
    raise SystemExit(main())
