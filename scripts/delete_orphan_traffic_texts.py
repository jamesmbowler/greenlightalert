#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".ppm"}


def extract_sequence(path: Path) -> str | None:
    parts = path.stem.split("_")
    if len(parts) < 3:
      return None
    return parts[-2]


def image_sequences(paths: Iterable[Path]) -> set[str]:
    sequences: set[str] = set()
    for path in paths:
        if path.suffix.lower() not in IMAGE_SUFFIXES:
            continue
        sequence = extract_sequence(path)
        if sequence is not None:
            sequences.add(sequence)
    return sequences


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Delete text files in trafficimages that do not have any matching image sequence."
    )
    parser.add_argument(
        "--dir",
        default="/Users/jamesbowler/esp32/trafficimages",
        help="Directory to scan. Defaults to the local trafficimages folder.",
    )
    parser.add_argument(
        "--apply",
        action="store_true",
        help="Actually delete orphaned text files. Without this flag, only print what would be deleted.",
    )
    args = parser.parse_args()

    root = Path(args.dir)
    if not root.is_dir():
        raise SystemExit(f"Directory not found: {root}")

    all_paths = list(root.iterdir())
    sequences = image_sequences(all_paths)
    orphaned: list[Path] = []

    for path in sorted(all_paths):
        if path.suffix.lower() != ".txt":
            continue
        sequence = extract_sequence(path)
        if sequence is None or sequence not in sequences:
            orphaned.append(path)

    if not orphaned:
        print("No orphaned text files found.")
        return 0

    for path in orphaned:
        if args.apply:
            path.unlink()
            print(f"Deleted {path}")
        else:
            print(f"Would delete {path}")

    if not args.apply:
        print("Run again with --apply to delete these files.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
