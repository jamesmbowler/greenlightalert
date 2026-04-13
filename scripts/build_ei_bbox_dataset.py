#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path


IMAGE_EXTS = {".jpg", ".jpeg", ".png"}


def parse_key_value_file(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for line in path.read_text().splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key] = value
    return values


def find_matching_image(root: Path, sequence: str) -> Path | None:
    matches = []
    for path in root.iterdir():
        if path.suffix.lower() not in IMAGE_EXTS:
            continue
        parts = path.stem.split("_")
        if len(parts) < 3:
            continue
        if parts[-2] == sequence and "_annotated" not in path.stem and not path.stem.startswith("ei_"):
            matches.append(path)
    matches.sort()
    return matches[0] if matches else None


def build_box(values: dict[str, str], label: str) -> dict[str, int | str] | None:
    try:
        x = int(values["bbox_x_min"])
        y = int(values["bbox_y_min"])
        width = int(values["bbox_width"])
        height = int(values["bbox_height"])
    except KeyError:
        return None

    if width <= 0 or height <= 0:
        return None

    return {
        "label": label,
        "x": x,
        "y": y,
        "width": width,
        "height": height,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Build an Edge Impulse bounding_boxes.labels dataset from trafficimages sidecars."
    )
    parser.add_argument(
        "--input-dir",
        default="/Users/jamesbowler/esp32/trafficimages",
        help="Directory containing images and red_frame_boxes text sidecars.",
    )
    parser.add_argument(
        "--output-dir",
        default="/Users/jamesbowler/esp32/ei_bbox_dataset/training",
        help="Output folder to write copied images and bounding_boxes.labels.",
    )
    parser.add_argument(
        "--label",
        default="traffic_light",
        help="Label name to assign to each bounding box.",
    )
    parser.add_argument(
        "--copy-images",
        action="store_true",
        help="Copy matching images into the output directory.",
    )
    args = parser.parse_args()

    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    labels: dict[str, list[dict[str, int | str]]] = {}

    for txt_path in sorted(input_dir.glob("red_frame_boxes_*.txt")):
        parts = txt_path.stem.split("_")
        if len(parts) < 4:
            continue
        sequence = parts[-2]
        values = parse_key_value_file(txt_path)
        image_path = find_matching_image(input_dir, sequence)
        if image_path is None:
            continue

        box = build_box(values, args.label)
        if box is None:
            continue

        labels[image_path.name] = [box]
        if args.copy_images:
            shutil.copy2(image_path, output_dir / image_path.name)

    out = {
        "version": 1,
        "type": "bounding-box-labels",
        "boundingBoxes": labels,
    }

    out_path = output_dir / "bounding_boxes.labels"
    out_path.write_text(json.dumps(out, indent=2))
    print(f"Wrote {out_path} with {len(labels)} labeled images")
    if not args.copy_images:
        print("Run again with --copy-images if you also want the matching images copied.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
