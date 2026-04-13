#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

import cv2


IMAGE_EXTS = {".jpg", ".jpeg", ".png"}


def parse_kv(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for line in path.read_text().splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key] = value
    return values


def extract_sequence(path: Path) -> str | None:
    parts = path.stem.split("_")
    if len(parts) < 3:
        return None
    return parts[-2]


def find_image(root: Path, sequence: str) -> Path | None:
    candidates: list[Path] = []
    for path in root.iterdir():
        if path.suffix.lower() not in IMAGE_EXTS:
            continue
        if "_annotated" in path.stem:
            continue
        if path.stem.startswith("ei_"):
            continue
        if extract_sequence(path) == sequence:
            candidates.append(path)
    candidates.sort()
    return candidates[0] if candidates else None


def annotate_image(image_path: Path, info: dict[str, str]) -> Path:
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"Failed to load image: {image_path}")

    box_count = int(info.get("boxes_count", "0"))
    for i in range(box_count):
        try:
            x1 = int(info[f"box_{i}_x_min"])
            y1 = int(info[f"box_{i}_y_min"])
            x2 = int(info[f"box_{i}_x_max"])
            y2 = int(info[f"box_{i}_y_max"])
            conf = float(info[f"box_{i}_confidence"])
        except KeyError:
            continue

        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 200, 255), 1)
        cv2.putText(
            image,
            f"{conf:.2f}",
            (x1, max(14, y1 - 4)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (0, 200, 255),
            1,
            cv2.LINE_AA,
        )

    if info.get("red_found") == "1":
        x1 = int(info["bbox_x_min"])
        y1 = int(info["bbox_y_min"])
        x2 = int(info["bbox_x_max"])
        y2 = int(info["bbox_y_max"])
        cx = int(info["center_x"])
        cy = int(info["center_y"])
        label = "red_ei" if info.get("red_found_by_ei") == "1" else "red"
        conf = info.get("ei_confidence", "")

        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
        cv2.circle(image, (cx, cy), 4, (0, 255, 255), cv2.FILLED)
        title = label if not conf else f"{label} {float(conf):.2f}"
        cv2.putText(
            image,
            title,
            (x1, max(18, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

    out_path = image_path.parent / f"{image_path.stem}_annotated{image_path.suffix}"
    if not cv2.imwrite(str(out_path), image):
        raise RuntimeError(f"Failed to save annotated image: {out_path}")
    return out_path


def main() -> int:
    parser = argparse.ArgumentParser(description="Annotate trafficimages from red_frame_boxes sidecars.")
    parser.add_argument(
        "--dir",
        default="/Users/jamesbowler/esp32/trafficimages",
        help="trafficimages directory",
    )
    args = parser.parse_args()

    root = Path(args.dir)
    if not root.is_dir():
        raise SystemExit(f"Directory not found: {root}")

    count = 0
    for txt_path in sorted(root.glob("red_frame_boxes_*.txt")):
        sequence = extract_sequence(txt_path)
        if sequence is None:
            continue
        image_path = find_image(root, sequence)
        if image_path is None:
            continue
        out_path = annotate_image(image_path, parse_kv(txt_path))
        print(f"Annotated {out_path}")
        count += 1

    print(f"Annotated {count} images")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
