#!/usr/bin/env python3

import argparse
import json
import random
import shutil
from pathlib import Path

import cv2


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Sample COCO train/valid splits, resize+crop images, and rewrite annotations."
    )
    parser.add_argument("--input", required=True, help="Input dataset root")
    parser.add_argument("--output", required=True, help="Output dataset root")
    parser.add_argument("--size", type=int, default=160, help="Output square size")
    parser.add_argument("--train-count", type=int, default=500, help="Train images to sample")
    parser.add_argument("--valid-count", type=int, default=500, help="Valid images to sample")
    parser.add_argument("--seed", type=int, default=42, help="Sampling seed")
    return parser.parse_args()


def copy_root_files(src_root: Path, dst_root: Path):
    dst_root.mkdir(parents=True, exist_ok=True)
    for child in src_root.iterdir():
        if child.is_file():
            shutil.copy2(child, dst_root / child.name)


def copy_split_non_images(src_split: Path, dst_split: Path):
    dst_split.mkdir(parents=True, exist_ok=True)
    for child in src_split.iterdir():
        if child.is_file() and child.suffix.lower() not in IMAGE_EXTS and child.name != "_annotations.coco.json":
            shutil.copy2(child, dst_split / child.name)


def load_coco(path: Path):
    with path.open() as fh:
        return json.load(fh)


def choose_images(images, count, rng):
    count = min(count, len(images))
    return rng.sample(images, count)


def transform_bbox(bbox, scale, crop_x, crop_y, size):
    x, y, w, h = bbox
    x1 = x * scale - crop_x
    y1 = y * scale - crop_y
    x2 = (x + w) * scale - crop_x
    y2 = (y + h) * scale - crop_y

    x1 = max(0.0, min(float(size), x1))
    y1 = max(0.0, min(float(size), y1))
    x2 = max(0.0, min(float(size), x2))
    y2 = max(0.0, min(float(size), y2))

    new_w = x2 - x1
    new_h = y2 - y1
    if new_w <= 1.0 or new_h <= 1.0:
        return None
    return [x1, y1, new_w, new_h]


def resize_and_crop(image, size):
    height, width = image.shape[:2]
    scale = max(size / width, size / height)
    resized_w = max(size, int(round(width * scale)))
    resized_h = max(size, int(round(height * scale)))
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    resized = cv2.resize(image, (resized_w, resized_h), interpolation=interpolation)
    crop_x = max(0, (resized_w - size) // 2)
    crop_y = max(0, (resized_h - size) // 2)
    cropped = resized[crop_y:crop_y + size, crop_x:crop_x + size]
    return cropped, scale, crop_x, crop_y


def process_split(src_root: Path, dst_root: Path, split: str, count: int, size: int, rng):
    src_split = src_root / split
    dst_split = dst_root / split
    copy_split_non_images(src_split, dst_split)

    coco = load_coco(src_split / "_annotations.coco.json")
    selected_images = choose_images(coco["images"], count, rng)
    selected_ids = {image["id"] for image in selected_images}

    anns_by_image = {}
    for ann in coco["annotations"]:
        anns_by_image.setdefault(ann["image_id"], []).append(ann)

    out_images = []
    out_annotations = []

    for image_entry in selected_images:
        src_image = src_split / image_entry["file_name"]
        if not src_image.exists():
            continue

        image = cv2.imread(str(src_image))
        if image is None:
            continue

        cropped, scale, crop_x, crop_y = resize_and_crop(image, size)
        dst_image = dst_split / image_entry["file_name"]
        cv2.imwrite(str(dst_image), cropped)

        new_image = dict(image_entry)
        new_image["width"] = size
        new_image["height"] = size
        out_images.append(new_image)

        for ann in anns_by_image.get(image_entry["id"], []):
            new_bbox = transform_bbox(ann["bbox"], scale, crop_x, crop_y, size)
            if new_bbox is None:
                continue
            new_ann = dict(ann)
            new_ann["bbox"] = new_bbox
            new_ann["area"] = new_bbox[2] * new_bbox[3]
            if ann.get("segmentation"):
                new_ann["segmentation"] = []
            out_annotations.append(new_ann)

    new_coco = dict(coco)
    new_coco["images"] = out_images
    new_coco["annotations"] = out_annotations

    with (dst_split / "_annotations.coco.json").open("w") as fh:
        json.dump(new_coco, fh)

    print(f"{split}: wrote {len(out_images)} images and {len(out_annotations)} annotations")


def main():
    args = parse_args()
    src_root = Path(args.input).expanduser().resolve()
    dst_root = Path(args.output).expanduser().resolve()
    rng = random.Random(args.seed)

    copy_root_files(src_root, dst_root)
    process_split(src_root, dst_root, "train", args.train_count, args.size, rng)
    process_split(src_root, dst_root, "valid", args.valid_count, args.size, rng)


if __name__ == "__main__":
    main()
