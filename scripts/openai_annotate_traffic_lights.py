#!/usr/bin/env python3

from __future__ import annotations

import argparse
import base64
import json
import mimetypes
import os
import shutil
import sys
import urllib.error
import urllib.request
from pathlib import Path


DEFAULT_MODEL = "gpt-4.1-mini"
IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".webp"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Use the OpenAI API to pre-annotate traffic light images with bounding boxes."
    )
    parser.add_argument("--image", help="Single image to annotate.")
    parser.add_argument("--input-dir", help="Directory of images to annotate.")
    parser.add_argument(
        "--output-dir",
        default="/Users/jamesbowler/esp32/openai_annotations",
        help="Directory to write JSON outputs and optional Edge Impulse labels.",
    )
    parser.add_argument(
        "--model",
        default=DEFAULT_MODEL,
        help=f"Vision-capable model to use. Default: {DEFAULT_MODEL}",
    )
    parser.add_argument(
        "--label",
        default="traffic_light",
        help="Label name to emit in boxes and Edge Impulse output.",
    )
    parser.add_argument(
        "--max-images",
        type=int,
        default=0,
        help="Limit number of images from --input-dir. 0 means no limit.",
    )
    parser.add_argument(
        "--write-ei-labels",
        action="store_true",
        help="Also write bounding_boxes.labels in Edge Impulse format.",
    )
    parser.add_argument(
        "--copy-images",
        action="store_true",
        help="Copy images into the output dir when writing Edge Impulse labels.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Do not call the API; just print which files would be annotated.",
    )
    args = parser.parse_args()
    if not args.image and not args.input_dir:
        parser.error("Provide either --image or --input-dir")
    return args


def gather_images(args: argparse.Namespace) -> list[Path]:
    if args.image:
        return [Path(args.image).expanduser().resolve()]

    root = Path(args.input_dir).expanduser().resolve()
    images = [p for p in sorted(root.iterdir()) if p.suffix.lower() in IMAGE_EXTS]
    if args.max_images > 0:
        images = images[: args.max_images]
    return images


def image_to_data_url(path: Path) -> str:
    mime = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
    data = base64.b64encode(path.read_bytes()).decode("ascii")
    return f"data:{mime};base64,{data}"


def build_request_payload(model: str, image_data_url: str) -> dict:
    schema = {
        "name": "traffic_light_boxes",
        "strict": True,
        "schema": {
            "type": "object",
            "properties": {
                "boxes": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "label": {"type": "string"},
                            "x": {"type": "integer"},
                            "y": {"type": "integer"},
                            "width": {"type": "integer"},
                            "height": {"type": "integer"},
                            "confidence": {"type": "number"},
                        },
                        "required": ["label", "x", "y", "width", "height", "confidence"],
                        "additionalProperties": False,
                    },
                }
            },
            "required": ["boxes"],
            "additionalProperties": False,
        },
    }

    instructions = (
        "Detect visible traffic signal housings in this image. "
        "Prefer a bounding box around the entire traffic light fixture, not individual bulbs, "
        "unless only one bulb is actually visible. Return pixel coordinates in the original image. "
        "If there are no visible traffic lights, return an empty boxes array."
    )

    return {
        "model": model,
        "input": [
            {
                "role": "user",
                "content": [
                    {"type": "input_text", "text": instructions},
                    {"type": "input_image", "image_url": image_data_url},
                ],
            }
        ],
        "text": {
            "format": {
                "type": "json_schema",
                "name": schema["name"],
                "strict": schema["strict"],
                "schema": schema["schema"],
            }
        },
    }


def call_openai(api_key: str, payload: dict) -> dict:
    request = urllib.request.Request(
        "https://api.openai.com/v1/responses",
        data=json.dumps(payload).encode("utf-8"),
        headers={
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json",
        },
        method="POST",
    )
    with urllib.request.urlopen(request) as response:
        return json.loads(response.read().decode("utf-8"))


def extract_structured_output(response_json: dict) -> dict:
    for item in response_json.get("output", []):
        for content in item.get("content", []):
            if content.get("type") == "output_text" and content.get("text"):
                return json.loads(content["text"])
    raise RuntimeError("No structured JSON output found in response")


def clamp_box(box: dict, image_width: int, image_height: int, label: str) -> dict | None:
    try:
        x = int(box["x"])
        y = int(box["y"])
        width = int(box["width"])
        height = int(box["height"])
        confidence = float(box["confidence"])
    except (KeyError, TypeError, ValueError):
        return None

    x = max(0, min(image_width - 1, x))
    y = max(0, min(image_height - 1, y))
    width = max(1, min(image_width - x, width))
    height = max(1, min(image_height - y, height))
    if width <= 0 or height <= 0:
        return None

    return {
        "label": label,
        "x": x,
        "y": y,
        "width": width,
        "height": height,
        "confidence": confidence,
    }


def read_image_size(path: Path) -> tuple[int, int]:
    import cv2

    image = cv2.imread(str(path))
    if image is None:
        raise RuntimeError(f"Failed to load image: {path}")
    height, width = image.shape[:2]
    return width, height


def annotate_images(args: argparse.Namespace) -> int:
    images = gather_images(args)
    if not images:
        print("No images found.")
        return 0

    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    if args.dry_run:
        for image in images:
            print(f"Would annotate {image}")
        return 0

    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        raise SystemExit("OPENAI_API_KEY is not set")

    ei_boxes: dict[str, list[dict]] = {}

    for image in images:
        width, height = read_image_size(image)
        payload = build_request_payload(args.model, image_to_data_url(image))
        try:
            response_json = call_openai(api_key, payload)
        except urllib.error.HTTPError as exc:
            body = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"OpenAI API request failed for {image.name}: {exc.code} {body}") from exc

        structured = extract_structured_output(response_json)
        boxes = []
        for raw_box in structured.get("boxes", []):
            box = clamp_box(raw_box, width, height, args.label)
            if box:
                boxes.append(box)

        out_json = {
            "image": image.name,
            "width": width,
            "height": height,
            "model": args.model,
            "boxes": boxes,
            "response_id": response_json.get("id"),
        }
        out_path = output_dir / f"{image.stem}.openai_boxes.json"
        out_path.write_text(json.dumps(out_json, indent=2))
        print(f"Wrote {out_path}")

        if args.write_ei_labels:
            ei_boxes[image.name] = [
                {
                    "label": box["label"],
                    "x": box["x"],
                    "y": box["y"],
                    "width": box["width"],
                    "height": box["height"],
                }
                for box in boxes
            ]
            if args.copy_images:
                shutil_path = output_dir / image.name
                if not shutil_path.exists():
                    shutil_path.write_bytes(image.read_bytes())

    if args.write_ei_labels:
        labels_path = output_dir / "bounding_boxes.labels"
        labels = {
            "version": 1,
            "type": "bounding-box-labels",
            "boundingBoxes": ei_boxes,
        }
        labels_path.write_text(json.dumps(labels, indent=2))
        print(f"Wrote {labels_path}")

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(annotate_images(parse_args()))
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(1)
