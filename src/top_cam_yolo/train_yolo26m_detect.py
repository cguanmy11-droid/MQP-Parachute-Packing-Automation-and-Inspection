from __future__ import annotations

import argparse
import json
import random
import shutil
from pathlib import Path

import numpy as np
from ultralytics import YOLO
from ultralytics.cfg import DEFAULT_CFG_DICT


IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"}
CLASS_NAMES = ["holes"]
CLASS_ID = {"holes": 0}


def _safe_stem(name: str) -> str:
    return "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in name)


def _find_image_for_json(json_path: Path, image_path_hint: str | None) -> Path | None:
    if image_path_hint:
        p = (json_path.parent / image_path_hint).resolve()
        if p.exists():
            return p
        p2 = Path(image_path_hint)
        if p2.exists():
            return p2

    stem = json_path.stem
    for ext in IMG_EXTS:
        c = json_path.with_suffix(ext)
        if c.exists():
            return c
    for c in json_path.parent.glob(f"{stem}.*"):
        if c.suffix.lower() in IMG_EXTS:
            return c
    return None


def _shape_to_bbox(shape: dict, w: int, h: int):
    points = shape.get("points", [])
    if not points:
        return None
    pts = np.array(points, dtype=np.float32)
    if pts.size == 0:
        return None

    x1 = float(np.clip(np.min(pts[:, 0]), 0, w - 1))
    y1 = float(np.clip(np.min(pts[:, 1]), 0, h - 1))
    x2 = float(np.clip(np.max(pts[:, 0]), 0, w - 1))
    y2 = float(np.clip(np.max(pts[:, 1]), 0, h - 1))
    if x2 <= x1 or y2 <= y1:
        return None
    return x1, y1, x2, y2


def _bbox_xyxy_to_yolo(x1: float, y1: float, x2: float, y2: float, w: int, h: int):
    cx = (x1 + x2) / 2.0 / w
    cy = (y1 + y2) / 2.0 / h
    bw = (x2 - x1) / w
    bh = (y2 - y1) / h
    cx = float(np.clip(cx, 0.0, 1.0))
    cy = float(np.clip(cy, 0.0, 1.0))
    bw = float(np.clip(bw, 0.0, 1.0))
    bh = float(np.clip(bh, 0.0, 1.0))
    return cx, cy, bw, bh


def normalize_labelme_to_holes(labelme_dir: Path) -> tuple[int, int, int, int]:
    json_files = sorted(labelme_dir.glob("*.json"))
    changed_files = 0
    changed_shapes = 0
    total_shapes = 0

    for json_path in json_files:
        data = json.loads(json_path.read_text(encoding="utf-8"))
        shapes = data.get("shapes", [])
        file_changed = 0
        for shape in shapes:
            total_shapes += 1
            if shape.get("label") != "holes":
                shape["label"] = "holes"
                file_changed += 1

        if file_changed > 0:
            changed_files += 1
            changed_shapes += file_changed
            json_path.write_text(
                json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8"
            )

    return len(json_files), total_shapes, changed_files, changed_shapes


def build_det_dataset_from_labelme(
    labelme_dir: Path,
    out_dir: Path,
    val_ratio: float = 0.2,
    seed: int = 42,
    min_size: int = 4,
):
    if out_dir.exists():
        shutil.rmtree(out_dir)

    for split in ("train", "val"):
        (out_dir / "images" / split).mkdir(parents=True, exist_ok=True)
        (out_dir / "labels" / split).mkdir(parents=True, exist_ok=True)

    json_files = sorted(labelme_dir.glob("*.json"))
    if not json_files:
        raise FileNotFoundError(f"未找到标注文件: {labelme_dir}")

    samples = []
    for json_path in json_files:
        meta = json.loads(json_path.read_text(encoding="utf-8"))
        img_path = _find_image_for_json(json_path, meta.get("imagePath"))
        if img_path is None or not img_path.exists():
            continue

        h = int(meta.get("imageHeight", 0))
        w = int(meta.get("imageWidth", 0))
        if w <= 0 or h <= 0:
            # 兜底: 从图片读取尺寸
            from PIL import Image

            with Image.open(img_path) as im:
                w, h = im.size

        yolo_boxes = []
        for shape in meta.get("shapes", []):
            bbox = _shape_to_bbox(shape, w, h)
            if bbox is None:
                continue
            x1, y1, x2, y2 = bbox
            if (x2 - x1) < min_size or (y2 - y1) < min_size:
                continue
            cx, cy, bw, bh = _bbox_xyxy_to_yolo(x1, y1, x2, y2, w, h)
            yolo_boxes.append((CLASS_ID["holes"], cx, cy, bw, bh))

        # 即使空标注也保留图片并写空txt，符合YOLO检测数据集格式
        samples.append((json_path, img_path, yolo_boxes))

    if not samples:
        raise RuntimeError("没有从 Labelme 标注中解析出任何样本。")

    rng = random.Random(seed)
    rng.shuffle(samples)
    n_val = max(1, int(len(samples) * val_ratio)) if len(samples) > 1 else 0
    val_set = {s[0] for s in samples[:n_val]}

    train_count = 0
    val_count = 0
    train_boxes = 0
    val_boxes = 0

    for json_path, img_path, boxes in samples:
        split = "val" if json_path in val_set else "train"
        stem = _safe_stem(json_path.stem)
        dst_img = out_dir / "images" / split / f"{stem}{img_path.suffix.lower()}"
        dst_lbl = out_dir / "labels" / split / f"{stem}.txt"
        shutil.copy2(img_path, dst_img)

        lines = [f"{cls_id} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}" for cls_id, cx, cy, bw, bh in boxes]
        dst_lbl.write_text("\n".join(lines), encoding="utf-8")

        if split == "train":
            train_count += 1
            train_boxes += len(boxes)
        else:
            val_count += 1
            val_boxes += len(boxes)

    yaml_path = out_dir / "dataset.yaml"
    yaml_text = (
        f"path: {out_dir.as_posix()}\n"
        "train: images/train\n"
        "val: images/val\n"
        f"nc: {len(CLASS_NAMES)}\n"
        f"names: {CLASS_NAMES}\n"
    )
    yaml_path.write_text(yaml_text, encoding="utf-8")

    print("[INFO] 检测数据集构建完成")
    print(f"  train images={train_count}, train boxes={train_boxes}")
    print(f"  val   images={val_count}, val   boxes={val_boxes}")
    print(f"  dataset yaml={yaml_path}")

    return yaml_path


def parse_args():
    parser = argparse.ArgumentParser(description="YOLO26m 检测训练脚本（Labelme -> YOLO + 全量增强参数）")
    parser.add_argument("--labelme_dir", type=str, default="Cropped_img", help="Labelme 数据目录（json + image）")
    parser.add_argument("--dataset_dir", type=str, default="det_dataset_holes", help="输出检测数据集目录")
    parser.add_argument("--rebuild_dataset", action="store_true", help="强制重建数据集")
    parser.add_argument("--weights", type=str, default="yolo26m.pt", help="初始检测权重")
    parser.add_argument("--epochs", type=int, default=120)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--val_ratio", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--device", type=str, default="0", help="如 0 / 0,1 / cpu")
    parser.add_argument("--project", type=str, default="runs/detect")
    parser.add_argument("--name", type=str, default="yolo26m_holes_all_aug")
    parser.add_argument("--build_only", action="store_true", help="仅构建数据集，不启动训练")
    return parser.parse_args()


def main():
    args = parse_args()
    root = Path(__file__).resolve().parent
    labelme_dir = (root / args.labelme_dir).resolve()
    dataset_dir = (root / args.dataset_dir).resolve()

    total_json, total_shapes, changed_files, changed_shapes = normalize_labelme_to_holes(labelme_dir)
    print(
        "[INFO] 标注统一结果: "
        f"json_files={total_json}, total_shapes={total_shapes}, "
        f"changed_files={changed_files}, changed_shapes={changed_shapes}"
    )

    yaml_path = dataset_dir / "dataset.yaml"
    need_build = args.rebuild_dataset or not yaml_path.exists()
    if need_build:
        yaml_path = build_det_dataset_from_labelme(
            labelme_dir=labelme_dir,
            out_dir=dataset_dir,
            val_ratio=args.val_ratio,
            seed=args.seed,
        )
    else:
        print(f"[INFO] 使用现有检测数据集: {dataset_dir}")

    # 覆盖并显式传入文档里的增强参数。
    # 注意：auto_augment/erasing 是分类增强参数，copy_paste 是分割增强参数，
    # 在检测任务中通常不生效，但这里仍统一显式传入，便于集中管理与后续切任务复用。
    aug_args = {
        "hsv_h": 0.015,
        "hsv_s": 0.7,
        "hsv_v": 0.4,
        "degrees": 0.0,
        "translate": 0.1,
        "scale": 0.5,
        "shear": 0.0,
        "perspective": 0.0,
        "flipud": 0.0,
        "fliplr": 0.5,
        "bgr": 0.0,
        "mosaic": 1.0,
        "mixup": 0.0,
        "cutmix": 0.0,
        "close_mosaic": 10,
        "copy_paste": 0.0,
        "copy_paste_mode": "flip",
        "auto_augment": "randaugment",
        "erasing": 0.4,
        "augmentations": None,
    }

    # 仅传入当前 Ultralytics 配置支持的键，避免版本差异导致报错。
    supported_aug_args = {k: v for k, v in aug_args.items() if k in DEFAULT_CFG_DICT}
    skipped_aug_args = sorted(set(aug_args.keys()) - set(supported_aug_args.keys()))
    if skipped_aug_args:
        print(f"[WARN] 当前版本不支持以下增强参数，已自动跳过: {skipped_aug_args}")

    if args.build_only:
        print("[INFO] --build_only 已启用，跳过训练。")
        return

    model = YOLO(str((root / args.weights).resolve()))
    model.train(
        data=str(yaml_path),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project=args.project,
        name=args.name,
        seed=args.seed,
        pretrained=True,
        **supported_aug_args,
    )


if __name__ == "__main__":
    main()

