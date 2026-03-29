import argparse
import random
import time
from pathlib import Path
from typing import List

import cv2
import numpy as np


IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Augment cls_dataset train/fully and train/partial to target count."
    )
    parser.add_argument("--dataset_root", type=str, default="cls_dataset", help="分类数据集根目录")
    parser.add_argument("--split", type=str, default="val", help="数据集 split（默认 train）")
    parser.add_argument(
        "--classes",
        nargs="+",
        default=["Fully_Insert", "Partial_Insert"],
        help="需要补齐的类别名（默认 fully partial）",
    )
    parser.add_argument("--target_count", type=int, default=276, help="每个类别目标数量")
    parser.add_argument("--seed", type=int, default=42, help="随机种子")
    parser.add_argument("--dry_run", action="store_true", help="只统计，不实际写入")
    return parser.parse_args()


def list_images(folder: Path) -> List[Path]:
    return [p for p in folder.iterdir() if p.is_file() and p.suffix.lower() in IMG_EXTS]


def apply_gaussian_blur(img: np.ndarray) -> np.ndarray:
    k = random.choice([3, 5, 7])
    sigma = random.uniform(0.15, 1.8)
    return cv2.GaussianBlur(img, (k, k), sigmaX=sigma, sigmaY=sigma)


def motion_blur_kernel(ksize: int, angle_deg: float) -> np.ndarray:
    kernel = np.zeros((ksize, ksize), dtype=np.float32)
    kernel[ksize // 2, :] = 1.0
    center = (ksize / 2 - 0.5, ksize / 2 - 0.5)
    rot = cv2.getRotationMatrix2D(center, angle_deg, 1.0)
    kernel = cv2.warpAffine(kernel, rot, (ksize, ksize))
    s = kernel.sum()
    if s <= 1e-6:
        return np.eye(ksize, dtype=np.float32) / ksize
    return kernel / s


def apply_motion_blur(img: np.ndarray) -> np.ndarray:
    k = random.choice([3, 5, 7])
    angle = random.uniform(-37.5, 37.5)
    kernel = motion_blur_kernel(k, angle)
    return cv2.filter2D(img, -1, kernel)


def apply_noise(img: np.ndarray) -> np.ndarray:
    std = random.uniform(3.0, 12.0)
    noise = np.random.normal(0.0, std, size=img.shape).astype(np.float32)
    out = img.astype(np.float32) + noise
    return np.clip(out, 0, 255).astype(np.uint8)


def apply_jpeg_artifact(img: np.ndarray) -> np.ndarray:
    quality = random.randint(45, 90)
    ok, enc = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        return img
    dec = cv2.imdecode(enc, cv2.IMREAD_COLOR)
    return dec if dec is not None else img


def apply_hsv_jitter(img: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(np.float32)
    h_shift = random.uniform(-7.5, 7.5)
    s_scale = random.uniform(0.85, 1.30)
    v_scale = random.uniform(0.88, 1.22)

    hsv[..., 0] = (hsv[..., 0] + h_shift) % 180.0
    hsv[..., 1] = np.clip(hsv[..., 1] * s_scale, 0, 255)
    hsv[..., 2] = np.clip(hsv[..., 2] * v_scale, 0, 255)
    out = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)
    return out


def augment_image(img: np.ndarray) -> np.ndarray:
    out = img.copy()
    changed = False

    # 全局门控：90% 概率执行增强，10% 保持原图复制
    if random.random() >= 0.90:
        return out

    # A. 轻模糊：Gaussian / Motion Blur，强度提升 50%（p=0.45）
    if random.random() < 0.45:
        if random.random() < 0.65:
            out = apply_gaussian_blur(out)
        else:
            out = apply_motion_blur(out)
        changed = True

    # B. 轻微噪声 / JPEG artifact，强度提升 50%（p=0.30）
    if random.random() < 0.30:
        if random.random() < 0.60:
            out = apply_noise(out)
        else:
            out = apply_jpeg_artifact(out)
        changed = True

    # 明亮、饱和度、HSV 微调，强度提升后每次都应用
    # if random.random() < 1.0:
    #     out = apply_hsv_jitter(out)
    #     changed = True

    # 保底：在进入增强分支后，至少做一次变化
    if not changed:
        out = apply_hsv_jitter(out)
    return out


def augment_class_dir(class_dir: Path, target_count: int, dry_run: bool = False) -> None:
    if not class_dir.exists() or not class_dir.is_dir():
        print(f"[WARN] 类别目录不存在，跳过: {class_dir}")
        return

    originals = list_images(class_dir)
    current = len(originals)
    need = max(0, target_count - current)

    print(f"\n[INFO] {class_dir}")
    print(f"       当前: {current} | 目标: {target_count} | 需新增: {need}")

    if current == 0:
        print("       [WARN] 无原始图片可供增强，跳过。")
        return
    if need == 0:
        print("       [OK] 已达到或超过目标数量，无需新增。")
        return
    if dry_run:
        print("       [DRY RUN] 不会实际写入文件。")
        return

    ts = int(time.time())
    for i in range(need):
        src = random.choice(originals)
        img = cv2.imread(str(src), cv2.IMREAD_COLOR)
        if img is None:
            print(f"       [WARN] 读取失败，跳过: {src.name}")
            continue
        aug = augment_image(img)
        out_name = f"{src.stem}_aug_{ts}_{i:05d}.jpg"
        out_path = class_dir / out_name
        cv2.imwrite(str(out_path), aug, [int(cv2.IMWRITE_JPEG_QUALITY), random.randint(85, 96)])

    final_count = len(list_images(class_dir))
    print(f"       [DONE] 最终数量: {final_count}")


def main() -> None:
    args = parse_args()
    random.seed(args.seed)
    np.random.seed(args.seed)

    dataset_root = Path(args.dataset_root)
    train_root = dataset_root / args.split
    if not train_root.exists():
        raise FileNotFoundError(f"找不到目录: {train_root}")

    print(f"[INFO] 数据集根目录: {dataset_root.resolve()}")
    print(f"[INFO] split: {args.split}, classes: {args.classes}, target_count: {args.target_count}")

    for cls_name in args.classes:
        class_dir = train_root / cls_name
        augment_class_dir(class_dir, target_count=args.target_count, dry_run=args.dry_run)

    print("\n[INFO] 完成。")


if __name__ == "__main__":
    main()
