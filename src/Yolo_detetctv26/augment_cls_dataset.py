"""
检测数据集像素级增强脚本

从 Raw+Fine_combine-formatted-aug 读取原始数据 (images/ + labels/),
将原始文件 + 增强文件一起写入 Raw+Fine_combine-formatted-aug+ 输出目录。

增强操作全部为像素级 (不改变 bbox), 因此标签文件直接复制。
原始文件名保持不变, 新增文件使用 {原名}_daug_{序号}.png/.txt 命名。
"""
import argparse
import random
import shutil
import time
from pathlib import Path
from typing import List

import cv2
import numpy as np


IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Augment detection dataset (images + labels) to target count."
    )
    parser.add_argument(
        "--dataset_root", type=str,
        default="Raw+Fine_combine-formatted-aug",
        help="输入数据集根目录 (包含 images/ 和 labels/ 子目录)",
    )
    parser.add_argument(
        "--output_root", type=str,
        default="Raw+Fine_combine-formatted-aug+",
        help="输出数据集根目录 (原始 + 增强结果)",
    )
    parser.add_argument("--target_count", type=int, default=500, help="目标图片总数")
    parser.add_argument("--seed", type=int, default=42, help="随机种子")
    parser.add_argument("--dry_run", action="store_true", help="只统计, 不实际写入")
    return parser.parse_args()


# ────────────────────────── 增强函数 ──────────────────────────

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
    return cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)


def augment_image(img: np.ndarray) -> np.ndarray:
    out = img.copy()
    changed = False

    # 全局门控: 90% 概率执行增强, 10% 保持原图复制
    if random.random() >= 0.90:
        return out

    # A. 轻模糊: Gaussian / Motion Blur (p=0.45)
    if random.random() < 0.45:
        if random.random() < 0.65:
            out = apply_gaussian_blur(out)
        else:
            out = apply_motion_blur(out)
        changed = True

    # B. 轻微噪声 / JPEG artifact (p=0.30)
    if random.random() < 0.30:
        if random.random() < 0.60:
            out = apply_noise(out)
        else:
            out = apply_jpeg_artifact(out)
        changed = True

    # 保底: 至少做一次 HSV 微调
    # if not changed:
    #     out = apply_hsv_jitter(out)
    return out


# ────────────────────────── 核心逻辑 ──────────────────────────

def list_images(folder: Path) -> List[Path]:
    return sorted(p for p in folder.iterdir() if p.is_file() and p.suffix.lower() in IMG_EXTS)


def augment_dataset(
    dataset_root: Path, output_root: Path, target_count: int, dry_run: bool = False
) -> None:
    src_img_dir = dataset_root / "images"
    src_lab_dir = dataset_root / "labels"

    if not src_img_dir.exists():
        raise FileNotFoundError(f"输入图片目录不存在: {src_img_dir}")
    if not src_lab_dir.exists():
        raise FileNotFoundError(f"输入标签目录不存在: {src_lab_dir}")

    # 输出目录
    out_img_dir = output_root / "images"
    out_lab_dir = output_root / "labels"

    originals = list_images(src_img_dir)
    current = len(originals)
    need = max(0, target_count - current)

    print(f"[INFO] 输入图片目录: {src_img_dir.resolve()}")
    print(f"[INFO] 输入标签目录: {src_lab_dir.resolve()}")
    print(f"[INFO] 输出目录:     {output_root.resolve()}")
    print(f"[INFO] 当前图片数: {current} | 目标: {target_count} | 需新增: {need}")

    if current == 0:
        print("[WARN] 无原始图片可供增强, 跳过。")
        return
    if dry_run:
        print("[DRY RUN] 不会实际写入文件。")
        return

    # 创建输出目录
    out_img_dir.mkdir(parents=True, exist_ok=True)
    out_lab_dir.mkdir(parents=True, exist_ok=True)

    # ── 第一步: 复制原始文件到输出目录 ──
    print("\n[STEP 1] 复制原始文件到输出目录...")
    copied = 0
    for img_path in originals:
        # 复制图片
        shutil.copy2(str(img_path), str(out_img_dir / img_path.name))
        # 复制对应标签
        lab_path = src_lab_dir / (img_path.stem + ".txt")
        if lab_path.exists():
            shutil.copy2(str(lab_path), str(out_lab_dir / lab_path.name))
        copied += 1
    print(f"  已复制: {copied} 张图片 + 对应标签")

    # ── 第二步: 生成增强文件到输出目录 ──
    if need == 0:
        print("\n[OK] 已达到或超过目标数量, 无需新增增强。")
    else:
        print(f"\n[STEP 2] 生成 {need} 张增强图片...")
        # 只从有对应 label 的图片中采样
        paired = [p for p in originals if (src_lab_dir / (p.stem + ".txt")).exists()]
        if not paired:
            print("[WARN] 无图片拥有对应的标签文件, 跳过增强。")
        else:
            print(f"  可用 (有标签) 的原始图片: {len(paired)}")
            ts = int(time.time())
            written = 0

            for i in range(need):
                src = random.choice(paired)
                img = cv2.imread(str(src), cv2.IMREAD_COLOR)
                if img is None:
                    print(f"  [WARN] 读取失败, 跳过: {src.name}")
                    continue

                aug = augment_image(img)

                # 新文件名: {原始stem}_daug_{时间戳}_{序号}.png / .txt
                new_stem = f"{src.stem}_daug_{ts}_{i:05d}"
                out_img = out_img_dir / f"{new_stem}.png"
                out_lab = out_lab_dir / f"{new_stem}.txt"

                cv2.imwrite(str(out_img), aug)
                src_lab = src_lab_dir / (src.stem + ".txt")
                shutil.copy2(str(src_lab), str(out_lab))
                written += 1

                if (i + 1) % 50 == 0 or (i + 1) == need:
                    print(f"  进度: {i + 1}/{need}")

            print(f"  成功写入增强图片: {written}")

    final_count = len(list_images(out_img_dir))
    print(f"\n[DONE] 输出目录最终图片总数: {final_count}")


def main() -> None:
    args = parse_args()
    random.seed(args.seed)
    np.random.seed(args.seed)

    dataset_root = Path(args.dataset_root)
    output_root = Path(args.output_root)

    if not dataset_root.exists():
        raise FileNotFoundError(f"找不到输入目录: {dataset_root}")

    print("=" * 60)
    print("检测数据集像素级增强")
    print(f"  输入: {dataset_root}")
    print(f"  输出: {output_root}")
    print("=" * 60)

    augment_dataset(
        dataset_root, output_root,
        target_count=args.target_count, dry_run=args.dry_run,
    )

    print("\n[INFO] 完成。")


if __name__ == "__main__":
    main()
