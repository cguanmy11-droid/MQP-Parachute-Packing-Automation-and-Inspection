import argparse
import json
import random
import shutil
from pathlib import Path

import numpy as np
import torch
import torchvision.transforms as T
from PIL import Image
from ultralytics import YOLO
from ultralytics.cfg import DEFAULT_CFG_DICT
from ultralytics.data.dataset import ClassificationDataset
from ultralytics.models.yolo.classify import ClassificationTrainer, ClassificationValidator


IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"}

# 模块级配置：存放无法通过 YOLO trainer 透传的自定义参数
_CUSTOM_ARGS: dict = {}


# ---------------------------------------------------------------------------
# Preprocessing transforms
# ---------------------------------------------------------------------------


class EnsureLandscape:
    """Rotate 90 deg when height > width (for rect mode)."""

    def __call__(self, img: Image.Image) -> Image.Image:
        w, h = img.size
        if h > w:
            return img.transpose(Image.ROTATE_90)
        return img


class LetterboxRect:
    """Keep aspect ratio and pad to fixed (H, W) rectangle."""

    def __init__(self, out_h: int, out_w: int, fill=(114, 114, 114)):
        self.out_h = int(out_h)
        self.out_w = int(out_w)
        self.fill = fill

    def __call__(self, img: Image.Image) -> Image.Image:
        if img.mode != "RGB":
            img = img.convert("RGB")
        w, h = img.size
        if w <= 0 or h <= 0:
            return img
        scale = min(self.out_w / w, self.out_h / h)
        nw = max(1, int(round(w * scale)))
        nh = max(1, int(round(h * scale)))
        if (nw, nh) != (w, h):
            img = img.resize((nw, nh), Image.BILINEAR)
        canvas = Image.new("RGB", (self.out_w, self.out_h), self.fill)
        left = (self.out_w - nw) // 2
        top = (self.out_h - nh) // 2
        canvas.paste(img, (left, top))
        return canvas


class LetterboxSquare:
    """Keep aspect ratio, pad to size x size (PIL). Pickle-safe for DataLoader workers."""

    def __init__(self, size: int, fill=(114, 114, 114)):
        self.size = int(size)
        self.fill = fill

    def __call__(self, img: Image.Image) -> Image.Image:
        if img.mode != "RGB":
            img = img.convert("RGB")
        w, h = img.size
        if w <= 0 or h <= 0:
            return img
        scale = min(self.size / w, self.size / h)
        nw = max(1, int(round(w * scale)))
        nh = max(1, int(round(h * scale)))
        if (nw, nh) != (w, h):
            img = img.resize((nw, nh), Image.BILINEAR)
        canvas = Image.new("RGB", (self.size, self.size), self.fill)
        left = (self.size - nw) // 2
        top = (self.size - nh) // 2
        canvas.paste(img, (left, top))
        return canvas


# ---------------------------------------------------------------------------
# Augmentation transforms
# ---------------------------------------------------------------------------


class RandomBGRSwap:
    """Randomly swap RGB->BGR with probability p."""

    def __init__(self, p: float = 0.0):
        self.p = float(max(0.0, min(1.0, p)))

    def __call__(self, img: Image.Image) -> Image.Image:
        if self.p <= 0 or random.random() >= self.p:
            return img
        arr = np.asarray(img)
        if arr.ndim == 3 and arr.shape[2] == 3:
            arr = arr[..., ::-1].copy()
            return Image.fromarray(arr)
        return img


class RandomGaussianNoise:
    """Additive Gaussian noise on RGB image."""

    def __init__(self, p: float = 0.0, std_range=(8.0, 28.0)):
        self.p = float(max(0.0, min(1.0, p)))
        self.std_min = float(std_range[0])
        self.std_max = float(std_range[1])

    def __call__(self, img: Image.Image) -> Image.Image:
        if self.p <= 0 or random.random() >= self.p:
            return img
        arr = np.asarray(img).astype(np.float32)
        std = random.uniform(self.std_min, self.std_max)
        noise = np.random.normal(0.0, std, size=arr.shape).astype(np.float32)
        arr = np.clip(arr + noise, 0, 255).astype(np.uint8)
        return Image.fromarray(arr)


class RandomSaltPepperNoise:
    """Randomly add salt-and-pepper impulse noise."""

    def __init__(self, p: float = 0.0, amount_range=(0.004, 0.02)):
        self.p = float(max(0.0, min(1.0, p)))
        self.amount_min = float(amount_range[0])
        self.amount_max = float(amount_range[1])

    def __call__(self, img: Image.Image) -> Image.Image:
        if self.p <= 0 or random.random() >= self.p:
            return img
        arr = np.asarray(img).copy()
        h, w = arr.shape[:2]
        amount = random.uniform(self.amount_min, self.amount_max)
        n = max(1, int(h * w * amount))
        ys = np.random.randint(0, h, n)
        xs = np.random.randint(0, w, n)
        half = n // 2
        arr[ys[:half], xs[:half]] = 255
        arr[ys[half:], xs[half:]] = 0
        return Image.fromarray(arr)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _arg_value(args, key: str, default):
    return getattr(args, key, default) if hasattr(args, key) else default


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
    x1 = int(np.floor(np.clip(np.min(pts[:, 0]), 0, w - 1)))
    y1 = int(np.floor(np.clip(np.min(pts[:, 1]), 0, h - 1)))
    # PIL.Image.crop 的右下角是开区间，x2/y2 应该允许取到 w/h，避免边界永远少 1px
    x2 = int(np.ceil(np.clip(np.max(pts[:, 0]), 1, w)))
    y2 = int(np.ceil(np.clip(np.max(pts[:, 1]), 1, h)))
    if x2 <= x1 or y2 <= y1:
        return None
    return x1, y1, x2, y2


# ---------------------------------------------------------------------------
# Dataset builder (Labelme -> cls_dataset)
# ---------------------------------------------------------------------------


def build_cls_dataset_from_labelme(
    labelme_dir: Path,
    out_dir: Path,
    val_ratio: float = 0.2,
    seed: int = 42,
    min_size: int = 8,
):
    if out_dir.exists():
        shutil.rmtree(out_dir)
    (out_dir / "train").mkdir(parents=True, exist_ok=True)
    (out_dir / "val").mkdir(parents=True, exist_ok=True)

    json_files = sorted(labelme_dir.glob("*.json"))
    if not json_files:
        raise FileNotFoundError(f"未找到标注文件: {labelme_dir}")

    all_samples = []  # (crop_path, class_name)
    class_counter: dict = {}

    for json_path in json_files:
        with open(json_path, "r", encoding="utf-8") as f:
            meta = json.load(f)

        img_path = _find_image_for_json(json_path, meta.get("imagePath"))
        if img_path is None or not img_path.exists():
            continue

        img = Image.open(img_path).convert("RGB")
        w, h = img.size
        shapes = meta.get("shapes", [])
        local_i = 0

        for shape in shapes:
            cls = str(shape.get("label", "")).strip()
            if not cls:
                continue
            bbox = _shape_to_bbox(shape, w, h)
            if bbox is None:
                continue
            x1, y1, x2, y2 = bbox
            if (x2 - x1) < min_size or (y2 - y1) < min_size:
                continue

            crop = img.crop((x1, y1, x2, y2))
            cls_safe = _safe_stem(cls)
            class_counter[cls_safe] = class_counter.get(cls_safe, 0) + 1
            name = f"{json_path.stem}_{local_i:03d}.png"
            tmp_dir = out_dir / "_all" / cls_safe
            tmp_dir.mkdir(parents=True, exist_ok=True)
            crop_path = tmp_dir / name
            crop.save(crop_path)
            all_samples.append((crop_path, cls_safe))
            local_i += 1

    if not all_samples:
        raise RuntimeError("没有从 Labelme 标注中解析出任何分类样本。")

    rng = random.Random(seed)
    by_class: dict = {}
    for p, c in all_samples:
        by_class.setdefault(c, []).append(p)

    for cls, paths in by_class.items():
        rng.shuffle(paths)
        n_val = max(1, int(len(paths) * val_ratio)) if len(paths) > 1 else 0
        val_set = set(paths[:n_val])

        for p in paths:
            split = "val" if p in val_set else "train"
            dst_dir = out_dir / split / cls
            dst_dir.mkdir(parents=True, exist_ok=True)
            shutil.copy2(p, dst_dir / p.name)

    shutil.rmtree(out_dir / "_all", ignore_errors=True)

    print("数据集构建完成:")
    for cls in sorted(by_class.keys()):
        train_n = len(list((out_dir / "train" / cls).glob("*")))
        val_n = len(list((out_dir / "val" / cls).glob("*")))
        print(f"  {cls}: train={train_n}, val={val_n}")


# ---------------------------------------------------------------------------
# Custom dataset / trainer / validator
# ---------------------------------------------------------------------------


class CustomizedDataset(ClassificationDataset):
    """
    分类数据集：支持正方形 letterbox 和矩形 letterbox 两种输入模式（通过 --use_rect 切换）。
    使用统一的强增强管线（blur / noise / color jitter 等）。
    """

    def __init__(self, root: str, args, augment: bool = False, prefix: str = ""):
        super().__init__(root, args, augment, prefix)
        self.augment = augment

        # 从模块级配置读取自定义参数（YOLO trainer 不会透传这些）
        use_rect = _CUSTOM_ARGS.get("use_rect", False)
        self.use_rect = use_rect

        if use_rect:
            self.out_h = int(_CUSTOM_ARGS.get("img_h", 256))
            self.out_w = int(_CUSTOM_ARGS.get("img_w", 512))
            self.ensure_landscape = bool(_CUSTOM_ARGS.get("ensure_landscape", True))
            self.letterbox = LetterboxRect(self.out_h, self.out_w)
        else:
            self.imgsz = int(_arg_value(args, "imgsz", 512))
            self.letterbox = LetterboxSquare(self.imgsz)

        # Augmentation config
        self.aug_cfg = {
            "hsv_h": float(_arg_value(args, "hsv_h", 0.015)),
            "hsv_s": float(_arg_value(args, "hsv_s", 0.7)),
            "hsv_v": float(_arg_value(args, "hsv_v", 0.4)),
            "degrees": float(_arg_value(args, "degrees", 0.0)),
            "translate": float(_arg_value(args, "translate", 0.0)),
            "scale": float(_arg_value(args, "scale", 0.0)),
            "shear": float(_arg_value(args, "shear", 0.0)),
            "fliplr": float(_arg_value(args, "fliplr", 0.5)),
            "flipud": float(_arg_value(args, "flipud", 0.0)),
            "bgr": float(_arg_value(args, "bgr", 0.0)),
            "auto_augment": _arg_value(args, "auto_augment", None),
            "erasing": float(_arg_value(args, "erasing", 0.0)),
            # Blur / noise degradation
            "blur_p": float(_CUSTOM_ARGS.get("blur_p", 0.55)),
            "noise_p": float(_CUSTOM_ARGS.get("noise_p", 0.45)),
            "sp_noise_p": float(_CUSTOM_ARGS.get("sp_noise_p", 0.35)),
        }

        if self.augment and str(prefix).lower().startswith("train"):
            mode_str = (
                f"rect {self.out_h}x{self.out_w}" if use_rect else f"square {self.imgsz}x{self.imgsz}"
            )
            print(f"[AUG] train input: {mode_str}")
            print(f"[AUG] train transforms cfg: {self.aug_cfg}")

        # ---- Build train transforms ----
        train_ops: list = []
        if use_rect and self.ensure_landscape:
            train_ops.append(EnsureLandscape())
        train_ops.append(self.letterbox)

        # Geometry augmentation
        if any(self.aug_cfg[k] > 0 for k in ("degrees", "translate", "scale", "shear")):
            scale = self.aug_cfg["scale"]
            train_ops.append(
                T.RandomAffine(
                    degrees=self.aug_cfg["degrees"],
                    translate=(self.aug_cfg["translate"], self.aug_cfg["translate"]),
                    scale=(max(0.1, 1.0 - scale), 1.0 + scale) if scale > 0 else None,
                    shear=self.aug_cfg["shear"] if self.aug_cfg["shear"] > 0 else None,
                    interpolation=T.InterpolationMode.BILINEAR,
                    fill=114,
                )
            )

        # Classification auto augmentation (optional)
        aa = self.aug_cfg["auto_augment"]
        if aa == "randaugment":
            train_ops.append(T.RandAugment(interpolation=T.InterpolationMode.BILINEAR))
        elif aa == "autoaugment":
            train_ops.append(T.AutoAugment(interpolation=T.InterpolationMode.BILINEAR))
        elif aa == "augmix":
            train_ops.append(T.AugMix(interpolation=T.InterpolationMode.BILINEAR))

        # Color augmentation
        train_ops.append(
            T.ColorJitter(
                brightness=self.aug_cfg["hsv_v"],
                contrast=self.aug_cfg["hsv_v"],
                saturation=self.aug_cfg["hsv_s"],
                hue=self.aug_cfg["hsv_h"],
            )
        )

        # Blur + noise: simulate defocus / motion degradation and sensor noise
        if self.aug_cfg["blur_p"] > 0:
            train_ops.append(
                T.RandomApply(
                    [T.GaussianBlur(kernel_size=7, sigma=(0.5, 4.0))],
                    p=self.aug_cfg["blur_p"],
                )
            )
        if self.aug_cfg["noise_p"] > 0:
            train_ops.append(RandomGaussianNoise(p=self.aug_cfg["noise_p"], std_range=(8.0, 28.0)))
        if self.aug_cfg["sp_noise_p"] > 0:
            train_ops.append(RandomSaltPepperNoise(p=self.aug_cfg["sp_noise_p"], amount_range=(0.004, 0.02)))

        train_ops.append(T.RandomHorizontalFlip(p=self.aug_cfg["fliplr"]))
        train_ops.append(T.RandomVerticalFlip(p=self.aug_cfg["flipud"]))
        train_ops.append(RandomBGRSwap(p=self.aug_cfg["bgr"]))
        train_ops.append(T.ToTensor())
        train_ops.append(T.Normalize(mean=(0.0, 0.0, 0.0), std=(1.0, 1.0, 1.0)))
        if self.aug_cfg["erasing"] > 0:
            train_ops.append(T.RandomErasing(p=self.aug_cfg["erasing"], inplace=True))

        self.train_transforms = T.Compose(train_ops)

        # ---- Build val transforms ----
        val_ops: list = []
        if use_rect and self.ensure_landscape:
            val_ops.append(EnsureLandscape())
        val_ops.append(self.letterbox)
        val_ops.append(T.ToTensor())
        val_ops.append(T.Normalize(mean=(0.0, 0.0, 0.0), std=(1.0, 1.0, 1.0)))
        self.val_transforms = T.Compose(val_ops)

    def __getitem__(self, index: int):
        sample = self.samples[index]
        path = sample[0]
        label = int(sample[1])
        img = Image.open(path).convert("RGB")

        if self.augment:
            img = self.train_transforms(img)
        else:
            img = self.val_transforms(img)

        return {"img": img, "cls": label}

    # 使用固定尺寸后，默认 collate 即可 stack，无需自定义 collate_fn。


class CustomizedTrainer(ClassificationTrainer):
    def build_dataset(self, img_path: str, mode: str = "train", batch=None):
        return CustomizedDataset(root=img_path, args=self.args, augment=mode == "train", prefix=mode)


class CustomizedValidator(ClassificationValidator):
    def build_dataset(self, img_path: str, mode: str = "train"):
        return CustomizedDataset(root=img_path, args=self.args, augment=False, prefix=self.args.split)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def parse_args():
    parser = argparse.ArgumentParser(
        description="YOLO26m-cls 分类训练脚本（含 Labelme 转换 + 强增强，支持正方形/矩形 letterbox）"
    )
    parser.add_argument("--labelme_dir", type=str, default="Cropped_img_cls", help="Labelme 数据目录（json + image）")
    parser.add_argument("--dataset_dir", type=str, default="cls_dataset", help="输出的分类数据集根目录")
    parser.add_argument("--rebuild_dataset", action="store_true", help="强制重建分类数据集")
    parser.add_argument("--weights", type=str, default="yolo26m-cls.pt", help="初始权重")
    parser.add_argument("--epochs", type=int, default=50)

    # ---- 输入模式（默认矩形 letterbox）----
    parser.add_argument("--img_h", type=int, default=256, help="矩形输入高")
    parser.add_argument("--img_w", type=int, default=512, help="矩形输入宽")
    parser.add_argument("--ensure_landscape", action="store_true", default=True,
                        help="竖图旋转 90 度统一为横图")

    # ---- 增强概率 ----
    parser.add_argument("--blur_p", type=float, default=0.25, help="GaussianBlur 触发概率")
    parser.add_argument("--noise_p", type=float, default=0.25, help="高斯噪声触发概率")
    parser.add_argument("--sp_noise_p", type=float, default=0.25, help="椒盐噪点触发概率")

    parser.add_argument("--batch", type=int, default=32)
    parser.add_argument("--workers", type=int, default=0,
                        help="DataLoader workers；Windows 建议 0 避免共享映射")
    parser.add_argument("--val_ratio", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--device", type=str, default="0", help="如 0 / 0,1 / cpu")
    parser.add_argument("--project", type=str, default="runs/classify")
    parser.add_argument("--name", type=str, default="yolo26m_cls_custom_aug")
    return parser.parse_args()


def main():
    args = parse_args()
    root = Path(__file__).resolve().parent
    labelme_dir = (root / args.labelme_dir).resolve()
    dataset_dir = (root / args.dataset_dir).resolve()

    # ---- 将自定义参数写入模块级配置，供 CustomizedDataset 读取 ----
    _CUSTOM_ARGS.update({
        "use_rect": True,
        "img_h": args.img_h,
        "img_w": args.img_w,
        "ensure_landscape": args.ensure_landscape,
        "blur_p": args.blur_p,
        "noise_p": args.noise_p,
        "sp_noise_p": args.sp_noise_p,
    })

    # ---- 构建 / 复用数据集 ----
    need_build = args.rebuild_dataset or not (dataset_dir / "train").exists() or not (dataset_dir / "val").exists()
    if need_build:
        print(f"[INFO] 构建分类数据集: {labelme_dir} -> {dataset_dir}")
        build_cls_dataset_from_labelme(
            labelme_dir=labelme_dir,
            out_dir=dataset_dir,
            val_ratio=args.val_ratio,
            seed=args.seed,
        )
    else:
        print(f"[INFO] 使用现有分类数据集: {dataset_dir}")

    model = YOLO(args.weights)

    # 计算 effective imgsz（YOLO trainer 接口只接受单一 int）
    effective_imgsz = max(args.img_h, args.img_w)

    # 严格 no-crop 模式：不做几何裁切增强
    aug_args = {
        "hsv_h": 0.015,
        "hsv_s": 0.7,
        "hsv_v": 0.4,
        "degrees": 0.0,
        "translate": 0.0,
        "scale": 0.0,
        "shear": 0.0,
        "perspective": 0.0,
        "flipud": 0.0,
        "fliplr": 0.5,
        "bgr": 0.0,
        "mosaic": 0.0,
        "mixup": 0.0,
        "cutmix": 0.0,
        "close_mosaic": 10,
        "copy_paste": 0.0,
        "copy_paste_mode": "flip",
        "auto_augment": None,
        "erasing": 0.0,
        "augmentations": None,
    }
    supported_aug_args = {k: v for k, v in aug_args.items() if k in DEFAULT_CFG_DICT}
    skipped_aug_args = sorted(set(aug_args.keys()) - set(supported_aug_args.keys()))
    if skipped_aug_args:
        print(f"[WARN] 当前版本不支持以下增强参数，已自动跳过: {skipped_aug_args}")

    print(f"[INFO] no-crop resize 已启用（rect {args.img_h}x{args.img_w}）：输入仅做 letterbox（等比例缩放+padding），不做几何裁切增强。")

    # 注意：分类增强主行为仍由 CustomizedDataset 的 transforms 控制，
    # 这里主要统一透传基础开关（如 fliplr/flipud/auto_augment/erasing 等）。
    model.train(
        data=str(dataset_dir),
        trainer=CustomizedTrainer,
        epochs=args.epochs,
        imgsz=effective_imgsz,
        batch=args.batch,
        workers=args.workers,
        device=args.device,
        project=args.project,
        name=args.name,
        seed=args.seed,
        pretrained=True,
        amp=True,
        **supported_aug_args,
    )

    metrics = model.val(
        data=str(dataset_dir),
        validator=CustomizedValidator,
        imgsz=effective_imgsz,
        batch=args.batch,
        workers=args.workers,
        device=args.device,
    )
    print(f"[VAL] top1={metrics.top1:.4f}, top5={metrics.top5:.4f}")


if __name__ == "__main__":
    main()
