"""
基于 Ultralytics YOLO26 的端到端训练脚本

功能:
1) 读取 Raw+Fine_combine-formatted-aug/ 下已有的 YOLO 格式数据集 (images/ + labels/)
2) 划分 Train/Val 集合并生成 data.yaml
3) 使用 YOLO26m (yolo26m.pt) 进行训练与验证, 启用增强与完整可视化

使用方法:
    python train_yolov26.py                    # 从头训练
    python train_yolov26.py --resume           # 从默认 checkpoint 继续训练
    python train_yolov26.py --resume path.pt   # 从指定 checkpoint 继续训练

依赖安装:
    pip install ultralytics>=8.4.0
"""
import argparse
import os
import random
import shutil
from pathlib import Path
from typing import List, Tuple

# ────────────────────────── 配置 ──────────────────────────
RAW_DIR = Path('Raw+Fine_combine-formatted-aug+')
OUT_DIR = Path('dataset_yolo26')
IMAGES_DIR = OUT_DIR / 'images'
LABELS_DIR = OUT_DIR / 'labels'

# 类别名称 (与 label 文件中的 class id 顺序一致)
CLASS_NAMES = ['hole']

# 划分比例与随机种子
VAL_RATIO = 0.2
RANDOM_SEED = 42

# 默认 resume checkpoint 路径
DEFAULT_RESUME_CKPT = Path('runs/detect/runs/yolo26m_hole2/weights/last.pt')


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLO26m 训练脚本")
    parser.add_argument(
        '--resume', nargs='?', const=str(DEFAULT_RESUME_CKPT), default=None,
        help='从 checkpoint 继续训练。不带路径则使用默认 last.pt, 也可指定路径',
    )
    parser.add_argument(
        '--epochs', type=int, default=200,
        help='训练总 epoch 数 (默认 85)',
    )
    return parser.parse_args()

# ────────────────────────── 工具函数 ──────────────────────────

def ensure_dirs():
    """创建输出目录结构"""
    for split in ('train', 'val'):
        (IMAGES_DIR / split).mkdir(parents=True, exist_ok=True)
        (LABELS_DIR / split).mkdir(parents=True, exist_ok=True)


def list_image_label_pairs(raw_dir: Path) -> List[Tuple[Path, Path]]:
    """
    从已有 YOLO 格式数据集中收集 (image, label) 对。
    images/ 下的 .png 文件与 labels/ 下同名 .txt 文件配对。
    """
    img_dir = raw_dir / 'images'
    lab_dir = raw_dir / 'labels'
    pairs: List[Tuple[Path, Path]] = []
    for img_path in sorted(img_dir.glob('*.png')):
        lab_path = lab_dir / (img_path.stem + '.txt')
        if lab_path.exists():
            pairs.append((img_path, lab_path))
        else:
            print(f'[警告] 缺少标签文件, 跳过: {img_path.name}')
    return pairs


def split_train_val(
    pairs: List[Tuple[Path, Path]], val_ratio: float, seed: int
) -> Tuple[List[Tuple[Path, Path]], List[Tuple[Path, Path]]]:
    """随机划分 train / val"""
    pairs_copy = list(pairs)
    random.Random(seed).shuffle(pairs_copy)
    n_val = int(len(pairs_copy) * val_ratio)
    return pairs_copy[n_val:], pairs_copy[:n_val]


def copy_split(pairs: List[Tuple[Path, Path]], split: str) -> int:
    """将 image + label 复制到对应的 train/val 子目录, 返回数量"""
    img_out = IMAGES_DIR / split
    lab_out = LABELS_DIR / split
    for img_path, lab_path in pairs:
        shutil.copy2(img_path, img_out / img_path.name)
        shutil.copy2(lab_path, lab_out / lab_path.name)
    return len(pairs)


def write_yaml(names: List[str], out_dir: Path) -> Path:
    """生成 data.yaml"""
    yaml_path = out_dir / 'data.yaml'
    train_path = (IMAGES_DIR / 'train').resolve().as_posix()
    val_path = (IMAGES_DIR / 'val').resolve().as_posix()

    lines = [
        f"path: {out_dir.resolve().as_posix()}",
        f"train: {train_path}",
        f"val: {val_path}",
        f"nc: {len(names)}",
        "names:",
    ]
    for i, n in enumerate(names):
        lines.append(f"  {i}: {n}")

    yaml_path.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return yaml_path


# ────────────────────────── 数据准备 ──────────────────────────

def prepare_dataset() -> Path:
    print('=' * 60)
    print('准备数据集: 划分 Train/Val, 生成 data.yaml')
    print('=' * 60)

    ensure_dirs()
    pairs = list_image_label_pairs(RAW_DIR)
    if not pairs:
        raise RuntimeError(f'未在 {RAW_DIR} 找到成对的 image/label 文件')

    train_pairs, val_pairs = split_train_val(pairs, VAL_RATIO, RANDOM_SEED)
    n_train = copy_split(train_pairs, 'train')
    n_val = copy_split(val_pairs, 'val')
    yaml_path = write_yaml(CLASS_NAMES, OUT_DIR)

    print(f'- 类别数: {len(CLASS_NAMES)} -> {CLASS_NAMES}')
    print(f'- 训练集: {n_train} 张, 验证集: {n_val} 张')
    print(f'- YAML: {yaml_path}')
    return yaml_path


# ────────────────────────── 训练 & 验证 ──────────────────────────

def train_and_validate(data_yaml: Path, resume_ckpt: str = None, epochs: int = 85):
    print('\n' + '=' * 60)
    if resume_ckpt:
        print(f'从 checkpoint 继续训练: {resume_ckpt}')
    else:
        print('开始训练 YOLO26m (含增强 & 完整可视化)...')
    print('=' * 60)

    try:
        from ultralytics import YOLO
        import torch
    except ImportError as e:
        raise RuntimeError(
            '未安装 ultralytics, 请先运行: pip install ultralytics>=8.4.0'
        ) from e

    device = '0' if torch.cuda.is_available() else 'cpu'
    print(f'使用设备: {"GPU (cuda:0)" if device == "0" else "CPU"}')

    # 统一的训练超参数 (从头训练和 resume 共用)
    train_kwargs = dict(
        data=str(data_yaml),
        epochs=epochs,
        imgsz=640,
        batch=4,
        workers=0,
        device=device,
        project='runs',
        name='yolo26m_hole',
        pretrained=True,
        save=True,
        plots=True,
        val=True,
        # 数据增强
        mosaic=1.0,
        mixup=0.1,
        degrees=10.0,
        translate=0.10,
        scale=0.50,
        shear=2.0,
        perspective=0.0,
        flipud=0.0,
        fliplr=0.5,
        hsv_h=0.005,
        hsv_s=0.3,
        hsv_v=0.1,
        close_mosaic=10,
    )

    if resume_ckpt:
        # ── 从已有 checkpoint 继续训练 ──
        ckpt_path = Path(resume_ckpt)
        if not ckpt_path.exists():
            raise FileNotFoundError(f'Checkpoint 不存在: {ckpt_path}')

        # 检查 checkpoint 是否已经训练完成
        import torch as _torch
        ckpt_data = _torch.load(str(ckpt_path), map_location='cpu')
        done_epochs = ckpt_data.get('epoch', -1) + 1  # 已完成的 epoch 数
        total_epochs = ckpt_data.get('train_args', {}).get('epochs', 0)

        model = YOLO(str(ckpt_path))

        if done_epochs < total_epochs:
            # 训练中途中断 → resume 继续
            print(f'[RESUME] 训练中断于 {done_epochs}/{total_epochs}, 继续训练到 {epochs} epochs')
            train_kwargs['resume'] = True
        else:
            # 训练已完成 → 加载权重开启新一轮训练
            print(f'[FINE-TUNE] 原训练已完成 ({done_epochs}/{total_epochs} epochs)')
            print(f'  加载权重, 开启新一轮训练 {epochs} epochs')
            # 不加 resume=True, 以已有权重为起点从 epoch 0 开始新训练
    else:
        # ── 从头训练 ──
        model = YOLO('yolo26m.pt')

    results = model.train(**train_kwargs)

    # 独立验证 (生成混淆矩阵 / PR 曲线等可视化)
    val_results = model.val(
        data=str(data_yaml),
        imgsz=640,
        batch=4,
        workers=0,
        device=device,
        plots=True,
        save_json=False,
        project='runs',
        name='yolo26m_hole_val',
    )

    print('\n' + '=' * 60)
    print('训练与验证完成。')
    print('=' * 60)


# ────────────────────────── 入口 ──────────────────────────

def main():
    args = parse_args()
    yaml_path = prepare_dataset()
    train_and_validate(yaml_path, resume_ckpt=args.resume, epochs=args.epochs)


if __name__ == '__main__':
    main()
