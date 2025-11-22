"""
基于 Ultralytics YOLOv8 的端到端训练脚本

功能:
1) 解析 raw_dataset-formatted/ 下的 LabelMe JSON 标注
2) 转换为 YOLO 检测格式 (labels/*.txt)
3) 划分 Train/Val 集合并生成 data.yaml
4) 使用 YOLOv8 训练与验证, 启用增强与完整可视化 (plots)

使用方法:
    python train_yolov8.py

依赖安装:
    pip install -r requirements.txt
"""
import os
import json
import random
import shutil
from pathlib import Path
from typing import Dict, List, Tuple

from PIL import Image


RAW_DIR = Path('raw_dataset-formatted')
OUT_DIR = Path('dataset_yolo')
IMAGES_DIR = OUT_DIR / 'images'
LABELS_DIR = OUT_DIR / 'labels'

# 划分比例与随机种子
VAL_RATIO = 0.2
RANDOM_SEED = 42


def ensure_dirs():
    (IMAGES_DIR / 'train').mkdir(parents=True, exist_ok=True)
    (IMAGES_DIR / 'val').mkdir(parents=True, exist_ok=True)
    (LABELS_DIR / 'train').mkdir(parents=True, exist_ok=True)
    (LABELS_DIR / 'val').mkdir(parents=True, exist_ok=True)


def list_pairs(raw_dir: Path) -> List[Tuple[Path, Path]]:
    """列出 (image_path, json_path) 对。如果对应 JSON 缺失则跳过。"""
    pairs: List[Tuple[Path, Path]] = []
    for img_path in sorted(raw_dir.glob('*.png')):
        json_path = img_path.with_suffix('.json')
        if json_path.exists():
            pairs.append((img_path, json_path))
    return pairs


def collect_class_names(json_paths: List[Path]) -> List[str]:
    """扫描所有 JSON, 收集 class 名称并排序, 返回有序列表。"""
    names_set = set()
    for jp in json_paths:
        try:
            with jp.open('r', encoding='utf-8') as f:
                data = json.load(f)
            for shp in data.get('shapes', []):
                label = str(shp.get('label', '')).strip()
                if label:
                    names_set.add(label)
        except Exception:
            # 忽略异常的标注文件
            continue
    names = sorted(list(names_set))
    return names


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def points_to_bbox(points: List[List[float]]) -> Tuple[float, float, float, float]:
    """将任意点集(如矩形/多边形)转换为包围盒 (x1,y1,x2,y2)。"""
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return min(xs), min(ys), max(xs), max(ys)


def xyxy_to_yolo(x1: float, y1: float, x2: float, y2: float, w: int, h: int) -> Tuple[float, float, float, float]:
    """将像素坐标的 (x1,y1,x2,y2) 转换为 YOLO 归一化 (xc,yc,bw,bh)。"""
    x1 = clamp(x1, 0, w - 1)
    y1 = clamp(y1, 0, h - 1)
    x2 = clamp(x2, 0, w - 1)
    y2 = clamp(y2, 0, h - 1)
    bw = max(0.0, x2 - x1)
    bh = max(0.0, y2 - y1)
    if bw <= 0 or bh <= 0:
        return 0.0, 0.0, 0.0, 0.0
    xc = x1 + bw / 2.0
    yc = y1 + bh / 2.0
    return xc / w, yc / h, bw / w, bh / h


def convert_one_labelme(json_path: Path, label_to_id: Dict[str, int]) -> List[str]:
    """将单个 LabelMe JSON 转为 YOLO 标签行列表: ['cls xc yc w h', ...]"""
    with json_path.open('r', encoding='utf-8') as f:
        data = json.load(f)

    # 优先从 JSON 读取图像尺寸, 缺失则用 PIL 读取对应 PNG
    img_w = data.get('imageWidth')
    img_h = data.get('imageHeight')
    if not img_w or not img_h:
        img_path = json_path.with_suffix('.png')
        with Image.open(img_path) as im:
            img_w, img_h = im.size

    lines: List[str] = []
    for shp in data.get('shapes', []):
        label = str(shp.get('label', '')).strip()
        pts = shp.get('points', [])
        if not label or not pts:
            continue
        if label not in label_to_id:
            # 未知类别跳过
            continue
        x1, y1, x2, y2 = points_to_bbox(pts)
        xc, yc, bw, bh = xyxy_to_yolo(x1, y1, x2, y2, img_w, img_h)
        if bw == 0.0 or bh == 0.0:
            continue
        cls_id = label_to_id[label]
        lines.append(f"{cls_id} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}")
    return lines


def split_train_val(pairs: List[Tuple[Path, Path]], val_ratio: float, seed: int) -> Tuple[List[Tuple[Path, Path]], List[Tuple[Path, Path]]]:
    random.Random(seed).shuffle(pairs)
    n = len(pairs)
    n_val = int(n * val_ratio)
    val_pairs = pairs[:n_val]
    train_pairs = pairs[n_val:]
    return train_pairs, val_pairs


def copy_and_write_labels(pairs: List[Tuple[Path, Path]], split: str, label_to_id: Dict[str, int]) -> int:
    """复制图片到 images/split, 生成 labels/split 下的 .txt。返回写入图片数量。"""
    img_out_dir = IMAGES_DIR / split
    lab_out_dir = LABELS_DIR / split
    count = 0
    for img_path, json_path in pairs:
        # 拷贝图片
        dst_img = img_out_dir / img_path.name
        shutil.copy2(img_path, dst_img)

        # 写入标签
        yolo_lines = convert_one_labelme(json_path, label_to_id)
        dst_lab = lab_out_dir / (img_path.stem + '.txt')
        with dst_lab.open('w', encoding='utf-8') as f:
            f.write('\n'.join(yolo_lines))
        count += 1
    return count


def write_yaml(names: List[str], out_dir: Path) -> Path:
    """生成 data.yaml 并返回路径。"""
    yaml_path = out_dir / 'data.yaml'
    # 使用 POSIX 风格路径, 兼容 Windows
    train_path = (IMAGES_DIR / 'train').resolve().as_posix()
    val_path = (IMAGES_DIR / 'val').resolve().as_posix()

    yaml_text = [
        f"path: {out_dir.resolve().as_posix()}",
        f"train: {train_path}",
        f"val: {val_path}",
        f"nc: {len(names)}",
        "names:",
    ]
    for i, n in enumerate(names):
        yaml_text.append(f"  {i}: {n}")

    with yaml_path.open('w', encoding='utf-8') as f:
        f.write('\n'.join(yaml_text) + '\n')
    return yaml_path


def prepare_dataset():
    print('准备数据集: 转换 LabelMe -> YOLO, 划分 Train/Val, 生成 data.yaml')
    ensure_dirs()
    pairs = list_pairs(RAW_DIR)
    if not pairs:
        raise RuntimeError(f'未在 {RAW_DIR} 找到成对的 PNG/JSON 文件')

    json_paths = [p[1] for p in pairs]
    names = collect_class_names(json_paths)
    if not names:
        raise RuntimeError('未在标注中发现任何类别名称')
    label_to_id = {n: i for i, n in enumerate(names)}

    train_pairs, val_pairs = split_train_val(pairs, VAL_RATIO, RANDOM_SEED)
    n_train = copy_and_write_labels(train_pairs, 'train', label_to_id)
    n_val = copy_and_write_labels(val_pairs, 'val', label_to_id)
    yaml_path = write_yaml(names, OUT_DIR)

    print(f'- 类别数: {len(names)} -> {names}')
    print(f'- 训练集: {n_train} 张, 验证集: {n_val} 张')
    print(f'- YAML: {yaml_path}')
    return yaml_path


def train_and_validate(data_yaml: Path):
    print('开始训练 YOLOv8 (含增强 & 完整可视化)...')
    try:
        from ultralytics import YOLO
        import torch
    except Exception as e:
        raise RuntimeError('未安装 ultralytics, 请先运行: pip install -r requirements.txt') from e

    # 自动检测设备
    device = '0' if torch.cuda.is_available() else 'cpu'
    print(f'使用设备: {device}')
    
    # 选择轻量模型以便快速验证流程, 可改为 yolov8s/ m / l 等
    model = YOLO('yolov8n.pt')

    # 训练: 启用 plots 以保存损失曲线/学习率曲线/指标曲线等
    results = model.train(
        data=str(data_yaml),
        epochs=50,
        imgsz=640,
        batch=4,
        workers=0,          # Windows 下更稳妥
        device=device,
        project='runs',
        name='yolov8_labelme',
        pretrained=True,
        save=True,
        plots=True,
        val=True,
        # 数据增强相关(训练阶段)
        mosaic=1.0,
        mixup=0.1,
        degrees=10.0,
        translate=0.10,
        scale=0.50,
        shear=2.0,
        perspective=0.0,
        flipud=0.0,
        fliplr=0.5,
        hsv_h=0.015,
        hsv_s=0.7,
        hsv_v=0.4,
        close_mosaic=10,
    )

    # 额外一次独立验证以生成所有可视化 (包括混淆矩阵/PR 曲线等)
    val_results = model.val(
        data=str(data_yaml),
        imgsz=640,
        batch=4,
        workers=0,
        device=device,
        plots=True,
        augment=True,  # 验证时的 TTA/增强
        save_json=False,
        project='runs',
        name='yolov8_labelme_val',
    )

    print('训练与验证完成。结果目录:')
    print(' - 训练: runs/detect/yolov8_labelme')
    print(' - 验证: runs/detect/yolov8_labelme_val')


def main():
    yaml_path = prepare_dataset()
    train_and_validate(yaml_path)


if __name__ == '__main__':
    main()


