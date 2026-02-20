"""
将 raw_dataset-formatted 中的 PNG+LabelMe JSON:
1) 转换为: 原始图像 + YOLO TXT 标注 (不划分 Train/Val)
2) 生成: 近似 train_yolov8.py(232-244) 的离线数据增强版本 + 对应 YOLO TXT

说明:
- 训练脚本里的 mosaic/mixup 属于多图/混合的在线增强，这里提供单图增强近似: 旋转(degrees)、平移(translate)、缩放(scale)、剪切(shear)、水平翻转(fliplr)、颜色HSV。
- 如确需离线导出 mosaic/mixup，我可以在此脚本基础上继续扩展。

使用方法:
    python prepare_yolo_with_aug.py


"""
from pathlib import Path
from typing import Dict, List, Tuple
import json
import random
import shutil

import numpy as np
import cv2


# 输入与输出目录
RAW_DIR = Path('Raw+Fine_combine-formatted')
OUT_ORIG = Path('Raw+Fine_combine-formatted-aug')
# 合并输出到同一文件夹
OUT_AUG = OUT_ORIG

# 增强参数(与 train_yolov8.py(232-244) 近似)
DEGREES = 15.0          # 旋转角度范围 ±DEGREES
TRANSLATE = 0.10        # 平移比例范围 ±TRANSLATE
SCALE_GAIN = 0.50       # 缩放幅度: [1-SCALE_GAIN, 1+SCALE_GAIN]
SHEAR = 2.0             # 剪切角度范围 ±SHEAR
FLIP_LR_P = 0.5         # 水平翻转概率(对应 fliplr=0.5)
HSV_H = 0.015           # 与 YOLO 类似: 约等于 ±(0.015*360°) ≈ ±5.4°
HSV_S = 0.7             # 饱和度缩放范围 [1-0.7, 1+0.7]
HSV_V = 0.5             # 亮度缩放范围 [1-0.4, 1+0.4]

# 每张图生成多少张增强图
AUG_PER_IMAGE = 2

# 追加离线 Mosaic & CutMix 设置
NUM_EXTRA = 85          # 额外生成的样本数量上限
MOSAIC_PROB = 0.5       # 生成时使用 Mosaic 的概率(否则 CutMix)
MOSAIC_SIZE = 640       # 离线 Mosaic 输出尺寸
AREA_FRAC_MIN = 0.1     # 保留框的最小面积占比阈值


def ensure_dirs(root: Path) -> None:
    (root / 'images').mkdir(parents=True, exist_ok=True)
    (root / 'labels').mkdir(parents=True, exist_ok=True)


def list_pairs(raw_dir: Path) -> List[Tuple[Path, Path]]:
    pairs: List[Tuple[Path, Path]] = []
    for img_path in sorted(raw_dir.glob('*.png')):
        json_path = img_path.with_suffix('.json')
        if json_path.exists():
            pairs.append((img_path, json_path))
    return pairs


def collect_class_names(json_paths: List[Path]) -> List[str]:
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
            continue
    return sorted(list(names_set))


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def points_to_bbox(points: List[List[float]]) -> Tuple[float, float, float, float]:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return min(xs), min(ys), max(xs), max(ys)


def xyxy_to_yolo(x1: float, y1: float, x2: float, y2: float, w: int, h: int) -> Tuple[float, float, float, float]:
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


def parse_labelme_boxes(json_path: Path, label_to_id: Dict[str, int]) -> Tuple[List[Tuple[int, float, float, float, float]], int, int]:
    """读取 LabelMe JSON, 返回: [(cls, x1, y1, x2, y2), ...], img_w, img_h"""
    with json_path.open('r', encoding='utf-8') as f:
        data = json.load(f)

    img_w = data.get('imageWidth')
    img_h = data.get('imageHeight')
    # 若 JSON 中缺失尺寸, 通过图片读取获得
    if not img_w or not img_h:
        img_path = json_path.with_suffix('.png')
        im = cv2.imread(str(img_path))
        if im is None:
            raise RuntimeError(f'无法读取图像: {img_path}')
        img_h, img_w = im.shape[:2]

    boxes: List[Tuple[int, float, float, float, float]] = []
    for shp in data.get('shapes', []):
        label = str(shp.get('label', '')).strip()
        pts = shp.get('points', [])
        if not label or not pts:
            continue
        if label not in label_to_id:
            continue
        x1, y1, x2, y2 = points_to_bbox(pts)
        boxes.append((label_to_id[label], x1, y1, x2, y2))
    return boxes, int(img_w), int(img_h)


def save_yolo_labels(label_path: Path, boxes_xyxy: List[Tuple[int, float, float, float, float]], w: int, h: int) -> None:
    lines: List[str] = []
    for cls_id, x1, y1, x2, y2 in boxes_xyxy:
        xc, yc, bw, bh = xyxy_to_yolo(x1, y1, x2, y2, w, h)
        if bw == 0.0 or bh == 0.0:
            continue
        lines.append(f"{cls_id} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}")
    label_path.write_text('\n'.join(lines), encoding='utf-8')


def yolo_to_xyxy(line: str, w: int, h: int) -> Tuple[int, float, float, float, float]:
    parts = line.strip().split()
    if len(parts) != 5:
        raise ValueError('YOLO 标签格式错误')
    cls_id = int(float(parts[0]))
    xc = float(parts[1]) * w
    yc = float(parts[2]) * h
    bw = float(parts[3]) * w
    bh = float(parts[4]) * h
    x1 = xc - bw / 2.0
    y1 = yc - bh / 2.0
    x2 = xc + bw / 2.0
    y2 = yc + bh / 2.0
    return cls_id, x1, y1, x2, y2


def load_yolo_boxes(label_path: Path, w: int, h: int) -> List[Tuple[int, float, float, float, float]]:
    if not label_path.exists():
        return []
    text = label_path.read_text(encoding='utf-8').strip()
    if not text:
        return []
    boxes: List[Tuple[int, float, float, float, float]] = []
    for line in text.splitlines():
        try:
            boxes.append(yolo_to_xyxy(line, w, h))
        except Exception:
            continue
    return boxes


def box_area(x1: float, y1: float, x2: float, y2: float) -> float:
    return max(0.0, x2 - x1) * max(0.0, y2 - y1)


def clip_box(x1: float, y1: float, x2: float, y2: float, w: int, h: int) -> Tuple[float, float, float, float]:
    nx1 = float(np.clip(x1, 0, w - 1))
    ny1 = float(np.clip(y1, 0, h - 1))
    nx2 = float(np.clip(x2, 0, w - 1))
    ny2 = float(np.clip(y2, 0, h - 1))
    return nx1, ny1, nx2, ny2


def rect_intersection(ax1: float, ay1: float, ax2: float, ay2: float,
                      bx1: float, by1: float, bx2: float, by2: float) -> Tuple[float, float, float, float]:
    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    if ix2 <= ix1 or iy2 <= iy1:
        return 0.0, 0.0, 0.0, 0.0
    return ix1, iy1, ix2, iy2


def list_all_images_labels(root: Path) -> List[Tuple[Path, Path]]:
    img_dir = root / 'images'
    lab_dir = root / 'labels'
    result: List[Tuple[Path, Path]] = []
    exts = ['*.png', '*.jpg', '*.jpeg', '*.bmp']
    img_paths: List[Path] = []
    for e in exts:
        img_paths.extend(sorted(img_dir.glob(e)))
    for ip in img_paths:
        lp = lab_dir / (ip.stem + '.txt')
        result.append((ip, lp))
    return result


def create_mosaic_sample(items: List[Tuple[Path, Path]], size: int) -> Tuple[np.ndarray, List[Tuple[int, float, float, float, float]]]:
    # 采样4张图, 每张缩放到 size//2 并拼接为2x2
    quad = random.sample(items, 4)
    tile_w = size // 2
    tile_h = size // 2
    canvas = np.zeros((size, size, 3), dtype=np.uint8)
    all_boxes: List[Tuple[int, float, float, float, float]] = []
    offsets = [(0, 0), (tile_w, 0), (0, tile_h), (tile_w, tile_h)]

    for (img_path, lab_path), (ox, oy) in zip(quad, offsets):
        im = cv2.imread(str(img_path))
        if im is None:
            continue
        oh, ow = im.shape[:2]
        im_r = cv2.resize(im, (tile_w, tile_h), interpolation=cv2.INTER_LINEAR)
        canvas[oy:oy + tile_h, ox:ox + tile_w] = im_r

        boxes = load_yolo_boxes(lab_path, ow, oh)
        for cls_id, x1, y1, x2, y2 in boxes:
            # 缩放到tile尺寸
            sx1 = x1 * (tile_w / ow) + ox
            sy1 = y1 * (tile_h / oh) + oy
            sx2 = x2 * (tile_w / ow) + ox
            sy2 = y2 * (tile_h / oh) + oy
            # 裁剪并基于面积占比过滤
            orig_area = box_area(sx1, sy1, sx2, sy2)
            cx1, cy1, cx2, cy2 = clip_box(sx1, sy1, sx2, sy2, size, size)
            clipped_area = box_area(cx1, cy1, cx2, cy2)
            if clipped_area < 2.0 or (orig_area > 0 and clipped_area / orig_area < AREA_FRAC_MIN):
                continue
            all_boxes.append((cls_id, cx1, cy1, cx2, cy2))

    return canvas, all_boxes


def create_cutmix_sample(items: List[Tuple[Path, Path]]) -> Tuple[np.ndarray, List[Tuple[int, float, float, float, float]]]:
    # 采样两张图: A为底图, B提供补丁
    a_path, b_path = random.sample(items, 2)
    imgA = cv2.imread(str(a_path[0]))
    imgB = cv2.imread(str(b_path[0]))
    if imgA is None or imgB is None:
        raise RuntimeError('读取图像失败')
    hA, wA = imgA.shape[:2]
    hB, wB = imgB.shape[:2]

    boxesA = load_yolo_boxes(a_path[1], wA, hA)
    boxesB = load_yolo_boxes(b_path[1], wB, hB)

    # 随机矩形补丁尺寸（基于A尺寸）
    rw = random.uniform(0.3, 0.6)
    rh = random.uniform(0.3, 0.6)
    pw = max(4, int(rw * wA))
    ph = max(4, int(rh * hA))
    pw = min(pw, wA - 1, wB - 1)
    ph = min(ph, hA - 1, hB - 1)

    # 在A中的粘贴位置尽量避免与A现有框相交(尝试10次)
    tries = 10
    ax1 = ay1 = 0
    for _ in range(tries):
        ax1 = random.randint(0, max(0, wA - pw))
        ay1 = random.randint(0, max(0, hA - ph))
        ax2, ay2 = ax1 + pw, ay1 + ph
        overlaps = False
        for _, x1, y1, x2, y2 in boxesA:
            ix1, iy1, ix2, iy2 = rect_intersection(ax1, ay1, ax2, ay2, x1, y1, x2, y2)
            if ix2 > ix1 and iy2 > iy1:
                overlaps = True
                break
        if not overlaps:
            break

    # 在B中裁剪同尺寸补丁
    bx1 = random.randint(0, max(0, wB - pw))
    by1 = random.randint(0, max(0, hB - ph))
    bx2, by2 = bx1 + pw, by1 + ph

    patch = imgB[by1:by2, bx1:bx2].copy()
    imgA[ay1:ay2, ax1:ax2] = patch

    out_boxes: List[Tuple[int, float, float, float, float]] = []
    # A的框保持不变
    out_boxes.extend(boxesA)

    # B的框：保留与裁剪补丁相交超过10%面积的部分，映射到A的位置
    for cls_id, x1, y1, x2, y2 in boxesB:
        ix1, iy1, ix2, iy2 = rect_intersection(x1, y1, x2, y2, bx1, by1, bx2, by2)
        inter_area = box_area(ix1, iy1, ix2, iy2)
        orig_area = box_area(x1, y1, x2, y2)
        if orig_area <= 0 or inter_area / orig_area < AREA_FRAC_MIN:
            continue
        # 裁剪到补丁内，并平移到A坐标
        cx1, cy1, cx2, cy2 = ix1, iy1, ix2, iy2
        # 相对于补丁左上角
        rx1 = cx1 - bx1 + ax1
        ry1 = cy1 - by1 + ay1
        rx2 = cx2 - bx1 + ax1
        ry2 = cy2 - by1 + ay1
        # 仍需裁剪到A边界
        rx1, ry1, rx2, ry2 = clip_box(rx1, ry1, rx2, ry2, wA, hA)
        if box_area(rx1, ry1, rx2, ry2) < 2.0:
            continue
        out_boxes.append((cls_id, rx1, ry1, rx2, ry2))

    return imgA, out_boxes

def random_hsv(image_bgr: np.ndarray) -> np.ndarray:
    """近似 YOLO hsv_h/s/v 的随机颜色增强"""
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV).astype(np.float32)
    h, s, v = cv2.split(hsv)

    # H: OpenCV 取值[0,179], 将 ±(0.015*360)=±5.4° 映射为单位≈±3
    h_shift = random.uniform(-HSV_H, HSV_H) * 360.0 / 2.0  # 角度→OpenCV单位
    h = (h + h_shift) % 180.0

    # S/V: 乘法缩放
    s_scale = random.uniform(1.0 - HSV_S, 1.0 + HSV_S)
    v_scale = random.uniform(1.0 - HSV_V, 1.0 + HSV_V)
    s = np.clip(s * s_scale, 0, 255)
    v = np.clip(v * v_scale, 0, 255)

    hsv = cv2.merge([h, s, v]).astype(np.uint8)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def build_affine_matrix(w: int, h: int) -> np.ndarray:
    """组合 旋转+缩放(绕中心) × 剪切 × 平移 的 3x3 仿射矩阵"""
    angle = random.uniform(-DEGREES, DEGREES)
    scale = random.uniform(1.0 - SCALE_GAIN, 1.0 + SCALE_GAIN)
    tx = random.uniform(-TRANSLATE, TRANSLATE) * w
    ty = random.uniform(-TRANSLATE, TRANSLATE) * h
    shear_x = random.uniform(-SHEAR, SHEAR)
    shear_y = random.uniform(-SHEAR, SHEAR)

    center = (w / 2.0, h / 2.0)
    M_rot2 = cv2.getRotationMatrix2D(center, angle, scale)  # 2x3
    M_rot = np.vstack([M_rot2, [0, 0, 1]]).astype(np.float32)  # 3x3

    shx = np.tan(np.deg2rad(shear_x))
    shy = np.tan(np.deg2rad(shear_y))
    M_shear = np.array([[1, shx, 0],
                        [shy, 1, 0],
                        [0,  0,  1]], dtype=np.float32)

    M_trans = np.array([[1, 0, tx],
                        [0, 1, ty],
                        [0, 0,  1]], dtype=np.float32)

    M = M_trans @ M_shear @ M_rot
    return M


def transform_bboxes_xyxy(
    boxes_xyxy: List[Tuple[int, float, float, float, float]],
    M: np.ndarray,
    w: int,
    h: int,
    min_size: float = 2.0,
) -> List[Tuple[int, float, float, float, float]]:
    """对 bbox 四角点应用 3x3 仿射变换, 裁剪并过滤过小目标"""
    out: List[Tuple[int, float, float, float, float]] = []
    for cls_id, x1, y1, x2, y2 in boxes_xyxy:
        # 四角点
        corners = np.array([
            [x1, y1, 1.0],
            [x2, y1, 1.0],
            [x2, y2, 1.0],
            [x1, y2, 1.0],
        ], dtype=np.float32).T  # (3,4)

        trans = M @ corners  # (3,4)
        xs = trans[0, :]
        ys = trans[1, :]

        nx1 = float(np.clip(xs.min(), 0, w - 1))
        ny1 = float(np.clip(ys.min(), 0, h - 1))
        nx2 = float(np.clip(xs.max(), 0, w - 1))
        ny2 = float(np.clip(ys.max(), 0, h - 1))

        if nx2 - nx1 < min_size or ny2 - ny1 < min_size:
            continue
        out.append((cls_id, nx1, ny1, nx2, ny2))
    return out


def maybe_flip_lr(image: np.ndarray, boxes_xyxy: List[Tuple[int, float, float, float, float]]) -> Tuple[np.ndarray, List[Tuple[int, float, float, float, float]]]:
    if random.random() > FLIP_LR_P:
        return image, boxes_xyxy
    h, w = image.shape[:2]
    flipped = cv2.flip(image, 1)
    flipped_boxes: List[Tuple[int, float, float, float, float]] = []
    for cls_id, x1, y1, x2, y2 in boxes_xyxy:
        nx1 = (w - 1) - x2
        nx2 = (w - 1) - x1
        flipped_boxes.append((cls_id, nx1, y1, nx2, y2))
    return flipped, flipped_boxes


def process_one_pair(
    img_path: Path,
    json_path: Path,
    label_to_id: Dict[str, int],
    out_orig: Path,
    out_aug: Path,
) -> Tuple[int, int]:
    """返回(保存原始成功=1/0, 生成增强数量)"""
    # 读取原始图像
    image = cv2.imread(str(img_path))
    if image is None:
        print(f'跳过无法读取的图像: {img_path.name}')
        return 0, 0
    h, w = image.shape[:2]

    # 读取标注框
    boxes_xyxy, jw, jh = parse_labelme_boxes(json_path, label_to_id)
    if jw != w or jh != h:
        # 若 JSON 与图像尺寸不一致, 以图像尺寸为准做一次裁剪
        fixed = []
        for cls_id, x1, y1, x2, y2 in boxes_xyxy:
            nx1 = clamp(x1, 0, w - 1)
            ny1 = clamp(y1, 0, h - 1)
            nx2 = clamp(x2, 0, w - 1)
            ny2 = clamp(y2, 0, h - 1)
            if nx2 > nx1 and ny2 > ny1:
                fixed.append((cls_id, nx1, ny1, nx2, ny2))
        boxes_xyxy = fixed

    # 保存原始图像与 YOLO 标签
    dst_img = out_orig / 'images' / img_path.name
    dst_lab = out_orig / 'labels' / (img_path.stem + '.txt')
    shutil.copy2(img_path, dst_img)
    save_yolo_labels(dst_lab, boxes_xyxy, w, h)

    # 生成增强图
    saved_aug = 0
    for i in range(AUG_PER_IMAGE):
        aug_img = image.copy()
        aug_boxes = boxes_xyxy.copy()

        # 1) 水平翻转(对应 fliplr)
        aug_img, aug_boxes = maybe_flip_lr(aug_img, aug_boxes)

        # 2) 旋转+缩放+剪切+平移
        M = build_affine_matrix(w, h)
        aug_img = cv2.warpAffine(aug_img, M[:2, :], (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
        aug_boxes = transform_bboxes_xyxy(aug_boxes, M, w, h)
        if not aug_boxes:
            continue

        # 3) HSV 调整
        aug_img = random_hsv(aug_img)

        # 保存增强后的样本
        out_name = f"{img_path.stem}_aug{i+1:02d}{img_path.suffix}"
        dst_img_aug = out_aug / 'images' / out_name
        dst_lab_aug = out_aug / 'labels' / (Path(out_name).stem + '.txt')
        cv2.imwrite(str(dst_img_aug), aug_img)
        save_yolo_labels(dst_lab_aug, aug_boxes, w, h)
        saved_aug += 1

    return 1, saved_aug


def generate_extra_samples(out_root: Path) -> Tuple[int, int]:
    """从合并后的数据集中随机挑选最多 NUM_EXTRA 张生成 Mosaic/CutMix.
    返回(生成mosaic数量, 生成cutmix数量)
    """
    items = list_all_images_labels(out_root)
    if not items:
        return 0, 0
    k = min(NUM_EXTRA, len(items))
    mosaic_n, cutmix_n = 0, 0
    img_dir = out_root / 'images'
    lab_dir = out_root / 'labels'

    for i in range(k):
        use_mosaic = (random.random() < MOSAIC_PROB) and (len(items) >= 4)
        if use_mosaic:
            try:
                mosaic_img, mosaic_boxes = create_mosaic_sample(items, MOSAIC_SIZE)
            except Exception:
                # 回退为 CutMix
                try:
                    cm_img, cm_boxes = create_cutmix_sample(items)
                except Exception:
                    continue
                out_name = f'cutmix_{i+1:03d}.png'
                cv2.imwrite(str(img_dir / out_name), cm_img)
                save_yolo_labels(lab_dir / (Path(out_name).stem + '.txt'), cm_boxes, cm_img.shape[1], cm_img.shape[0])
                cutmix_n += 1
                continue

            out_name = f'mosaic_{i+1:03d}.png'
            cv2.imwrite(str(img_dir / out_name), mosaic_img)
            save_yolo_labels(lab_dir / (Path(out_name).stem + '.txt'), mosaic_boxes, MOSAIC_SIZE, MOSAIC_SIZE)
            mosaic_n += 1
        else:
            try:
                cm_img, cm_boxes = create_cutmix_sample(items)
            except Exception:
                continue
            out_name = f'cutmix_{i+1:03d}.png'
            cv2.imwrite(str(img_dir / out_name), cm_img)
            save_yolo_labels(lab_dir / (Path(out_name).stem + '.txt'), cm_boxes, cm_img.shape[1], cm_img.shape[0])
            cutmix_n += 1

    return mosaic_n, cutmix_n


def main() -> None:
    ensure_dirs(OUT_ORIG)
    ensure_dirs(OUT_AUG)

    pairs = list_pairs(RAW_DIR)
    if not pairs:
        print(f'未在 {RAW_DIR} 发现 PNG+JSON 成对文件')
        return

    json_paths = [p[1] for p in pairs]
    names = collect_class_names(json_paths)
    if not names:
        print('未在标注中发现任何类别名称')
        return
    label_to_id: Dict[str, int] = {n: i for i, n in enumerate(names)}

    num_orig, num_aug = 0, 0
    for img_path, json_path in pairs:
        ok, aug_n = process_one_pair(img_path, json_path, label_to_id, OUT_ORIG, OUT_AUG)
        num_orig += ok
        num_aug += aug_n

    # 追加生成 Mosaic/CutMix 样本
    mosaic_n, cutmix_n = generate_extra_samples(OUT_ORIG)

    print('转换与增强完成:')
    print(f'- 类别({len(names)}): {names}')
    print(f'- 原始样本: {num_orig} 张')
    print(f'- 增强样本: {num_aug} 张')
    print(f'- Mosaic 额外样本: {mosaic_n} 张')
    print(f'- CutMix 额外样本: {cutmix_n} 张')
    print(f'-> 输出目录(合并): {OUT_ORIG}')


if __name__ == '__main__':
    main()


