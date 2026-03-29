import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import torch
from ultralytics import YOLO


def parse_args():
    parser = argparse.ArgumentParser(description="YOLO26 检测+分类视频推理并保存结果")
    parser.add_argument(
        "--source",
        type=str,
        # default=r"C:\Users\40386\Desktop\WPI\MQP\top_cam\test_video_short.mp4",
        default=r"C:\Users\40386\Desktop\WPI\MQP\top_cam\Test_video2_fast.mp4",
        help="输入视频路径",
    )
    parser.add_argument(
        "--det_model",
        type=str,
        default=r"C:\Users\40386\Desktop\WPI\MQP\top_cam\runs\detect\runs\detect\yolo26m_holes_all_aug\weights\best.pt",
        help="检测模型 pt 路径",
    )
    parser.add_argument(
        "--cls_model",
        type=str,
        default=r"C:\Users\40386\Desktop\WPI\MQP\top_cam\runs\classify\runs\classify\yolo26m_cls_custom_aug\weights\best.pt",
        help="分类模型 pt 路径（类别期望包含 fully/partial/not）",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=r"C:\Users\40386\Desktop\WPI\MQP\top_cam\runs\video_pred\test_video_pred.mp4",
        help="输出视频路径",
    )
    parser.add_argument("--device", type=str, default="cuda:0", help="推理设备，默认使用 GPU(cuda:0)")
    parser.add_argument("--det_imgsz", type=int, default=640, help="检测输入尺寸")
    parser.add_argument("--cls_h", type=int, default=512, help="分类输入高（letterbox 模式，默认与训练一致）")
    parser.add_argument("--cls_w", type=int, default=256, help="分类输入宽（letterbox 模式，默认与训练一致）")
    parser.add_argument("--cls_ensure_landscape", action="store_true", default=True, help="先将竖图旋转 90° 统一为横图（与训练一致）")
    parser.add_argument(
        "--cls_input_mode",
        type=str,
        default="letterbox",
        choices=["raw", "letterbox"],
        help="cls 输入模式：raw=使用检测裁剪原始尺寸；letterbox=等比例padding到 cls_h×cls_w（默认与训练一致）",
    )
    parser.add_argument(
        "--cls_expand_ratio",
        type=float,
        default=0.0,
        help="分类裁剪框扩张比例（如 0.2 表示宽高各扩 20%，仅影响分类输入，不影响显示框）",
    )
    parser.add_argument("--conf", type=float, default=0.35, help="检测置信度阈值")
    parser.add_argument("--iou", type=float, default=0.45, help="检测 NMS IoU 阈值")
    parser.add_argument("--show", action="store_true", help="是否实时显示窗口")
    return parser.parse_args()


def _to_float(x):
    if hasattr(x, "item"):
        return float(x.item())
    return float(x)


def _get_top1_label_and_conf(result):
    if result is None or result.probs is None:
        return "unknown", 0.0

    probs = result.probs
    top1 = int(probs.top1)
    conf = _to_float(probs.top1conf)
    names = result.names
    if isinstance(names, dict):
        label = str(names.get(top1, top1))
    else:
        label = str(names[top1])
    return label, conf


def _label_to_color(label: str):
    name = _normalize_label(label)
    if name == "fully":
        return (0, 255, 0)  # green
    if name == "partial":
        return (0, 255, 255)  # yellow
    if name == "not":
        return (0, 0, 255)  # red
    return (255, 255, 255)  # unknown -> white


def _normalize_label(label: str) -> str:
    """Map model class names to fully/partial/not."""
    s = str(label).strip().lower().replace("-", "_").replace(" ", "_")
    if "not" in s:
        return "not"
    if "partial" in s:
        return "partial"
    if "fully" in s or "full" in s:
        return "fully"
    return s


def _resolve_gpu_device(device_arg: str) -> str:
    dev = str(device_arg).strip().lower()
    if dev in {"cpu", "-1"}:
        raise ValueError("当前脚本已改为使用 GPU，请将 --device 设置为 cuda:0 或 0。")
    if not torch.cuda.is_available():
        raise RuntimeError("未检测到可用 CUDA GPU，请先安装 CUDA 版 PyTorch 或检查显卡驱动。")
    if dev == "0":
        return "cuda:0"
    if dev.startswith("cuda:"):
        return dev
    return device_arg


def _ensure_landscape_bgr(img_bgr):
    """If portrait (h>w), rotate 90 deg to landscape."""
    if img_bgr is None:
        return img_bgr
    h, w = img_bgr.shape[:2]
    if h > w:
        return cv2.rotate(img_bgr, cv2.ROTATE_90_CLOCKWISE)
    return img_bgr


def _letterbox_rect_bgr(img_bgr, out_h: int, out_w: int, fill_value: int = 114):
    """保持长宽比，padding 到 out_h×out_w。输入/输出都是 BGR(np.ndarray)。"""
    if img_bgr is None:
        return img_bgr
    h, w = img_bgr.shape[:2]
    if h <= 0 or w <= 0:
        return img_bgr

    out_h = int(out_h)
    out_w = int(out_w)
    scale = min(out_w / w, out_h / h)
    nw = max(1, int(round(w * scale)))
    nh = max(1, int(round(h * scale)))
    if (nw, nh) != (w, h):
        img_bgr = cv2.resize(img_bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)

    canvas = (fill_value * np.ones((out_h, out_w, 3), dtype=img_bgr.dtype))
    left = (out_w - nw) // 2
    top = (out_h - nh) // 2
    canvas[top : top + nh, left : left + nw] = img_bgr
    return canvas


def _expand_xyxy_for_cls(x1: int, y1: int, x2: int, y2: int, w: int, h: int, ratio: float):
    """Expand bbox by ratio for classification crop only."""
    r = max(0.0, float(ratio))
    if r <= 0.0:
        return x1, y1, x2, y2

    bw = x2 - x1
    bh = y2 - y1
    cx = (x1 + x2) / 2.0
    cy = (y1 + y2) / 2.0
    nw = bw * (1.0 + r)
    nh = bh * (1.0 + r)

    ex1 = int(round(cx - nw / 2.0))
    ey1 = int(round(cy - nh / 2.0))
    ex2 = int(round(cx + nw / 2.0))
    ey2 = int(round(cy + nh / 2.0))

    ex1 = max(0, min(ex1, w - 1))
    ey1 = max(0, min(ey1, h - 1))
    ex2 = max(0, min(ex2, w))
    ey2 = max(0, min(ey2, h))
    return ex1, ey1, ex2, ey2


def main():
    args = parse_args()
    run_device = _resolve_gpu_device(args.device)

    src_path = Path(args.source)
    det_model_path = Path(args.det_model)
    cls_model_path = Path(args.cls_model)
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    if not src_path.exists():
        raise FileNotFoundError(f"输入视频不存在: {src_path}")
    if not det_model_path.exists():
        raise FileNotFoundError(f"检测模型不存在: {det_model_path}")
    if not cls_model_path.exists():
        raise FileNotFoundError(f"分类模型不存在: {cls_model_path}")

    det_model = YOLO(str(det_model_path))
    cls_model = YOLO(str(cls_model_path))
    cap = cv2.VideoCapture(str(src_path))
    if not cap.isOpened():
        raise RuntimeError(f"无法打开视频: {src_path}")

    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps <= 0:
        fps = 30.0
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    writer = cv2.VideoWriter(
        str(out_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (width, height),
    )
    if not writer.isOpened():
        cap.release()
        raise RuntimeError(f"无法写入输出视频: {out_path}")

    print(f"[INFO] source:    {src_path}")
    print(f"[INFO] det model: {det_model_path}")
    print(f"[INFO] cls model: {cls_model_path}")
    print(f"[INFO] output: {out_path}")
    print(f"[INFO] device: {run_device} ({torch.cuda.get_device_name(0)})")
    print(f"[INFO] cls_expand_ratio: {args.cls_expand_ratio:.3f}")
    print(f"[INFO] cls_input_mode: {args.cls_input_mode}")
    print(
        f"[INFO] cls_preprocess: ensure_landscape={args.cls_ensure_landscape}, "
        f"letterbox={args.cls_h}x{args.cls_w}"
    )
    print(f"[INFO] video info: {width}x{height}, fps={fps:.2f}, total_frames={total_frames}")

    frame_idx = 0
    total_infer_time = 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        t0 = time.perf_counter()
        det_results = det_model.predict(
            source=frame,
            task="detect",
            imgsz=args.det_imgsz,
            device=run_device,
            conf=args.conf,
            iou=args.iou,
            verbose=False,
        )

        n_det = 0
        cls_cost = 0.0
        boxes = det_results[0].boxes if det_results else None
        if boxes is not None and len(boxes) > 0:
            h, w = frame.shape[:2]
            crops = []
            box_infos = []
            for box in boxes:
                xyxy = box.xyxy[0].tolist()
                x1, y1, x2, y2 = map(int, xyxy)
                x1 = max(0, min(x1, w - 1))
                y1 = max(0, min(y1, h - 1))
                x2 = max(0, min(x2, w))
                y2 = max(0, min(y2, h))
                if x2 <= x1 or y2 <= y1:
                    continue

                # 仅分类输入使用扩张后的框；显示框仍保持检测原框(x1,y1,x2,y2)
                cx1, cy1, cx2, cy2 = _expand_xyxy_for_cls(x1, y1, x2, y2, w, h, args.cls_expand_ratio)
                if cx2 <= cx1 or cy2 <= cy1:
                    continue

                crop = frame[cy1:cy2, cx1:cx2]
                if crop.size == 0:
                    continue

                # 分类输入模式:
                # - raw: 直接使用 detect 裁剪的原始尺寸和比例（不做额外 resize/pad）
                # - letterbox: 与训练一致（先竖图转横图，再等比例 padding 到矩形尺寸）
                if args.cls_input_mode == "letterbox":
                    if args.cls_ensure_landscape:
                        crop = _ensure_landscape_bgr(crop)
                    crop = _letterbox_rect_bgr(crop, args.cls_h, args.cls_w, fill_value=114)
                crops.append(crop)
                box_infos.append((x1, y1, x2, y2, box))

            cls_results = []
            if crops:
                t_cls = time.perf_counter()
                cls_kwargs = {
                    "source": crops,
                    "task": "classify",
                    "device": run_device,
                    "batch": max(1, len(crops)),
                    "verbose": False,
                }
                # raw 模式不显式指定 imgsz；letterbox 模式按训练输入长边指定
                if args.cls_input_mode == "letterbox":
                    cls_kwargs["imgsz"] = max(args.cls_h, args.cls_w)
                cls_results = cls_model.predict(**cls_kwargs)
                cls_cost += time.perf_counter() - t_cls

            for i, (x1, y1, x2, y2, box) in enumerate(box_infos):
                cls_label_raw, cls_conf = _get_top1_label_and_conf(cls_results[i] if i < len(cls_results) else None)
                cls_label = _normalize_label(cls_label_raw)
                det_conf = _to_float(box.conf[0]) if box.conf is not None else 0.0
                color = _label_to_color(cls_label)

                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                text = f"{cls_label}({cls_label_raw}) | cls={cls_conf:.2f} det={det_conf:.2f}"
                ty = max(20, y1 - 8)
                cv2.putText(
                    frame,
                    text,
                    (x1, ty),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color,
                    2,
                    cv2.LINE_AA,
                )
                n_det += 1

        infer_cost = time.perf_counter() - t0
        total_infer_time += infer_cost
        infer_fps = 1.0 / max(infer_cost, 1e-9)
        status = f"dets={n_det} | inferFPS={infer_fps:.1f} | cls_t={cls_cost*1000:.1f}ms"
        cv2.putText(frame, status, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

        writer.write(frame)
        frame_idx += 1

        if frame_idx % 100 == 0:
            print(f"[INFO] processed {frame_idx}/{total_frames if total_frames > 0 else '?'} frames")

        if args.show:
            cv2.imshow("YOLO26 Detect + Classify", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    writer.release()
    if args.show:
        cv2.destroyAllWindows()

    avg_infer_fps = frame_idx / max(total_infer_time, 1e-9)
    print(f"[DONE] frames={frame_idx}, avg_infer_fps={avg_infer_fps:.2f}")
    print(f"[DONE] saved to: {out_path}")


if __name__ == "__main__":
    main()

