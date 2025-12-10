"""
使用训练好的 YOLOv8 模型检测视频中的目标

功能:
1) 加载训练好的模型权重
2) 读取 video/ 文件夹中的视频
3) 对视频进行目标检测
4) 输出带检测框的处理后视频

使用方法:
    python detect_video.py

依赖安装:
    pip install -r requirements.txt
"""
import os
from pathlib import Path
from ultralytics import YOLO


# 配置参数
VIDEO_DIR = Path('video')                           # 输入视频文件夹
WEIGHT_PATH = Path('runs/yolov8_labelme2/weights/best.pt')  # 训练好的权重
OUTPUT_DIR = Path('video_output')                   # 输出文件夹
CONF_THRESHOLD = 0.5                               # 置信度阈值
IOU_THRESHOLD = 0.55                                # NMS 的 IOU 阈值


def main():
    """主函数：加载模型并处理视频"""
    
    # 检查权重文件是否存在
    if not WEIGHT_PATH.exists():
        print(f'❌ 错误: 权重文件不存在: {WEIGHT_PATH}')
        print('请先训练模型或检查权重路径是否正确')
        return
    
    # 创建输出文件夹
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    
    # 加载训练好的模型
    print(f'📦 加载模型权重: {WEIGHT_PATH}')
    model = YOLO(str(WEIGHT_PATH))
    
    # 查找视频文件
    video_files = []
    for ext in ['*.mp4', '*.avi', '*.mov', '*.mkv']:
        video_files.extend(VIDEO_DIR.glob(ext))
    
    if not video_files:
        print(f'❌ 错误: 在 {VIDEO_DIR} 文件夹中未找到视频文件')
        return
    
    print(f'🎬 找到 {len(video_files)} 个视频文件')
    
    # 处理每个视频
    for video_path in video_files:
        print(f'\n▶️  开始处理: {video_path.name}')
        
        # 输出文件路径
        output_path = OUTPUT_DIR / f'detected_{video_path.name}'
        
        # 使用 YOLO 进行视频推理
        # save=True 会自动保存带检测框的视频
        results = model.predict(
            source=str(video_path),      # 输入视频路径
            conf=CONF_THRESHOLD,         # 置信度阈值
            iou=IOU_THRESHOLD,           # NMS IOU 阈值
            save=True,                   # 保存结果视频
            project=str(OUTPUT_DIR),     # 输出项目文件夹
            name='',                     # 不创建子文件夹
            exist_ok=True,               # 允许覆盖
            show_labels=True,            # 显示标签
            show_conf=True,              # 显示置信度
            line_width=2,                # 边框线宽
        )
        
        print(f'✅ 处理完成: {video_path.name}')
        print(f'💾 输出保存至: {output_path}')
    
    print(f'\n🎉 所有视频处理完成！')
    print(f'📁 输出文件夹: {OUTPUT_DIR.resolve()}')


if __name__ == '__main__':
    main()






