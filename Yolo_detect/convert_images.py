"""
图片格式转换和重命名脚本
将 raw_dataset 文件夹中的所有图片统一转换为 PNG 格式，并按序号重命名
"""
import os
from PIL import Image
from pathlib import Path

# 设置输入和输出文件夹路径
input_folder = 'raw_dataset'
output_folder = 'raw_dataset-formatted'

# 创建输出文件夹
os.makedirs(output_folder, exist_ok=True)

# 支持的图片格式
supported_formats = ('.jpg', '.jpeg', '.png', '.bmp', '.gif', '.tiff')

# 统计信息
total_images = 0
converted_images = 0
failed_images = []

print(f"开始处理图片...")
print(f"输入文件夹: {input_folder}")
print(f"输出文件夹: {output_folder}")
print("-" * 50)

# 获取所有图片文件并排序
all_files = os.listdir(input_folder)
image_files = []

# 先筛选出所有图片文件
for filename in all_files:
    file_ext = os.path.splitext(filename)[1].lower()
    if file_ext in supported_formats:
        image_files.append(filename)

# 对文件名进行排序
image_files.sort()

total_images = len(image_files)

# 处理每个图片文件
for index, filename in enumerate(image_files, start=1):
    file_path = os.path.join(input_folder, filename)
    
    try:
        # 打开图片
        img = Image.open(file_path)
        
        # 如果是 RGBA 模式，转换为 RGB
        if img.mode == 'RGBA':
            img = img.convert('RGB')
        
        # 生成新的文件名（使用序号重命名）
        new_filename = f"image_{index:03d}.png"
        output_path = os.path.join(output_folder, new_filename)
        
        # 保存图片
        img.save(output_path, 'PNG')
        converted_images += 1
        print(f"[OK] {filename} -> {new_filename}")
        
    except Exception as e:
        failed_images.append(filename)
        print(f"[失败] {filename} - 错误: {str(e)}")

# 输出统计信息
print("-" * 50)
print(f"\n转换完成!")
print(f"总共找到图片: {total_images}")
print(f"成功转换: {converted_images}")
print(f"失败: {len(failed_images)}")

if failed_images:
    print(f"\n失败的文件:")
    for f in failed_images:
        print(f"  - {f}")

