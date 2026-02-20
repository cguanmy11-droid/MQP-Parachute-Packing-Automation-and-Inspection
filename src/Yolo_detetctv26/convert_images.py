"""
图片格式转换和重命名脚本
将 Raw+Fine_combine 文件夹中的所有图片统一转换为 PNG 格式，并按序号重命名
同时将对应的 JSON 标注文件也按相同规则重命名复制到输出文件夹
"""
import os
import shutil
from PIL import Image
from pathlib import Path

# 设置输入和输出文件夹路径
input_folder = 'Raw+Fine_combine'
output_folder = 'Raw+Fine_combine-formatted'

# 创建输出文件夹
os.makedirs(output_folder, exist_ok=True)

# 支持的图片格式
supported_formats = ('.jpg', '.jpeg', '.png', '.bmp', '.gif', '.tiff')

# 统计信息
total_images = 0
converted_images = 0
copied_jsons = 0
failed_images = []
missing_jsons = []

print(f"开始处理图片和对应的 JSON 文件...")
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

# 处理每个图片文件及其对应的 JSON 文件
for index, filename in enumerate(image_files, start=1):
    file_path = os.path.join(input_folder, filename)
    base_name = os.path.splitext(filename)[0]
    
    try:
        # 打开图片
        img = Image.open(file_path)
        
        # 如果是 RGBA 模式，转换为 RGB
        if img.mode == 'RGBA':
            img = img.convert('RGB')
        
        # 生成新的文件名（使用序号重命名）
        new_image_name = f"image_{index:03d}.png"
        output_path = os.path.join(output_folder, new_image_name)
        
        # 保存图片
        img.save(output_path, 'PNG')
        converted_images += 1
        print(f"[OK] {filename} -> {new_image_name}")
        
        # 处理对应的 JSON 文件
        json_filename = base_name + '.json'
        json_path = os.path.join(input_folder, json_filename)
        
        if os.path.exists(json_path):
            new_json_name = f"image_{index:03d}.json"
            json_output_path = os.path.join(output_folder, new_json_name)
            shutil.copy2(json_path, json_output_path)
            copied_jsons += 1
            print(f"     [JSON] {json_filename} -> {new_json_name}")
        else:
            missing_jsons.append(filename)
            print(f"     [警告] 未找到对应的 JSON 文件: {json_filename}")
        
    except Exception as e:
        failed_images.append(filename)
        print(f"[失败] {filename} - 错误: {str(e)}")

# 输出统计信息
print("-" * 50)
print(f"\n转换完成!")
print(f"总共找到图片: {total_images}")
print(f"成功转换图片: {converted_images}")
print(f"成功复制 JSON: {copied_jsons}")
print(f"图片转换失败: {len(failed_images)}")
print(f"缺少 JSON 文件: {len(missing_jsons)}")

if failed_images:
    print(f"\n转换失败的文件:")
    for f in failed_images:
        print(f"  - {f}")

if missing_jsons:
    print(f"\n缺少对应 JSON 的图片:")
    for f in missing_jsons:
        print(f"  - {f}")

