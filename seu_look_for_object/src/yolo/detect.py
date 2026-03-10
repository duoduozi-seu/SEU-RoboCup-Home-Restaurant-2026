import warnings
warnings.filterwarnings('ignore')
from ultralytics import YOLO
import os
import cv2
import datetime
from pathlib import Path
import sys

# 添加face_detect目录到Python路径
sys.path.append('/home/duo/ultralytics/face_detect')
from newDetect import face_detection

if __name__ == '__main__':
    model = YOLO('/home/duo/ultralytics/runs3/weights/epoch40.pt')
    
    # 设置需要预测的图片文件夹路径
    input_folder = '/home/duo/catkin_ws/src/nav_my_pkg/pictures'  # 修改为你的图片文件夹路径
    
    # 支持图片扩展名
    image_extensions = ['.jpg', '.jpeg', '.png', '.bmp', '.tiff']
    
    # 遍历文件夹中的所有图片进行预测
    for filename in os.listdir(input_folder):
        filepath = os.path.join(input_folder, filename)
        file_ext = Path(filename).suffix.lower()
        
        if file_ext in image_extensions:
            print(f"正在处理图片: {filename}")
            model.predict(source=filepath,
                        imgsz=640,
                        line_width=1,
                        conf=0.25,
                        device='cpu',
                        save=True
                        )

# 添加时间水印

# 设置图片文件夹路径
image_folder = "/home/duo/ultralytics/robocup_detect"

# 支持图片扩展名
image_extensions = ['.jpg', '.jpeg', '.png', '.bmp', '.tiff']

# 获取当前系统日期和时间
current_time = datetime.datetime.now()
date_str = current_time.strftime("%Y-%m-%d")
time_str = current_time.strftime("%H:%M:%S")
first_line_text = f"{date_str} {time_str}"
second_line_text = "NineLoongRushRushRush"

# 遍历文件夹中的所有图片
for filename in os.listdir(image_folder):
    filepath = os.path.join(image_folder, filename)
    file_ext = Path(filename).suffix.lower()
    
    if file_ext in image_extensions:
        # 读取图片
        img = cv2.imread(filepath)
        if img is None:
            print(f"无法读取图片: {filename}")
            continue
        
        # 获取图片尺寸
        height, width = img.shape[:2]
        
        # 设置字体参数
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5  # 字体大小
        font_color = (255, 255, 255)  # 白色文字
        thickness = 1  # 线条粗细
        
        # 计算文字位置（右下角10%区域）
        x_pos = int(width * 0.9)  # 从宽度90%处开始
        y_pos = int(height * 0.9)  # 从高度90%处开始
        
        # 计算文字大小以确定行距
        (text_width1, text_height1), _ = cv2.getTextSize(first_line_text, font, font_scale, thickness)
        (text_width2, text_height2), _ = cv2.getTextSize(second_line_text, font, font_scale, thickness)
        
        # 调整位置确保文字在图片范围内
        x_pos = min(x_pos, width - max(text_width1, text_width2) - 10)
        y_pos = min(y_pos, height - text_height1 - text_height2 - 10)
        
        # 添加第一行文字（日期时间）
        cv2.putText(img, first_line_text, (x_pos, y_pos), font, font_scale, font_color, thickness)
        
        # 添加第二行文字（固定文本）
        cv2.putText(img, second_line_text, (x_pos, y_pos + text_height1 + 5), font, font_scale, font_color, thickness)
        
        # 保存图片（覆盖原文件）
        cv2.imwrite(filepath, img)
        print(f"已处理图片: {filename}")

print("所有图片处理完成！")
