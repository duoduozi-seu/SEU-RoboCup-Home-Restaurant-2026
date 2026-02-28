#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
person_analyse_all_test_zyd.py
==============================
完全独立的人员分析测试脚本，不依赖ROS。
直接连接 RealSense 摄像头，实时显示视频窗口，标注：
- 人员检测框
- 距离坐标 (相对于摄像头)
- 性别
- 姿势/动作
- 衣服颜色

使用方法:
    conda activate yolov11
    python person_analyse_all_test_zyd.py

按 'q' 退出, 's' 截图保存
"""

import sys
import os
import time
import cv2
import numpy as np

# 添加路径
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PACKAGE_DIR = os.path.dirname(_SCRIPT_DIR)
_SRC_DIR = os.path.dirname(_PACKAGE_DIR)
sys.path.insert(0, _SRC_DIR)

# 导入核心分析模块
from common.camera import Camera, predict_gender, get_3d_camera_coordinate, color_cn2en


class PersonAnalyseTest:
    """独立的人员分析测试类"""
    
    def __init__(self, camera_type='realsense', serial_number=None):
        print("=" * 60)
        print("人员分析测试 - 无ROS版本")
        print("=" * 60)
        
        # 初始化摄像头
        print("正在初始化摄像头...")
        self.camera = Camera(camera_type=camera_type, serial_number=serial_number)
        
        # 统计信息
        self.fps_start_time = time.time()
        self.frame_count = 0
        self.current_fps = 0
        
        # 窗口名称
        self.window_name = "Person Analyse Test (Press 'q' to quit, 's' to save)"
        
    def draw_info_panel(self, image, info_dict):
        """在图像上绘制信息面板"""
        h, w = image.shape[:2]
        
        # 半透明背景
        overlay = image.copy()
        cv2.rectangle(overlay, (10, 10), (350, 150), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, image, 0.4, 0, image)
        
        # 绘制信息
        y_offset = 30
        for key, value in info_dict.items():
            text = f"{key}: {value}"
            cv2.putText(image, text, (20, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            y_offset += 25
        
        return image
    
    def draw_person_box(self, image, box, info, color=(0, 255, 0)):
        """在图像上绘制人员检测框和信息"""
        x1, y1, x2, y2 = box[:4]
        
        # 绘制边界框
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        
        # 绘制信息标签背景
        label_size = cv2.getTextSize(info, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
        cv2.rectangle(image, (x1, y1 - 25), (x1 + label_size[0] + 10, y1), color, -1)
        
        # 绘制信息文字
        cv2.putText(image, info, (x1 + 5, y1 - 8), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        return image
    
    def analyse_frame(self, color_image, depth_frame, depth_intrin):
        """分析单帧图像"""
        results = []
        
        # 转换颜色空间供分析使用
        img_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        
        # 1. 人体检测
        people = self.camera.people_track.detect_people(
            color_image, 
            depth_frame=depth_frame, 
            depth_intrin=depth_intrin
        )
        
        for person in people:
            x1, y1, x2, y2, pos_info = person
            
            person_data = {
                'box': (x1, y1, x2, y2),
                'distance': None,
                'position': None,
                'gender': 'unknown',
                'pose': 'none',
                'cloth_color': 'unknown'
            }
            
            # 2. 计算距离和坐标
            if pos_info is not None:
                distance, coords = pos_info
                person_data['distance'] = distance
                person_data['position'] = coords  # (x, y, z) 相对于摄像头
            else:
                # 尝试从中心点计算
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                if depth_frame is not None and depth_intrin is not None:
                    try:
                        dis, coord = get_3d_camera_coordinate(cx, cy, depth_frame, depth_intrin)
                        person_data['distance'] = dis
                        person_data['position'] = coord
                    except:
                        pass
            
            # 3. 裁剪人物区域进行分析
            person_crop = img_rgb[y1:y2, x1:x2]
            if person_crop.size > 0:
                # 性别预测
                try:
                    person_data['gender'] = predict_gender(person_crop)
                except:
                    pass
            
            # 4. 姿势检测 (使用完整图像)
            try:
                pose_result = self.camera.pose.get_pose(color_image)
                person_data['pose'] = pose_result
            except:
                pass
            
            # 5. 衣服颜色 (使用上半身区域)
            try:
                upper_body = img_rgb[y1:y1+(y2-y1)//2, x1:x2]
                if upper_body.size > 0:
                    # 简单的颜色分析：取主色调
                    avg_color = np.mean(upper_body, axis=(0, 1))
                    person_data['cloth_color'] = self.get_dominant_color_name(avg_color)
            except:
                pass
            
            results.append(person_data)
        
        return results
    
    def get_dominant_color_name(self, rgb_color):
        """根据RGB值返回颜色名称"""
        r, g, b = rgb_color
        
        # 简单的颜色分类
        if r > 150 and g < 100 and b < 100:
            return "red"
        elif r < 100 and g > 150 and b < 100:
            return "green"
        elif r < 100 and g < 100 and b > 150:
            return "blue"
        elif r > 200 and g > 200 and b > 200:
            return "white"
        elif r < 50 and g < 50 and b < 50:
            return "black"
        elif r > 150 and g > 150 and b < 100:
            return "yellow"
        elif r > 150 and g > 100 and b < 100:
            return "orange"
        elif r > 100 and g > 100 and b > 100:
            return "gray"
        else:
            return "unknown"
    
    def run(self):
        """主循环"""
        print("正在启动摄像头...")
        self.camera.start_camera()
        print("摄像头已启动")
        print("按 'q' 退出, 按 's' 保存截图")
        print("-" * 60)
        
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1280, 720)
        
        try:
            while True:
                # 获取帧
                color_image, depth_image, depth_intrin, depth_frame = self.camera.get_frame()
                
                if color_image is None:
                    print("等待摄像头帧...")
                    time.sleep(0.1)
                    continue
                
                # 计算 FPS
                self.frame_count += 1
                if time.time() - self.fps_start_time >= 1.0:
                    self.current_fps = self.frame_count
                    self.frame_count = 0
                    self.fps_start_time = time.time()
                
                # 分析帧
                people = self.analyse_frame(color_image, depth_frame, depth_intrin)
                
                # 绘制检测结果
                display_image = color_image.copy()
                
                for i, person in enumerate(people):
                    x1, y1, x2, y2 = person['box']
                    
                    # 构建标签信息
                    label_parts = []
                    
                    if person['distance'] is not None:
                        label_parts.append(f"D:{person['distance']:.2f}m")
                    
                    if person['gender'] != 'unknown':
                        label_parts.append(person['gender'])
                    
                    if person['pose'] != 'none':
                        label_parts.append(person['pose'])
                    
                    label = " | ".join(label_parts) if label_parts else f"Person {i+1}"
                    
                    # 绘制边界框
                    self.draw_person_box(display_image, person['box'], label)
                    
                    # 在框下方显示更多信息
                    if person['position'] is not None:
                        x, y, z = person['position']
                        pos_text = f"XYZ: ({x:.2f}, {y:.2f}, {z:.2f})"
                        cv2.putText(display_image, pos_text, (x1, y2 + 20),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                    
                    # 始终显示衣服颜色
                    cloth_text = f"Cloth: {person['cloth_color']}"
                    cv2.putText(display_image, cloth_text, (x1, y2 + 40),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                
                # 绘制信息面板
                info = {
                    "FPS": self.current_fps,
                    "People": len(people),
                    "Time": time.strftime("%H:%M:%S"),
                }
                if people:
                    info["Nearest"] = f"{min([p['distance'] for p in people if p['distance']]):.2f}m" if any(p['distance'] for p in people) else "N/A"
                
                display_image = self.draw_info_panel(display_image, info)
                
                # 显示图像
                cv2.imshow(self.window_name, display_image)
                
                # 键盘控制
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("退出...")
                    break
                elif key == ord('s'):
                    filename = f"screenshot_{int(time.time())}.png"
                    cv2.imwrite(filename, display_image)
                    print(f"截图已保存: {filename}")
        
        except KeyboardInterrupt:
            print("\n手动中断")
        
        finally:
            print("正在关闭摄像头...")
            self.camera.stop()
            cv2.destroyAllWindows()
            print("完成")


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Person Analyse Test (No ROS)')
    parser.add_argument('--camera', type=str, default='realsense',
                       choices=['realsense', 'webcam'],
                       help='Camera type (default: realsense)')
    parser.add_argument('--serial', type=str, default=None,
                       help='RealSense serial number (optional)')
    
    args = parser.parse_args()
    
    # 创建并运行测试
    test = PersonAnalyseTest(
        camera_type=args.camera,
        serial_number=args.serial
    )
    test.run()


if __name__ == "__main__":
    main()
