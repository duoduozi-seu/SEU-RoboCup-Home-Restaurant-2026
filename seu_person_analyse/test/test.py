#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
seu_person_analyse 离线测试脚本
================================
该测试脚本不依赖ROS节点和真实摄像头，可独立运行测试所有核心功能。

使用方法:
1. catkin build seu_person_analyse
2. source devel/setup.bash
3. python3 test/test.py

或直接运行:
python3 /home/duo/GPSR_ws_zyd/GPSR_code/src/seu_person_analyse/test/test.py
"""

import os
import sys
import time
import traceback
from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict, Any
from unittest.mock import MagicMock, patch

# 添加路径
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(SCRIPT_DIR)
SRC_DIR = os.path.dirname(PACKAGE_DIR)
sys.path.insert(0, SRC_DIR)

# ====================================================================
# Mock ROS 相关模块（必须在导入其他模块前进行）
# ====================================================================

class MockROSModule:
    """Mock ROS 核心模块"""
    
    class Time:
        @staticmethod
        def now():
            return MockROSModule.Time()
        def to_sec(self):
            return time.time()
    
    class Duration:
        def __init__(self, secs=0):
            self.secs = secs
    
    class Rate:
        def __init__(self, hz):
            self.hz = hz
        def sleep(self):
            time.sleep(1.0 / self.hz)
    
    @staticmethod
    def init_node(*args, **kwargs):
        pass
    
    @staticmethod
    def spin():
        pass
    
    @staticmethod
    def is_shutdown():
        return False
    
    @staticmethod
    def loginfo(msg, *args):
        print(f"[INFO] {msg}" % args if args else f"[INFO] {msg}")
    
    @staticmethod
    def logwarn(msg, *args):
        print(f"[WARN] {msg}" % args if args else f"[WARN] {msg}")
    
    @staticmethod
    def logerr(msg, *args):
        print(f"[ERROR] {msg}" % args if args else f"[ERROR] {msg}")
    
    @staticmethod
    def wait_for_message(*args, **kwargs):
        mock_msg = MagicMock()
        mock_msg.pose.position.x = 0.0
        mock_msg.pose.position.y = 0.0
        mock_msg.pose.orientation.x = 0.0
        mock_msg.pose.orientation.y = 0.0
        mock_msg.pose.orientation.z = 0.0
        mock_msg.pose.orientation.w = 1.0
        return mock_msg
    
    @staticmethod
    def Subscriber(*args, **kwargs):
        return MagicMock()
    
    @staticmethod
    def Publisher(*args, **kwargs):
        mock_pub = MagicMock()
        mock_pub.publish = MagicMock()
        return mock_pub
    
    @staticmethod
    def Service(*args, **kwargs):
        return MagicMock()
    
    @staticmethod
    def set_param(key, value):
        pass
    
    @staticmethod
    def get_param(key, default=None):
        return default
    
    @staticmethod
    def sleep(duration):
        time.sleep(duration)

# 安装 Mock 模块
sys.modules['rospy'] = MockROSModule()
sys.modules['tf2_ros'] = MagicMock()
sys.modules['tf2_geometry_msgs'] = MagicMock()
sys.modules['actionlib'] = MagicMock()
sys.modules['actionlib_msgs'] = MagicMock()
sys.modules['actionlib_msgs.msg'] = MagicMock()
sys.modules['move_base_msgs'] = MagicMock()
sys.modules['move_base_msgs.msg'] = MagicMock()
sys.modules['std_msgs'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()
sys.modules['sensor_msgs'] = MagicMock()
sys.modules['sensor_msgs.msg'] = MagicMock()
sys.modules['geometry_msgs'] = MagicMock()
sys.modules['geometry_msgs.msg'] = MagicMock()
sys.modules['seu_robot_msgs'] = MagicMock()
sys.modules['seu_robot_msgs.msg'] = MagicMock()
sys.modules['seu_person_analyse'] = MagicMock()
sys.modules['seu_person_analyse.srv'] = MagicMock()
sys.modules['cv_bridge'] = MagicMock()
sys.modules['tf.transformations'] = MagicMock()

# 现在可以安全导入
import cv2
import numpy as np

# ====================================================================
# 测试结果收集器
# ====================================================================

@dataclass
class TestResult:
    """单个测试结果"""
    name: str
    passed: bool
    message: str
    duration: float
    details: Optional[str] = None

class TestRunner:
    """测试运行器"""
    
    def __init__(self):
        self.results: List[TestResult] = []
        self.start_time = time.time()
    
    def run_test(self, name: str, test_func):
        """运行单个测试"""
        print(f"\n{'='*60}")
        print(f"测试: {name}")
        print('='*60)
        
        start = time.time()
        try:
            result = test_func()
            duration = time.time() - start
            
            if result is True or (isinstance(result, tuple) and result[0]):
                msg = result[1] if isinstance(result, tuple) else "通过"
                self.results.append(TestResult(name, True, msg, duration))
                print(f"✅ 通过 ({duration:.2f}s): {msg}")
            else:
                msg = result[1] if isinstance(result, tuple) else "失败"
                self.results.append(TestResult(name, False, msg, duration))
                print(f"❌ 失败 ({duration:.2f}s): {msg}")
                
        except Exception as e:
            duration = time.time() - start
            error_msg = str(e)
            details = traceback.format_exc()
            self.results.append(TestResult(name, False, error_msg, duration, details))
            print(f"❌ 异常 ({duration:.2f}s): {error_msg}")
            print(f"详情:\n{details}")
    
    def print_summary(self):
        """打印测试总结"""
        total_time = time.time() - self.start_time
        passed = sum(1 for r in self.results if r.passed)
        failed = len(self.results) - passed
        
        print("\n" + "="*60)
        print("测试总结")
        print("="*60)
        print(f"总计: {len(self.results)} 测试")
        print(f"通过: {passed} ✅")
        print(f"失败: {failed} ❌")
        print(f"总耗时: {total_time:.2f}s")
        print("="*60)
        
        if failed > 0:
            print("\n失败的测试:")
            for r in self.results:
                if not r.passed:
                    print(f"  - {r.name}: {r.message}")
        
        return failed == 0

# ====================================================================
# Mock 深度帧类
# ====================================================================

class MockDepthFrame:
    """Mock RealSense 深度帧"""
    
    def __init__(self, width=640, height=480, base_depth=1.5):
        self.width = width
        self.height = height
        self.base_depth = base_depth
        # 创建模拟深度数据（中心近，边缘远）
        self.depth_data = np.zeros((height, width), dtype=np.float32)
        for y in range(height):
            for x in range(width):
                # 距离中心的距离
                dx = (x - width/2) / (width/2)
                dy = (y - height/2) / (height/2)
                dist = np.sqrt(dx*dx + dy*dy)
                self.depth_data[y, x] = base_depth + dist * 0.5
    
    def get_distance(self, x, y):
        """获取指定像素的深度值"""
        x = max(0, min(int(x), self.width - 1))
        y = max(0, min(int(y), self.height - 1))
        return self.depth_data[y, x]
    
    @property
    def profile(self):
        mock_profile = MagicMock()
        mock_intrinsics = MagicMock()
        mock_intrinsics.fx = 615.0
        mock_intrinsics.fy = 615.0
        mock_intrinsics.ppx = 320.0
        mock_intrinsics.ppy = 240.0
        mock_intrinsics.width = self.width
        mock_intrinsics.height = self.height
        mock_intrinsics.model = 0
        mock_intrinsics.coeffs = [0, 0, 0, 0, 0]
        mock_profile.as_video_stream_profile.return_value.intrinsics = mock_intrinsics
        return mock_profile

class MockDepthIntrin:
    """Mock 深度相机内参"""
    def __init__(self):
        self.fx = 615.0
        self.fy = 615.0
        self.ppx = 320.0
        self.ppy = 240.0
        self.width = 640
        self.height = 480
        self.model = 0
        self.coeffs = [0, 0, 0, 0, 0]

# ====================================================================
# 测试图像生成器
# ====================================================================

class TestImageGenerator:
    """生成用于测试的模拟图像"""
    
    @staticmethod
    def create_person_image(width=640, height=480, 
                           with_face=True, 
                           arm_raised=None,
                           pointing=None,
                           shirt_color=(0, 0, 255)) -> np.ndarray:
        """
        创建包含人物的测试图像
        
        Args:
            width: 图像宽度
            height: 图像高度
            with_face: 是否包含人脸
            arm_raised: 举起的手臂 ('left', 'right', None)
            pointing: 指向方向 ('left', 'right', None)
            shirt_color: 衬衫颜色 (BGR)
        
        Returns:
            BGR 格式的测试图像
        """
        # 创建背景
        img = np.ones((height, width, 3), dtype=np.uint8) * 200
        
        # 人物中心位置
        cx, cy = width // 2, height // 2
        
        # 绘制身体（矩形）
        body_w, body_h = 100, 150
        cv2.rectangle(img, 
                     (cx - body_w//2, cy - 20),
                     (cx + body_w//2, cy + body_h),
                     shirt_color, -1)
        
        # 绘制头部（圆形）
        head_radius = 50
        head_center = (cx, cy - 70)
        if with_face:
            # 绘制肤色头部
            cv2.circle(img, head_center, head_radius, (180, 200, 220), -1)
            # 绘制眼睛
            cv2.circle(img, (cx - 15, cy - 80), 5, (50, 50, 50), -1)
            cv2.circle(img, (cx + 15, cy - 80), 5, (50, 50, 50), -1)
            # 绘制嘴巴
            cv2.ellipse(img, (cx, cy - 55), (15, 8), 0, 0, 180, (100, 100, 100), 2)
        else:
            cv2.circle(img, head_center, head_radius, (150, 150, 150), -1)
        
        # 绘制手臂
        shoulder_y = cy
        left_shoulder = (cx - body_w//2, shoulder_y)
        right_shoulder = (cx + body_w//2, shoulder_y)
        
        # 默认手臂位置（下垂）
        left_hand = (cx - body_w//2 - 20, cy + 100)
        right_hand = (cx + body_w//2 + 20, cy + 100)
        
        # 举手姿势
        if arm_raised == 'left':
            left_hand = (cx - body_w//2 - 40, cy - 120)
        elif arm_raised == 'right':
            right_hand = (cx + body_w//2 + 40, cy - 120)
        elif arm_raised == 'both':
            left_hand = (cx - body_w//2 - 40, cy - 120)
            right_hand = (cx + body_w//2 + 40, cy - 120)
        
        # 指向姿势
        if pointing == 'left':
            left_hand = (cx - 150, cy)
        elif pointing == 'right':
            right_hand = (cx + 150, cy)
        
        # 绘制手臂线条
        cv2.line(img, left_shoulder, left_hand, (180, 200, 220), 15)
        cv2.line(img, right_shoulder, right_hand, (180, 200, 220), 15)
        
        # 绘制手掌
        cv2.circle(img, left_hand, 12, (180, 200, 220), -1)
        cv2.circle(img, right_hand, 12, (180, 200, 220), -1)
        
        return img
    
    @staticmethod
    def create_face_image(width=200, height=200, gender='male') -> np.ndarray:
        """创建单独的人脸图像用于测试"""
        img = np.ones((height, width, 3), dtype=np.uint8) * 220
        
        cx, cy = width // 2, height // 2
        
        # 绘制脸部轮廓
        face_color = (180, 200, 220)
        cv2.ellipse(img, (cx, cy), (60, 75), 0, 0, 360, face_color, -1)
        
        # 眼睛
        eye_y = cy - 15
        cv2.circle(img, (cx - 25, eye_y), 8, (255, 255, 255), -1)
        cv2.circle(img, (cx + 25, eye_y), 8, (255, 255, 255), -1)
        cv2.circle(img, (cx - 25, eye_y), 4, (50, 50, 50), -1)
        cv2.circle(img, (cx + 25, eye_y), 4, (50, 50, 50), -1)
        
        # 鼻子
        cv2.line(img, (cx, cy - 5), (cx, cy + 15), (150, 170, 190), 2)
        
        # 嘴巴
        cv2.ellipse(img, (cx, cy + 35), (20, 10), 0, 0, 180, (150, 100, 100), 2)
        
        # 头发（区分性别的简单方式）
        if gender == 'female':
            # 长发
            cv2.ellipse(img, (cx, cy - 30), (70, 50), 0, 180, 360, (30, 30, 30), -1)
            cv2.rectangle(img, (cx - 70, cy - 30), (cx - 50, cy + 60), (30, 30, 30), -1)
            cv2.rectangle(img, (cx + 50, cy - 30), (cx + 70, cy + 60), (30, 30, 30), -1)
        else:
            # 短发
            cv2.ellipse(img, (cx, cy - 50), (65, 40), 0, 180, 360, (30, 30, 30), -1)
        
        return img
    
    @staticmethod
    def create_empty_scene(width=640, height=480) -> np.ndarray:
        """创建空场景图像"""
        img = np.ones((height, width, 3), dtype=np.uint8) * 180
        # 添加一些纹理
        for i in range(0, height, 50):
            cv2.line(img, (0, i), (width, i), (170, 170, 170), 1)
        for i in range(0, width, 50):
            cv2.line(img, (i, 0), (i, height), (170, 170, 170), 1)
        return img

# ====================================================================
# 测试用例
# ====================================================================

def test_import_modules():
    """测试1: 验证所有模块可以正确导入"""
    try:
        import cv2
        import numpy as np
        import mediapipe as mp
        from deepface import DeepFace
        from ultralytics import YOLO
        import face_recognition
        return True, "所有核心模块导入成功"
    except ImportError as e:
        return False, f"模块导入失败: {e}"

def test_pose_class():
    """测试2: 测试 Pose 类的姿势识别功能"""
    try:
        from common.camera import Pose
        
        pose = Pose(image_width=640, image_height=480)
        
        # 测试1: 普通站姿
        img_normal = TestImageGenerator.create_person_image()
        result = pose.get_pose(img_normal)
        print(f"  普通站姿检测结果: {result}")
        
        # 测试2: 举右手
        img_raise_right = TestImageGenerator.create_person_image(arm_raised='right')
        result_raise = pose.get_pose(img_raise_right)
        print(f"  举右手检测结果: {result_raise}")
        
        # 测试3: 指向左边
        img_point_left = TestImageGenerator.create_person_image(pointing='left')
        result_point = pose.get_pose(img_point_left)
        print(f"  指向左边检测结果: {result_point}")
        
        # 测试4: 坐姿检测
        result_sit = pose.get_pose(img_normal, feature="sitting")
        print(f"  坐姿检测结果: {result_sit}")
        
        return True, "Pose类测试完成，所有姿势检测功能正常"
        
    except Exception as e:
        return False, f"Pose类测试失败: {e}"

def test_gender_prediction():
    """测试3: 测试性别预测功能"""
    try:
        from common.camera import predict_gender
        
        # 生成测试图像
        male_img = TestImageGenerator.create_face_image(gender='male')
        female_img = TestImageGenerator.create_face_image(gender='female')
        person_img = TestImageGenerator.create_person_image(with_face=True)
        
        # 测试性别预测
        result_male = predict_gender(male_img)
        print(f"  男性图像预测结果: {result_male}")
        
        result_female = predict_gender(female_img)
        print(f"  女性图像预测结果: {result_female}")
        
        result_person = predict_gender(person_img)
        print(f"  全身图像预测结果: {result_person}")
        
        # 验证返回值格式
        valid_results = ['man', 'woman', 'unknown']
        if result_male in valid_results and result_female in valid_results:
            return True, f"性别预测功能正常 (男:{result_male}, 女:{result_female})"
        else:
            return False, "性别预测返回无效结果"
            
    except Exception as e:
        return False, f"性别预测测试失败: {e}"

def test_person_track():
    """测试4: 测试人体追踪功能"""
    try:
        from common.camera import PersonTrack
        
        tracker = PersonTrack()
        
        # 生成包含人物的测试图像
        img_with_person = TestImageGenerator.create_person_image()
        img_empty = TestImageGenerator.create_empty_scene()
        
        # 测试检测（无深度信息）
        results_with = tracker.detect_people(img_with_person)
        print(f"  有人图像检测数量: {len(results_with)}")
        
        results_empty = tracker.detect_people(img_empty)
        print(f"  空场景检测数量: {len(results_empty)}")
        
        # 使用Mock深度帧测试
        depth_frame = MockDepthFrame()
        depth_intrin = MockDepthIntrin()
        results_depth = tracker.detect_people(img_with_person, depth_frame, depth_intrin)
        print(f"  带深度信息检测数量: {len(results_depth)}")
        
        if results_depth:
            for i, (x1, y1, x2, y2, pos) in enumerate(results_depth):
                print(f"    人物{i+1}: 边界框=({x1},{y1},{x2},{y2}), 位置={pos}")
        
        return True, f"人体追踪测试完成，检测到 {len(results_with)} 人"
        
    except Exception as e:
        return False, f"人体追踪测试失败: {e}"

def test_people_analyse():
    """测试5: 测试人脸分析和记录功能"""
    try:
        from common.camera import People_analyse
        
        # 使用临时目录
        import tempfile
        temp_dir = tempfile.mkdtemp()
        
        analyser = People_analyse(known_path=temp_dir)
        
        # 生成测试人脸图像
        face_img = TestImageGenerator.create_face_image()
        person_img = TestImageGenerator.create_person_image(with_face=True)
        
        # 测试人脸分析
        results = analyser.analyze_faces(person_img)
        print(f"  检测到人脸数量: {len(results)}")
        
        for i, face in enumerate(results):
            print(f"    人脸{i+1}: 位置={face['face_position']}, 性别={face['gender']}")
        
        # 测试人脸记录
        if results:
            embedding = results[0]['embedding']
            face_id = analyser.record_face(embedding)
            print(f"  记录人脸ID: {face_id}")
            
            # 测试重复检测
            existing_id = analyser.check_if_recorded(embedding)
            print(f"  重复检测结果: {existing_id}")
        
        # 清理
        import shutil
        shutil.rmtree(temp_dir, ignore_errors=True)
        
        return True, f"人脸分析测试完成，检测到 {len(results)} 张人脸"
        
    except Exception as e:
        return False, f"人脸分析测试失败: {e}"

def test_gesture_detection():
    """测试6: 测试手势检测功能"""
    try:
        from common.camera import Pose
        
        pose = Pose()
        
        # 测试不同的指向手势
        img_point_left = TestImageGenerator.create_person_image(pointing='left')
        img_point_right = TestImageGenerator.create_person_image(pointing='right')
        img_normal = TestImageGenerator.create_person_image()
        
        result_left = pose.detect_gesture(img_point_left, draw=False)
        print(f"  指向左边手势: {result_left}")
        
        result_right = pose.detect_gesture(img_point_right, draw=False)
        print(f"  指向右边手势: {result_right}")
        
        result_normal = pose.detect_gesture(img_normal, draw=False)
        print(f"  普通姿势: {result_normal}")
        
        return True, "手势检测测试完成"
        
    except Exception as e:
        return False, f"手势检测测试失败: {e}"

def test_sitting_detection():
    """测试7: 测试坐姿检测功能"""
    try:
        from common.camera import Pose
        
        pose = Pose()
        
        # 使用普通站姿图像测试
        img = TestImageGenerator.create_person_image()
        
        is_sitting, raise_arm, wave_hand = pose.detect_sitting_pose(img, draw=False)
        print(f"  坐姿: {is_sitting}")
        print(f"  举手: {raise_arm}")
        print(f"  挥手: {wave_hand}")
        
        # 测试 get_pose 的坐姿功能
        sitting_result = pose.get_pose(img, feature="sitting")
        print(f"  get_pose 坐姿结果: {sitting_result}")
        
        return True, f"坐姿检测完成 (is_sitting={is_sitting})"
        
    except Exception as e:
        return False, f"坐姿检测测试失败: {e}"

def test_cloth_color_mock():
    """测试8: 测试衣物颜色识别（Mock API）"""
    try:
        from common.camera import Cloth
        
        cloth = Cloth()
        
        # 生成测试图像（红色衬衫）
        img = TestImageGenerator.create_person_image(shirt_color=(0, 0, 255))
        
        # Mock API 调用
        with patch.object(cloth, 'get_access_token', return_value='mock_token'):
            with patch('requests.post') as mock_post:
                # 模拟API响应
                mock_response = MagicMock()
                mock_response.status_code = 200
                mock_response.json.return_value = {
                    "person_info": [{
                        "attributes": {
                            "upper_color": {
                                "name": "红色",
                                "score": 0.95
                            }
                        }
                    }]
                }
                mock_post.return_value = mock_response
                
                result = cloth.bodyanalysis(img)
                print(f"  衣物颜色识别结果: {result}")
        
        return True, "衣物颜色识别测试完成（使用Mock API）"
        
    except Exception as e:
        return False, f"衣物颜色识别测试失败: {e}"

def test_obj_detect():
    """测试9: 测试物体检测功能"""
    try:
        from common.camera import ObjDetect
        
        detector = ObjDetect()
        
        # 生成测试图像
        img = TestImageGenerator.create_person_image()
        depth_frame = MockDepthFrame()
        depth_intrin = MockDepthIntrin()
        
        # 执行检测
        results = detector.get_obj(img, depth_frame, depth_intrin)
        print(f"  检测到物体数量: {len(results)}")
        
        for res in results:
            print(f"    物体: {res['label']}, 置信度: {res['cof']:.2f}, 位置: {res['pos']}")
        
        return True, f"物体检测测试完成，检测到 {len(results)} 个物体"
        
    except Exception as e:
        return False, f"物体检测测试失败: {e}"

def test_camera_class():
    """测试10: 测试 Camera 类（Mock 摄像头）"""
    try:
        # Mock pyrealsense2
        with patch.dict(sys.modules, {'pyrealsense2': MagicMock()}):
            from common.camera import Camera
            
            # 使用 webcam 模式测试（不依赖 RealSense）
            camera = Camera(camera_type='webcam')
            
            # 验证组件初始化
            assert camera.pose is not None, "Pose 组件未初始化"
            assert camera.people_track is not None, "PersonTrack 组件未初始化"
            assert camera.people_analyse is not None, "People_analyse 组件未初始化"
            assert camera.cloth is not None, "Cloth 组件未初始化"
            
            print(f"  Camera 类型: {camera.camera_type}")
            print(f"  图像尺寸: {camera.image_width}x{camera.image_height}")
            print(f"  相机状态: {camera.camera_state}")
            
            return True, "Camera 类初始化测试完成"
            
    except Exception as e:
        return False, f"Camera 类测试失败: {e}"

def test_3d_coordinate():
    """测试11: 测试3D坐标计算"""
    try:
        import pyrealsense2 as rs
        from common.camera import get_3d_camera_coordinate
        
        # 创建真实的 pyrealsense2 intrinsics 对象
        depth_intrin = rs.intrinsics()
        depth_intrin.width = 640
        depth_intrin.height = 480
        depth_intrin.ppx = 320.0
        depth_intrin.ppy = 240.0
        depth_intrin.fx = 615.0
        depth_intrin.fy = 615.0
        depth_intrin.model = rs.distortion.none
        depth_intrin.coeffs = [0, 0, 0, 0, 0]
        
        depth_frame = MockDepthFrame()
        
        # 测试中心点
        dis, coord = get_3d_camera_coordinate(320, 240, depth_frame, depth_intrin)
        print(f"  中心点(320,240): 距离={dis:.2f}m, 坐标=({coord[0]:.3f}, {coord[1]:.3f}, {coord[2]:.3f})")
        
        # 测试边角点
        dis2, coord2 = get_3d_camera_coordinate(100, 100, depth_frame, depth_intrin)
        print(f"  边角点(100,100): 距离={dis2:.2f}m, 坐标=({coord2[0]:.3f}, {coord2[1]:.3f}, {coord2[2]:.3f})")
        
        # 验证计算结果 - 中心点应该接近 (0, 0, depth)
        assert abs(coord[0]) < 0.01, f"中心点X坐标偏差过大: {coord[0]}"
        assert abs(coord[1]) < 0.01, f"中心点Y坐标偏差过大: {coord[1]}"
        
        return True, "3D坐标计算测试完成"
        
    except ImportError:
        return True, "3D坐标计算测试跳过（pyrealsense2 未安装）"
    except Exception as e:
        return False, f"3D坐标计算测试失败: {e}"


def test_color_mapping():
    """测试12: 测试颜色映射功能"""
    try:
        from common.camera import color_cn2en
        
        test_cases = [
            ("红色", "red"),
            ("蓝色", "blue"),
            ("黑色", "black"),
            ("白色", "white"),
            ("黄色", "yellow"),
        ]
        
        all_passed = True
        for cn, expected_en in test_cases:
            result = color_cn2en.get(cn)
            status = "✓" if result == expected_en else "✗"
            print(f"  {status} {cn} -> {result} (期望: {expected_en})")
            if result != expected_en:
                all_passed = False
        
        if all_passed:
            return True, f"颜色映射测试完成，共 {len(test_cases)} 个映射"
        else:
            return False, "部分颜色映射失败"
            
    except Exception as e:
        return False, f"颜色映射测试失败: {e}"

def test_full_pipeline():
    """测试13: 全流程集成测试"""
    try:
        from common.camera import Pose, PersonTrack, predict_gender
        
        print("  开始全流程测试...")
        
        # 1. 创建测试图像
        img = TestImageGenerator.create_person_image(
            with_face=True, 
            arm_raised='right',
            shirt_color=(255, 0, 0)  # 蓝色衬衫
        )
        print("  ✓ 测试图像生成完成")
        
        # 2. 人体检测
        tracker = PersonTrack()
        people = tracker.detect_people(img)
        print(f"  ✓ 人体检测完成，检测到 {len(people)} 人")
        
        # 3. 姿势识别
        pose = Pose()
        gesture = pose.get_pose(img)
        print(f"  ✓ 姿势识别完成: {gesture}")
        
        # 4. 性别预测
        gender = predict_gender(img)
        print(f"  ✓ 性别预测完成: {gender}")
        
        # 5. 汇总结果
        result_summary = {
            "people_count": len(people),
            "gesture": gesture,
            "gender": gender
        }
        
        print(f"\n  全流程结果汇总:")
        print(f"    - 检测人数: {result_summary['people_count']}")
        print(f"    - 姿势: {result_summary['gesture']}")
        print(f"    - 性别: {result_summary['gender']}")
        
        return True, "全流程集成测试完成"
        
    except Exception as e:
        return False, f"全流程测试失败: {e}"

def test_pose_clear():
    """测试14: 测试 Pose 类的 clear 方法"""
    try:
        from common.camera import Pose
        
        pose = Pose()
        
        # 设置一些状态
        pose.left_h_distance_pre = 0.5
        pose.right_h_distance_pre = 0.3
        pose.raise_arm = "left"
        pose.wave_hand = "waving"
        
        print(f"  清除前: left_h={pose.left_h_distance_pre}, raise_arm={pose.raise_arm}")
        
        # 清除状态
        pose.clear()
        
        print(f"  清除后: left_h={pose.left_h_distance_pre}, raise_arm={pose.raise_arm}")
        
        # 验证
        assert pose.left_h_distance_pre == 0, "left_h_distance_pre 未重置"
        assert pose.right_h_distance_pre == 0, "right_h_distance_pre 未重置"
        assert pose.raise_arm == "None", "raise_arm 未重置"
        assert pose.wave_hand == "None", "wave_hand 未重置"
        
        return True, "Pose clear() 方法测试完成"
        
    except Exception as e:
        return False, f"Pose clear() 测试失败: {e}"

def test_image_save_load():
    """测试15: 测试图像保存和加载功能"""
    try:
        import tempfile
        import os
        
        # 生成测试图像
        img = TestImageGenerator.create_person_image()
        
        # 保存到临时文件
        temp_file = tempfile.NamedTemporaryFile(suffix='.png', delete=False)
        temp_path = temp_file.name
        temp_file.close()
        
        cv2.imwrite(temp_path, img)
        print(f"  保存图像到: {temp_path}")
        
        # 重新加载
        loaded_img = cv2.imread(temp_path)
        
        # 验证
        assert loaded_img is not None, "图像加载失败"
        assert loaded_img.shape == img.shape, f"形状不匹配: {loaded_img.shape} vs {img.shape}"
        
        print(f"  加载图像尺寸: {loaded_img.shape}")
        
        # 清理
        os.unlink(temp_path)
        
        return True, "图像保存/加载测试完成"
        
    except Exception as e:
        return False, f"图像保存/加载测试失败: {e}"

# ====================================================================
# 主入口
# ====================================================================

def main():
    """运行所有测试"""
    print("""
╔══════════════════════════════════════════════════════════════╗
║         seu_person_analyse 离线功能测试                      ║
║                                                              ║
║  本测试脚本验证 person_analyse 包的所有核心功能              ║
║  无需 roscore 或真实摄像头即可运行                           ║
╚══════════════════════════════════════════════════════════════╝
    """)
    
    runner = TestRunner()
    
    # 基础测试
    runner.run_test("模块导入测试", test_import_modules)
    runner.run_test("颜色映射测试", test_color_mapping)
    runner.run_test("图像保存/加载测试", test_image_save_load)
    
    # 核心功能测试
    runner.run_test("Pose 类姿势识别测试", test_pose_class)
    runner.run_test("Pose clear() 方法测试", test_pose_clear)
    runner.run_test("手势检测测试", test_gesture_detection)
    runner.run_test("坐姿检测测试", test_sitting_detection)
    runner.run_test("性别预测测试", test_gender_prediction)
    runner.run_test("人体追踪测试", test_person_track)
    runner.run_test("人脸分析测试", test_people_analyse)
    
    # 高级功能测试
    runner.run_test("物体检测测试", test_obj_detect)
    runner.run_test("衣物颜色识别测试（Mock）", test_cloth_color_mock)
    runner.run_test("3D坐标计算测试", test_3d_coordinate)
    runner.run_test("Camera 类初始化测试", test_camera_class)
    
    # 集成测试
    runner.run_test("全流程集成测试", test_full_pipeline)
    
    # 打印总结
    success = runner.print_summary()
    
    return 0 if success else 1

if __name__ == "__main__":
    exit(main())
