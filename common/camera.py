#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import base64
import os
import time

# 动态获取当前脚本所在目录作为基础路径
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_COMMON_DIR = _SCRIPT_DIR  # common 目录
_SRC_DIR = os.path.dirname(_SCRIPT_DIR)  # src 目录

import cv2
import numpy as np
import requests
from deepface import DeepFace
from scipy.spatial.distance import cosine
import pyrealsense2 as rs
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import face_recognition
from ultralytics import YOLO
from collections import deque
color_cn2en = {
    "蓝": "blue",
    "黑": "black",
    "白": "white",
    "红": "red",
    "橙": "orange",
    "灰": "gray",
    "蓝色": "blue",
    "红色": "red",
    "黑色": "black",
    "白色": "white",
    "橙色": "orange",
    "灰色": "gray",
    "黄色": "yellow"
}


# import rospy
# import tf2_ros
# import tf2_geometry_msgs
# from geometry_msgs.msg import PointStamped
#
#
# def transform_camera_to_map(x, y, z,
#                             source_frame="camera1_color_optical_frame",
#                             target_frame="map",
#                             timeout=1.0):
#     """
#     将相机坐标系下的点 (x, y, z) 转换到全局 map 坐标系
#     参数：
#         x, y, z           - 相机坐标系中的三维点坐标
#         source_frame      - 源坐标系（默认是 RealSense 相机的光学坐标系）
#         target_frame      - 目标坐标系（默认是全局 map）
#         timeout           - 变换等待时间（秒）
#     返回：
#         转换后的 (x, y, z)，如果失败返回 None
#     """
#     try:
#         buffer = tf2_ros.Buffer()
#         listener = tf2_ros.TransformListener(buffer)
#         # 构造带有 frame 的点
#         ps = PointStamped()
#         ps.header.stamp = rospy.Time.now()
#         ps.header.frame_id = source_frame
#         ps.point.x = x
#         ps.point.y = y
#         ps.point.z = z
#         # 等待变换关系
#         buffer.can_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(timeout))
#         # 执行变换
#         ps_out = buffer.transform(ps, target_frame, rospy.Duration(timeout))
#         return ps_out.point.x, ps_out.point.y, ps_out.point.z
#     except Exception as e:
#         rospy.logwarn("坐标变换失败：%s", str(e))
#         return None

def predict_gender(face_img_rgb):
    try:
        analysis = DeepFace.analyze(img_path=face_img_rgb, actions=["gender"], enforce_detection=False)
        gender = analysis[0]['dominant_gender'] if isinstance(analysis, list) else analysis['dominant_gender']
        return "woman" if gender.lower() == "woman" else "man"
    except Exception as e:
        print(f"⚠️ 性别识别失败: {e}")
        return "unknown"


def get_3d_camera_coordinate(x, y, aligned_depth_frame, depth_intrin):
    """计算3D相机坐标"""
    pix_x = int(x)
    pix_y = int(y)
    
    # 边界检查，防止越界导致 RuntimeError: out of range value for argument "x"
    h, w = 480, 640  # 默认分辨率
    if hasattr(aligned_depth_frame, 'get_profile'):
        try:
            profile = aligned_depth_frame.get_profile().as_video_stream_profile()
            w, h = profile.width(), profile.height()
        except:
            pass
            
    if pix_x < 0 or pix_x >= w or pix_y < 0 or pix_y >= h:
        return 0.0, [0.0, 0.0, 0.0]

    dis = aligned_depth_frame.get_distance(pix_x, pix_y)
    camera_coordinate = rs.rs2_deproject_pixel_to_point(
        depth_intrin, [pix_x, pix_y], dis
    )
    return dis, camera_coordinate

class ObjDetect:
    def __init__(self):
        self.model = YOLO(os.path.join(_COMMON_DIR, "models/epoch10.pt"), verbose=False)

    def get_obj(self,image,depth_frame,depth_intrin,imgsz = 640,conf = 0.1):
        res = []
        results = self.model.predict(image, imgsz=imgsz, conf=conf,iou=0.45,augment=True, verbose=False)
        if results and hasattr(results[0], 'boxes') and len(results[0].boxes) > 0:
            for box in results[0].boxes:
                # 获取当前检测框对应的类别ID，并转换为标签名称
                cls_id = int(box.cls[0])
                detected_label = self.model.names.get(cls_id, f"ID:{cls_id}")
                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                pos = get_3d_camera_coordinate((x1+x2)//2, (y1+y2)//2, depth_frame, depth_intrin)
                confidence = float(box.conf[0].cpu().numpy())
                res.append({"label":detected_label,
                            "box":(x1,y1,x2,y2),
                            "pos":pos[1],
                            "cof":confidence
                            })
        return res
    def draw_box(self,img,results):
        for res in results:
            detected_label = res['label']
            x1, y1, x2, y2 = res['box']
            pos = res['pos']
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(img, f"Label:{detected_label} X:{pos[0]:.2f} Y:{pos[1]:.2f} Z:{pos[2]:.2f}",
                        ((x1 + x2) // 2 + 10, (y1 + y2) // 2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 2)



class PersonTrack:
    def __init__(self, model_path=None):
        if model_path is None:
            model_path = os.path.join(_COMMON_DIR, 'models/yolov8n.pt')
        self.detector = YOLO(model_path, verbose=False)
        self.pose = mp.solutions.pose.Pose(static_image_mode=True)
        self.mp_pose = mp.solutions.pose

    def detect_people(self, image, depth_frame=None, depth_intrin=None):
        """使用YOLOv8检测所有人"""
        results = self.detector(image, verbose=False)
        boxes = []
        for r in results:
            for box, cls in zip(r.boxes.xyxy, r.boxes.cls):
                if int(cls) == 0:  # class 0 == 'person'
                    x1, y1, x2, y2 = map(int, box)
                    pos = None
                    if depth_frame is not None:
                        left, right = self.extract_pose_from_box(image, (x1, y1, x2, y2))
                        if left[0] is not None:
                            center = ((left[0] + right[0]) // 2, (left[1] + right[1]) // 2)
                            cv2.circle(image, (center[0], center[1]), 5, (0, 0, 255), -1)
                            pos = get_3d_camera_coordinate(center[0], center[1], depth_frame, depth_intrin)
                    boxes.append((x1, y1, x2, y2, pos))
        return boxes

    def extract_pose_from_box(self, image, box=(0,0,640,480)):
        """在每个人框中运行Pose，返回肩膀坐标"""
        x1, y1, x2, y2 = box
        cropped = image[y1:y2, x1:x2]
        img_rgb = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)
        results = self.pose.process(img_rgb)

        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark
            left_shoulder = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
            right_shoulder = landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]

            # 映射回原图
            lx = int(left_shoulder.x * (x2 - x1)) + x1
            ly = int(left_shoulder.y * (y2 - y1)) + y1
            rx = int(right_shoulder.x * (x2 - x1)) + x1
            ry = int(right_shoulder.y * (y2 - y1)) + y1
            return (lx, ly), (rx, ry)
        return (None, None),(None, None)


class People_analyse:
    def __init__(self, known_path="known_faces"):
        self.known_path = known_path
        self.known_faces = {}  # id -> embedding
        self.record_faces = {}  # id -> embedding

        os.makedirs(self.known_path, exist_ok=True)
        self._load_database(self.known_path, self.known_faces)

    def _load_database(self, path, storage_dict):
        """加载文件夹下所有人脸图像及其特征"""
        for filename in os.listdir(path):
            if filename.lower().endswith((".jpg", ".png", ".jpeg")):
                img_path = os.path.join(path, filename)
                try:
                    embedding = self.get_embedding_from_path(img_path)
                    if embedding is not None:
                        face_id = os.path.splitext(filename)[0]
                        storage_dict[face_id] = embedding
                except Exception as e:
                    print(f"❌ 加载失败 {filename}: {e}")

    def get_embedding_from_path(self, img_path):
        """使用 face_recognition 获取单个人脸的 128D 特征"""
        image = face_recognition.load_image_file(img_path)
        encodings = face_recognition.face_encodings(image)
        return encodings[0].tolist() if encodings else None

    def get_embedding_from_iamge(self, image):
        img_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        face_locations = face_recognition.face_locations(img_rgb, model='vog')  # or cnn
        face_encodings = face_recognition.face_encodings(img_rgb, face_locations)
        for (top, right, bottom, left), encoding in zip(face_locations, face_encodings):
            return (right, top, left, bottom), encoding.tolist()
        return (None, None, None, None), None

    def get_embedding_from_iamge2(self, image, box):
        img_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        face_encodings = face_recognition.face_encodings(img_rgb, box)
        print(len(face_encodings))
        for encoding in face_encodings:
            return encoding.tolist()
        return None

    def analyze_faces(self, color_image):
        """使用 face_recognition 检测人脸 + DeepFace 分析性别"""
        img_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        face_locations = face_recognition.face_locations(img_rgb, model='vog')  # or cnn
        face_encodings = face_recognition.face_encodings(img_rgb, face_locations)

        results = []
        for (top, right, bottom, left), encoding in zip(face_locations, face_encodings):
            face_img = img_rgb[top:bottom, left:right]
            gender = predict_gender(face_img)

            results.append({
                "face_position": (left, top, right - left, bottom - top),
                "gender": gender,
                "embedding": encoding.tolist()  # 保持兼容
            })

        return results

    def check_if_recorded(self, face_embedding, threshold=0.05):
        """使用 cosine 距离比较 face_recognition 提取的 embedding"""
        for face_id, known_embedding in self.known_faces.items():
            if cosine(face_embedding, known_embedding) < threshold:
                return face_id
        for face_id, new_embedding in self.record_faces.items():
            print(face_id, ":", cosine(face_embedding, new_embedding))
            if cosine(face_embedding, new_embedding) < threshold:
                return face_id
        return None

    def record_face(self, face_embedding):
        """若新脸，存入 new_faces，返回新 ID；若为已知脸，返回原 ID"""
        existing_id = self.check_if_recorded(face_embedding)
        if existing_id:
            return existing_id  # 已知人脸

        # 为新脸生成唯一 ID
        new_id = f"person_{len(self.record_faces) + 1:03d}"
        self.record_faces[new_id] = face_embedding
        return new_id


class Pose:
    def __init__(self, image_width=640, image_height=480):
        """初始化Pose识别类"""
        self.image_width = image_width
        self.image_height = image_height
        self.mp_pose = mp.solutions.pose
        self.mp_hands = mp.solutions.hands.Hands(
            static_image_mode=True,
            max_num_hands=2,
            min_detection_confidence=0.5
        )
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.left_h_distance_pre = 0
        self.right_h_distance_pre = 0
        self.raise_arm = "None"
        self.wave_hand = "None"

    def detect_gesture(self, color_image, draw=False, width=640, height=480):
        """检测手势并区分左右手"""
        img = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        hands_results = self.mp_hands.process(img)
        point_pos = None

        if hands_results.multi_hand_landmarks and hands_results.multi_handedness:
            for hand_landmarks, hand_handedness in zip(hands_results.multi_hand_landmarks,
                                                       hands_results.multi_handedness):
                # 获取左右手标签
                hand_label = hand_handedness.classification[0].label  # 'Left' or 'Right'
                if hand_label == 'Left':
                    hand_label = 'Right'
                else:
                    hand_label = 'Left'
                # 获取关键点
                index_finger_mcp = hand_landmarks.landmark[5]
                index_finger_tip = hand_landmarks.landmark[8]
                middle_finger_tip = hand_landmarks.landmark[12]
                ring_finger_tip = hand_landmarks.landmark[16]
                pinky_tip = hand_landmarks.landmark[20]

                # 选择颜色：右手绿色，左手蓝色
                color = (0, 255, 0) if hand_label == 'Right' else (255, 255, 0)
                if draw:
                    # 绘制关键点
                    for landmark in [index_finger_mcp, index_finger_tip, middle_finger_tip, ring_finger_tip, pinky_tip]:
                        x, y = int(landmark.x * width), int(landmark.y * height)
                        cv2.circle(color_image, (x, y), 5, color, -1)

                # 简单判断食指是否水平伸出，并判断方向
                if 0 < index_finger_tip.x < 1:
                    if abs(index_finger_mcp.y - index_finger_tip.y) < 0.1:
                        if index_finger_mcp.x > index_finger_tip.x:
                            point_pos = 'right'
                        elif index_finger_mcp.x < index_finger_tip.x:
                            point_pos = 'left'
                # print("检测到的是：", hand_label, "指向", point_pos)
        return point_pos  # 你可以根据需要返回左右手标识也一起返回

    def detect_sitting_pose(self, color_image, draw=False, width=640, height=480):
        """检测坐姿并更新手势状态"""
        img = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        pose_results = self.pose.process(img)
        raise_arm = None
        wave_hand = None
        is_sitting = False
        if pose_results.pose_landmarks:
            landmarks = pose_results.pose_landmarks.landmark
            # --- 更新手势状态 ---
            raise_arm, wave_hand = self._update_arm_status_from_pose(landmarks)

            # 坐姿检测逻辑
            is_sitting = (
                    abs(landmarks[self.mp_pose.PoseLandmark.LEFT_KNEE].y -
                        landmarks[self.mp_pose.PoseLandmark.LEFT_HIP].y) < 0.3 and
                    abs(landmarks[self.mp_pose.PoseLandmark.RIGHT_KNEE].y -
                        landmarks[self.mp_pose.PoseLandmark.RIGHT_HIP].y) < 0.3
            )
            if draw:
                # 绘制关键点
                for landmark in landmarks:
                    x, y = int(landmark.x * width), int(landmark.y * height)
                    cv2.circle(color_image, (x, y), 5, (255, 255, 255), -1)

        return is_sitting, raise_arm, wave_hand

    def _update_arm_status_from_pose(self, landmarks):
        """使用姿势关键点更新举手/挥手状态"""
        left_shoulder = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
        right_shoulder = landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]
        left_wrist = landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST]
        right_wrist = landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST]

        left_v = left_wrist.y - left_shoulder.y
        right_v = right_wrist.y - right_shoulder.y
        left_h = left_wrist.x - left_shoulder.x
        right_h = right_wrist.x - right_shoulder.x

        raise_arm = None
        # 判断举手
        if left_v < -0.05 and right_v < -0.05:
            if left_v < right_v:
                raise_arm = "left"
            else:
                raise_arm = "right"
        elif left_v < -0.05:
            raise_arm = "left"
        elif right_v < -0.05:
            raise_arm = "right"

        # 判断挥手（相对于上次位置差异）
        wave_threshold = 0.15  # 可调节
        if abs(left_h - self.left_h_distance_pre) > wave_threshold or abs(
                right_h - self.right_h_distance_pre) > wave_threshold:
            wave_hand = "waving"
        else:
            wave_hand = None

        # 保存本次位置用于下一帧比较
        self.left_h_distance_pre = left_h
        self.right_h_distance_pre = right_h
        return raise_arm, wave_hand

    def clear(self):
        self.left_h_distance_pre = 0
        self.right_h_distance_pre = 0
        self.raise_arm = "None"
        self.wave_hand = "None"

    def get_pose(self, color_image, feature=None, width=640, height=480):
        point_pos = self.detect_gesture(color_image, True, width=width, height=height)
        is_sitting, raise_arm, wave_hand = self.detect_sitting_pose(color_image, True, width=width, height=height)
        if feature is None:
            if point_pos is not None:
                return f"pointing {point_pos}"
            elif wave_hand is not None:
                return f"waving"
            elif raise_arm is not None:
                return f"raising {raise_arm} arm"
            # elif is_sitting:
            #     return f"sitting"
            else:
                return f"none"
        elif feature == "pointing":
            if point_pos is not None:
                return f"pointing {point_pos}"
        elif feature == "raising":
            if wave_hand is not None:
                return f"waving"
            elif raise_arm is not None:
                return f"raising {raise_arm} arm"
        elif feature == "sitting":
            if is_sitting:
                return f"sitting"
            else:
                return f"standing"
        return "none"


class Cloth:
    def __init__(self):
        self.API_KEY = "qdHHrNUVxPT5epB9tkMyg3ma"
        self.SECRET_KEY = "ALmrJxY0iEotVYzDo8IcWX3iFGrvQxjZ"

    def get_access_token(self):
        url = "https://aip.baidubce.com/oauth/2.0/token"
        params = {
            "grant_type": "client_credentials",
            "client_id": self.API_KEY,
            "client_secret": self.SECRET_KEY
        }
        response = requests.post(url, params=params)
        if response.status_code == 200:
            return response.json().get("access_token")
        else:
            print("获取access_token失败:", response.text)
            return None

    def get_file_content_as_base64(self, path):
        with open(path, "rb") as f:
            return base64.b64encode(f.read()).decode("utf-8")

    def bodyanalysis(self, img_input):
        """分析图像中的人体属性，支持传入路径或 BGR 图像"""
        access_token = self.get_access_token()
        if not access_token:
            return

        # 判断输入类型：str 表示路径，ndarray 表示 BGR 图像
        if isinstance(img_input, str):
            # 读取并转换为 base64
            with open(img_input, "rb") as f:
                image_base64 = base64.b64encode(f.read()).decode('utf-8')
        elif isinstance(img_input, np.ndarray):
            # OpenCV 图像，先编码为 JPEG
            success, buffer = cv2.imencode('.jpg', img_input)
            if not success:
                print("图像编码失败")
                return
            image_base64 = base64.b64encode(buffer).decode('utf-8')
        else:
            print("不支持的图像输入类型，请传入文件路径或BGR图像")
            return

        url = f"https://aip.baidubce.com/rest/2.0/image-classify/v1/body_attr?access_token={access_token}"
        params = {
            "image": image_base64,
            "type": "upper_color"
        }
        headers = {
            'Content-Type': 'application/x-www-form-urlencoded'
        }

        response = requests.post(url, data=params, headers=headers)
        if response.status_code == 200:
            result = response.json()
            if "person_info" in result:
                best_upper_color = None
                best_score = -1.0

                for person in result["person_info"]:
                    upper_attr = person["attributes"].get("upper_color", {})
                    name = upper_attr.get("name", "")
                    score = upper_attr.get("score", 0)

                    if name not in ["不确定", "未知"] and score > best_score:
                        best_score = score
                        best_upper_color = name

                if best_upper_color:
                    print(f"得分最高的上身颜色是: {best_upper_color}（score: {best_score:.2f}）")
                    return color_cn2en[best_upper_color]
                else:
                    print("没有找到确定的上身颜色。")
                    return None
            else:
                print("未检测到人物信息。")
                return None

        else:
            print("请求失败:", response.status_code, response.text)

        return response.json()


class Camera:
    def __init__(self, camera_type='realsense', serial_number=None):
        """初始化相机及所有必要组件"""
        self.camera_type = camera_type  # 'realsense' 或 'webcam'
        self.serial_number = serial_number
        self.real_sence_device = ['135122072963','141722074334','135122074163','135122077457']
        self.camera_state = False
        self.clicked_point_here = None
        self.image_width = 640
        self.image_height = 480
        # 人体检测
        self.people_track = PersonTrack()
        # 初始化MediaPipe组件
        self.pose = Pose(image_width=self.image_width, image_height=self.image_height)
        # 定位、特征追踪、性别识别
        self.people_analyse = People_analyse()
        # 衣物颜色识别
        self.cloth = Cloth()
        if self.camera_type == 'realsense':
            self.align = rs.align(rs.stream.color)
            self.config = rs.config()
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.pipeline = rs.pipeline()
        elif self.camera_type == 'webcam':
            self.cap = cv2.VideoCapture(0)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)

    def start_camera(self):
        if not self.camera_state:
            """启动相机"""
            if self.camera_type == 'realsense':
                if self.serial_number:
                    if self.serial_number < len(rs.context().query_devices()):
                        self.config.enable_device(
                            rs.context().query_devices()[self.serial_number].get_info(rs.camera_info.serial_number))
                    else:
                        self.config.enable_device(
                            rs.context().query_devices()[0].get_info(rs.camera_info.serial_number))
                self.profile = self.pipeline.start(self.config)
            elif self.camera_type == 'webcam':
                if not self.cap.isOpened():
                    self.cap.open(0)
            self.camera_state = True
        print(f'{self.camera_type} 相机已启动')


    def stop_camera(self):
        if self.camera_state:
            """停止相机"""
            if self.camera_type == 'realsense':
                self.pipeline.stop()
            elif self.camera_type == 'webcam':
                self.cap.release()
            cv2.destroyAllWindows()
            self.camera_state = False
        print('相机已停止')

    def stop(self):
        """stop_camera 的别名，增加兼容性"""
        self.stop_camera()

    def get_frame(self):
        """获取当前帧"""
        if not self.camera_state:
            print('相机未启动')
            return None, None, None, None
        try:
            if self.camera_type == 'realsense':
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()

                if not depth_frame or not color_frame:
                    return None, None, None, None

                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
                depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                return color_image, depth_image, depth_intrin, depth_frame

            elif self.camera_type == 'webcam':
                ret, frame = self.cap.read()
                if not ret:
                    return None, None, None, None
                return frame, None, None, None
        except Exception as e:
            print(f"读取失败: {e}")
            return None, None, None, None

    def wait_for_open_door(self):

        clicked_coord_here = None
        self.start_camera()
        time.sleep(2)

        def mouse_callback_here(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                self.clicked_point_here = (x, y)  # 保存点击坐标
        # 创建窗口并设置鼠标回调
        cv2.namedWindow('DOOR')
        cv2.setMouseCallback('DOOR', mouse_callback_here)
        print("点击窗口获取3D坐标，按'q'退出...")

        while True:
            color_image, depth_image, depth_intrin, depth_frame = self.get_frame()
            if color_image is None:
                continue

            # 获取5个点的深度值
            distances = []
            points = [(100, 100), (200, 100), (300, 100), (400, 100), (500, 100)]  # 预设5个检测点的坐标
            for x, y in points:
                distance, coord = get_3d_camera_coordinate(x, y, depth_frame, depth_intrin)
                distances.append(distance)

            # 检查是否有至少3个点的深度大于2
            count = sum(1 for d in distances if d > 3)
            if count >= 3:
                print("door opened")
                break

            # 如果有点击坐标，计算并显示3D坐标
            if self.clicked_point_here is not None:
 
                x, y = self.clicked_point_here
                distance, coord = get_3d_camera_coordinate(x, y, depth_frame, depth_intrin)
                clicked_coord_here = coord
                print(x,y)
                cv2.circle(color_image, (x, y), 5, (0, 0, 255), -1)
                cv2.putText(color_image, f"d:{distance:.2f},X:{coord[0]:.2f} Y:{coord[1]:.2f} Z:{coord[2]:.2f}",
                            (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow('DOOR', color_image)
            # 检查退出键
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # 清理资源
        self.stop_camera()
        cv2.destroyAllWindows()

    def save_iamge(self,image,name):
        cv2.imwrite(os.path.join(_COMMON_DIR, f"images/{name}.png"), image)

    def avg_image(self, image,start ,end):
        """
        读取从 image_start 到 image_end 的图像，计算并返回它们的平均值

        参数:
            image_start (int): 起始图像编号（包含）
            image_end (int): 结束图像编号（包含）

        返回:
            avg_image (numpy.ndarray): 平均后的图像
            None: 如果没有找到任何图像
        """
        images = []

        # 遍历从 start 到 end 的所有图像
        for i in range(start, end + 1):
            # 构造图像文件名（根据你的实际命名格式调整）
            image_path = os.path.join(_COMMON_DIR, f"images/{image}_{i}.png")  # 假设图像按数字编号

            # 检查文件是否存在
            if not os.path.exists(image_path):
                print(f"警告：图像 {image_path} 不存在，跳过")
                continue

            # 读取图像
            img = cv2.imread(image_path)
            if img is not None:
                images.append(img)
            else:
                print(f"警告：无法读取图像 {image_path}，跳过")

        # 如果没有找到任何图像
        if not images:
            print("错误：没有找到任何有效图像")
            return None
        # 计算平均值
        avg_image = np.mean(images, axis=0).astype(np.uint8)
        return avg_image


if __name__ == "__main__":
    clicked_point = None
    clicked_coord = None


    def mouse_callback(event, x, y, flags, param):
        global clicked_point
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked_point = (x, y)  # 保存点击坐标

    objdetect = ObjDetect()
    # 初始化相机
    #camera = Camera(camera_type='webcam')
    camera = Camera(serial_number=1)
    camera.start_camera()
    
    # 创建窗口并设置鼠标回调
    cv2.namedWindow('Live Feed')
    cv2.setMouseCallback('Live Feed', mouse_callback)

    print("点击窗口获取3D坐标，按'q'退出...")
    last_frames = deque(maxlen=5)
    while True:
        color_image, depth_image, depth_intrin, depth_frame = camera.get_frame()
        #color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        if color_image is not None:
            results = objdetect.get_obj(color_image, depth_frame, depth_intrin)
            for res in results:
                x1, y1, x2, y2 = res['box']
                label = res['label']
                pos = res['pos']
                cof = res['cof']
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(color_image, f"Label:{label} confidence:{cof:.2f}",
                            ((x1 + x2) // 2 + 10, (y1 + y2) // 2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0), 2)
            print(results)


        # 实时检测人脸
        # face_results = camera.people_analyse.analyze_faces(color_image)
        # for face in face_results:
        #     x, y, w, h = face['face_position']
        #     gender = face['gender']
        #     embedding = face['embedding']
        #
        #     # 记录或识别人脸ID
        #     face_id = camera.people_analyse.record_face(embedding)
        #
        #     # 绘制人脸框和性别信息
        #     cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        #     cv2.putText(color_image, f"{gender}({face_id})", (x, y - 10),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # face_results = camera.people_track.detect_people(color_image, depth_frame, depth_intrin)
        # for face in face_results:
        #     x1, y1, x2, y2, pos = face
        #     if pos is not None:
        #         print(pos)
        #     cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        #     cropped = color_image[y1:y2, x1:x2]
        #     # (x3, y3, x4, y4), e1 = camera.people_analyse.get_embedding_from_iamge(cropped)
        #     # if e1 is not None:
        #     #     face_id = camera.people_analyse.record_face(e1)
        #     #     cv2.rectangle(color_image, (x3, y3), (x4, y4), (0, 0, 0), 2)
        #     #     cv2.putText(color_image, f"({face_id})", (x3, y3 - 10),
        #     #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        #
        #     # 实时检测位姿
        #     # point_pos = camera.pose.detect_gesture(cropped, True, x2 - x1, y2 - y1)
        #     # is_sitting, raise_arm, wave_hand = camera.pose.detect_sitting_pose(cropped, True, x2 - x1, y2 - y1)
        #     pose = camera.pose.get_pose(color_image,  feature="pointing")
        #     print(pose)
        #     # 在图像上绘制信息
        #     # cv2.putText(color_image, f"sit: {is_sitting}", (10, 30),
        #     #             cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        #     # cv2.putText(color_image, f"point: {point_pos}", (10, 60),
        #     #             cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
        #     # cv2.putText(color_image, f"raise: {raise_arm}", (10, 90),
        #     #             cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
        #     # cv2.putText(color_image, f"wave: {wave_hand}", (10, 120),
        #     #             cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        #     cv2.putText(color_image, f"pose: {pose}", (10, 120),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        #
        # # 显示已记录不同人脸数
        # total_faces = len(camera.people_analyse.record_faces)
        # cv2.putText(color_image, f"Recorded people: {total_faces}", (10, 30),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # 如果有点击坐标，计算并显示3D坐标
        if clicked_point is not None:
            x, y = clicked_point
            distance, coord = get_3d_camera_coordinate(x, y, depth_frame, depth_intrin)
            clicked_coord = coord
            cv2.circle(color_image, (x, y), 5, (0, 0, 255), -1)
            cv2.putText(color_image, f"d:{distance:.2f},X:{coord[0]:.2f} Y:{coord[1]:.2f} Z:{coord[2]:.2f}",
                        (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


        cv2.imshow('Live Feed',color_image)

        # 检查退出键
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 清理资源
    camera.stop_camera()
    cv2.destroyAllWindows()
    print("视频流处理结束")

