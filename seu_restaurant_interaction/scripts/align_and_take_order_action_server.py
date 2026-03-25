#!/home/duo/anaconda3/envs/gpsr_perception310/bin/python
# -*- coding: utf-8 -*-
"""餐厅题桌边微调与接单 action server。"""

import os
import sys

import actionlib
import rospy
import tf2_ros

from geometry_msgs.msg import PoseStamped, Twist
from seu_restaurant_msgs.msg import (
    AlignAndTakeOrderAction,
    AlignAndTakeOrderResult,
    ParseRestaurantOrderAction,
    ParseRestaurantOrderGoal,
)
from seu_speech_recognition.srv import seu_speech_recognition as SpeechSrv
from seu_speech_recognition.srv import seu_speech_recognitionRequest
from seu_voice.srv import seu_voice as VoiceSrv
from seu_voice.srv import seu_voiceRequest

# 这个入口脚本依赖同目录的 `person_align_core.py`。
# 这里主动补上当前脚本目录，保证直接运行 scripts 文件时导入稳定。
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from person_align_core import classify_confirmation_text, select_nearest_person_within_range


ACTION_NAME = "align_and_take_order"
ORDER_PARSER_ACTION_NAME = "parse_restaurant_order"
MAP_FRAME = "map"
ROBOT_BASE_FRAME = "base_link"
CMD_VEL_TOPIC = "/cmd_vel"
VOICE_SERVICE_NAME = "voice_service"
SPEECH_SERVICE_NAME = "speech_recognition_service"
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models", "yolo26m-pose.pt")

FRAME_WIDTH = 848
FRAME_HEIGHT = 480
FRAME_FPS = 30
POSE_IMGSZ = 960
POSE_CONF = 0.35
POSE_IOU = 0.45
KEYPOINT_VISIBILITY = 0.35
DEPTH_CENTER_BOX_Y_RATIO = 0.55
DEPTH_SAMPLE_RADIUS = 6
MAX_PERSON_DISTANCE_M = 3
TARGET_SERVICE_DISTANCE_M = 2
CENTER_TOLERANCE_PX = 40
DISTANCE_TOLERANCE_M = 0.15
ANGULAR_SPEED = 0.18
LINEAR_SPEED = 0.08

KEYPOINT_INDEX = {
    "left_shoulder": 5,
    "right_shoulder": 6,
    "left_hip": 11,
    "right_hip": 12,
}


def clamp_box(x1, y1, x2, y2, width, height):
    """把检测框约束到图像内部。"""
    x1 = max(0, min(int(x1), width - 1))
    y1 = max(0, min(int(y1), height - 1))
    x2 = max(0, min(int(x2), width - 1))
    y2 = max(0, min(int(y2), height - 1))
    if x2 <= x1:
        x2 = min(width - 1, x1 + 1)
    if y2 <= y1:
        y2 = min(height - 1, y1 + 1)
    return x1, y1, x2, y2


def sample_depth_median(depth_frame, center_x, center_y, radius=DEPTH_SAMPLE_RADIUS):
    """对深度窗口取中值。"""
    profile = depth_frame.get_profile().as_video_stream_profile()
    width = profile.width()
    height = profile.height()
    center_x = int(center_x)
    center_y = int(center_y)
    values = []
    for pixel_y in range(max(0, center_y - radius), min(height, center_y + radius + 1)):
        for pixel_x in range(max(0, center_x - radius), min(width, center_x + radius + 1)):
            depth_value = depth_frame.get_distance(pixel_x, pixel_y)
            if depth_value > 0.0:
                values.append(depth_value)
    if not values:
        return 0.0
    values.sort()
    return float(values[len(values) // 2])


def transform_to_pose_stamped(transform_message):
    """把 TF 变换提取成 PoseStamped。"""
    pose = PoseStamped()
    pose.header.stamp = transform_message.header.stamp
    pose.header.frame_id = transform_message.header.frame_id
    pose.pose.position.x = transform_message.transform.translation.x
    pose.pose.position.y = transform_message.transform.translation.y
    pose.pose.position.z = transform_message.transform.translation.z
    pose.pose.orientation = transform_message.transform.rotation
    return pose


class LocalNearestPersonDetector:
    """只负责在桌边找最近的人，不做挥手判断。"""

    def __init__(self, model_path=MODEL_PATH):
        import pyrealsense2 as rs
        import torch
        from ultralytics import YOLO

        self.rs = rs
        self.pose_device = 0 if torch.cuda.is_available() else "cpu"
        if not os.path.exists(model_path):
            raise FileNotFoundError("interaction model not found: {}".format(model_path))

        self.pose_model = YOLO(model_path)
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, FRAME_WIDTH, FRAME_HEIGHT, rs.format.z16, FRAME_FPS)
        config.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.bgr8, FRAME_FPS)
        self.align_to_color = rs.align(rs.stream.color)
        self.pipeline.start(config)

    def close(self):
        """关闭相机。"""
        try:
            self.pipeline.stop()
        except Exception:
            pass

    def process_once(self):
        """处理一帧并返回 2m 内最近的人。"""
        import numpy as np

        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align_to_color.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return None

        color_image = np.asanyarray(color_frame.get_data())
        pose_result = self.pose_model.predict(
            color_image,
            verbose=False,
            imgsz=POSE_IMGSZ,
            conf=POSE_CONF,
            iou=POSE_IOU,
            classes=[0],
            device=self.pose_device,
        )[0]

        candidates = []
        if pose_result.boxes is None or pose_result.keypoints is None:
            return None

        boxes_xyxy = pose_result.boxes.xyxy.detach().cpu().numpy()
        keypoints_xy = pose_result.keypoints.xy.detach().cpu().numpy()
        if pose_result.keypoints.conf is None:
            keypoints_conf = np.ones((len(boxes_xyxy), keypoints_xy.shape[1]), dtype="float32")
        else:
            keypoints_conf = pose_result.keypoints.conf.detach().cpu().numpy()

        for det_index in range(len(boxes_xyxy)):
            x1, y1, x2, y2 = boxes_xyxy[det_index]
            person_box = clamp_box(x1, y1, x2, y2, FRAME_WIDTH, FRAME_HEIGHT)
            if (person_box[2] - person_box[0]) < 50 or (person_box[3] - person_box[1]) < 120:
                continue

            point_map = {}
            for point_name, point_index in KEYPOINT_INDEX.items():
                point_x, point_y = keypoints_xy[det_index, point_index]
                point_conf = float(keypoints_conf[det_index, point_index])
                point_map[point_name] = (int(point_x), int(point_y), point_conf)

            depth_center_x, depth_center_y = self._compute_depth_center(person_box, point_map)
            candidates.append(
                {
                    "depth_m": sample_depth_median(depth_frame, depth_center_x, depth_center_y),
                    "center_x": int((person_box[0] + person_box[2]) / 2),
                    "box": person_box,
                }
            )

        return select_nearest_person_within_range(candidates, MAX_PERSON_DISTANCE_M)

    @staticmethod
    def _compute_depth_center(person_box, point_map):
        """优先使用躯干区域作为深度采样点。"""
        left_shoulder = point_map["left_shoulder"]
        right_shoulder = point_map["right_shoulder"]
        left_hip = point_map["left_hip"]
        right_hip = point_map["right_hip"]

        shoulder_points = [point for point in (left_shoulder, right_shoulder) if point[2] > KEYPOINT_VISIBILITY]
        hip_points = [point for point in (left_hip, right_hip) if point[2] > KEYPOINT_VISIBILITY]
        box_center_x = (person_box[0] + person_box[2]) // 2
        box_center_y = int(person_box[1] + (person_box[3] - person_box[1]) * DEPTH_CENTER_BOX_Y_RATIO)

        if len(shoulder_points) == 2 and len(hip_points) == 2:
            shoulder_center_x = sum(point[0] for point in shoulder_points) / 2.0
            shoulder_center_y = sum(point[1] for point in shoulder_points) / 2.0
            hip_center_x = sum(point[0] for point in hip_points) / 2.0
            hip_center_y = sum(point[1] for point in hip_points) / 2.0
            return int((shoulder_center_x + hip_center_x) / 2.0), int((shoulder_center_y + hip_center_y) / 2.0)

        return box_center_x, box_center_y


class AlignAndTakeOrderActionServer:
    """桌边微调与接单动作服务端。"""

    def __init__(self):
        self.current_pose = None
        self.voice_client = None
        self.speech_client = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.cmd_vel_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)

        self.server = actionlib.SimpleActionServer(
            ACTION_NAME,
            AlignAndTakeOrderAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.server.start()
        rospy.loginfo("[restaurant_interaction] action ready: %s", ACTION_NAME)

    def _get_current_pose(self):
        """直接从 TF 读取 map -> base_link 当前位姿。"""
        transform_message = self.tf_buffer.lookup_transform(
            MAP_FRAME,
            ROBOT_BASE_FRAME,
            rospy.Time(0),
            rospy.Duration(2.0),
        )
        self.current_pose = transform_to_pose_stamped(transform_message)
        return self.current_pose

    def _stop_robot(self):
        """停车。"""
        self.cmd_vel_pub.publish(Twist())

    def _get_voice_client(self):
        """延迟创建 voice service proxy。"""
        if self.voice_client is None:
            rospy.wait_for_service(VOICE_SERVICE_NAME, timeout=2.0)
            self.voice_client = rospy.ServiceProxy(VOICE_SERVICE_NAME, VoiceSrv)
        return self.voice_client

    def _get_speech_client(self):
        """延迟创建 speech service proxy。"""
        if self.speech_client is None:
            rospy.wait_for_service(SPEECH_SERVICE_NAME, timeout=2.0)
            self.speech_client = rospy.ServiceProxy(SPEECH_SERVICE_NAME, SpeechSrv)
        return self.speech_client

    def _say(self, text):
        """调用现有 TTS 服务播报一句话。"""
        response = self._get_voice_client().call(seu_voiceRequest(text=text))
        if not response.state:
            raise RuntimeError("voice service failed: {}".format(response.errormsg))

    def _listen_once(self):
        """调用现有语音识别服务录一句完整语音。"""
        request = seu_speech_recognitionRequest(enable=False, task="restaurant")
        response = self._get_speech_client().call(request)
        if not response.state:
            return ""
        return (response.word or "").strip().lower()

    def _align_to_customer(self, timeout_sec):
        """只做简单的旋转对正和前后微调。"""
        detector = LocalNearestPersonDetector()
        try:
            rate = rospy.Rate(10)
            start_time = rospy.Time.now()
            while not rospy.is_shutdown():
                elapsed = (rospy.Time.now() - start_time).to_sec()
                if elapsed > timeout_sec:
                    return False
                if self.server.is_preempt_requested():
                    self._stop_robot()
                    self.server.set_preempted()
                    return False

                candidate = detector.process_once()
                if candidate is None:
                    # 看不到人时保持原地搜索，直到重新进入有效检测范围。
                    twist = Twist()
                    twist.angular.z = ANGULAR_SPEED
                    self.cmd_vel_pub.publish(twist)
                    rate.sleep()
                    continue

                center_error = candidate["center_x"] - (FRAME_WIDTH // 2)
                depth_error = candidate["depth_m"] - TARGET_SERVICE_DISTANCE_M
                aligned = abs(center_error) <= CENTER_TOLERANCE_PX and abs(depth_error) <= DISTANCE_TOLERANCE_M
                if aligned:
                    self._stop_robot()
                    return True

                twist = Twist()
                if abs(center_error) > CENTER_TOLERANCE_PX:
                    twist.angular.z = -ANGULAR_SPEED if center_error > 0 else ANGULAR_SPEED
                elif depth_error > DISTANCE_TOLERANCE_M:
                    twist.linear.x = LINEAR_SPEED
                elif depth_error < -DISTANCE_TOLERANCE_M:
                    twist.linear.x = -LINEAR_SPEED
                self.cmd_vel_pub.publish(twist)
                rate.sleep()
            return False
        finally:
            self._stop_robot()
            detector.close()

    def execute_cb(self, goal):
        """执行局部对正 + 接单 + yes/no 确认 + LLM 订单解析。"""
        result = AlignAndTakeOrderResult()
        result.success = False
        result.error_msg = ""
        result.order_text = ""
        result.orderlist = []
        result.rounds_used = 0

        try:
            self.current_pose = self._get_current_pose()

            aligned = self._align_to_customer(goal.align_timeout_sec)
            if not aligned:
                if self.server.is_preempt_requested():
                    return
                result.error_msg = "failed to align to the nearest customer within timeout"
                self.server.set_aborted(result, result.error_msg)
                return

            # 订单 LLM 解析也走 actionlib，保持和其他餐厅节点一致的通信方式。
            llm_client = actionlib.SimpleActionClient(ORDER_PARSER_ACTION_NAME, ParseRestaurantOrderAction)
            if not llm_client.wait_for_server(rospy.Duration(5)):
                result.error_msg = "llm parse failed or network error: order parser action unavailable"
                self.server.set_aborted(result, result.error_msg)
                return

            for round_index in range(max(1, int(goal.max_rounds))):
                self._say(goal.ask_text)
                order_text = self._listen_once()
                if not order_text:
                    continue

                confirm_prompt = "I heard {}. Is that correct? Please answer yes or no.".format(order_text)
                self._say(confirm_prompt)
                confirm_text = self._listen_once()
                confirm_state = classify_confirmation_text(confirm_text)
                if confirm_state != "yes":
                    continue

                # 顾客确认原句后，再交给 LLM 做订单物品抽取。
                llm_goal = ParseRestaurantOrderGoal()
                llm_goal.order_text = order_text
                llm_client.send_goal(llm_goal)
                llm_client.wait_for_result()
                llm_result = llm_client.get_result()

                # no_info 代表这句原始语音本身没有可用订单信息，直接回到重新询问流程。
                if llm_result is not None and not llm_result.success and llm_result.error_msg == "no_info":
                    continue

                # 除 no_info 外，其余失败都按 LLM/网络错误处理，当前 interaction 直接失败退出。
                if llm_result is None:
                    result.error_msg = "llm parse failed or network error: empty llm result"
                    self.server.set_aborted(result, result.error_msg)
                    return
                if not llm_result.success:
                    llm_error_msg = llm_result.error_msg or "unknown llm error"
                    result.error_msg = "llm parse failed or network error: {}".format(llm_error_msg)
                    self.server.set_aborted(result, result.error_msg)
                    return

                self.current_pose = self._get_current_pose()
                result.success = True
                result.order_text = order_text
                result.orderlist = list(llm_result.items)
                result.rounds_used = round_index + 1
                result.service_pose_map = self.current_pose
                self.server.set_succeeded(result)
                return

            result.error_msg = "failed to confirm the order within max rounds"
            self.server.set_aborted(result, result.error_msg)
        except Exception as exc:
            self._stop_robot()
            result.error_msg = "align and take order failed: {}".format(exc)
            self.server.set_aborted(result, result.error_msg)
        finally:
            self._stop_robot()


def main():
    """节点入口。"""
    rospy.init_node("align_and_take_order_action_server")
    AlignAndTakeOrderActionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
