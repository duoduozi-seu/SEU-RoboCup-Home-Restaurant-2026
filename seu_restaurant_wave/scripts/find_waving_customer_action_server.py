#!/home/duo/anaconda3/envs/gpsr_perception310/bin/python
# -*- coding: utf-8 -*-
"""餐厅题挥手搜索 action server。"""

import math
import os
import sys

import actionlib
import rospy
import tf2_ros

from geometry_msgs.msg import Point, PoseStamped, Twist
from sensor_msgs.msg import Image
from seu_restaurant_msgs.msg import FindWavingCustomerAction, FindWavingCustomerResult
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray

# 这个入口脚本依赖同目录的 `wave_detector_core.py`。
# 这里主动补上当前脚本目录，保证直接运行 scripts 文件时导入稳定。
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from wave_detector_core import WaveDetectorRuntime


ACTION_NAME = "find_waving_customer"
ODOM_FRAME = "odom"
ROBOT_BASE_FRAME = "base_link"
CMD_VEL_TOPIC = "/cmd_vel"
DEBUG_IMAGE_TOPIC = "/restaurant_wave/debug_image"
MARKER_TOPIC = "/restaurant_wave/target_marker"
MARKER_NS = "restaurant_wave_target"
DEFAULT_CAMERA_FRAME = "camera1_color_optical_frame"
DEFAULT_RATE_HZ = 10.0
CAMERA_BACKWARD_BIAS_M = -0.60
SAME_PERSON_DISTANCE_M = 0.40


def camera_point_to_base_link_point(camera_point):
    """把 RealSense 光学坐标系下的点转成带相机后移偏置的 base_link 坐标。

    这里不再依赖相机 TF，只做最小轴变换，并补一个相机沿机身后方 60cm 的偏置：
    1. optical z(前) -> base_link x(前)
    2. optical x(右) -> base_link y(左) 的负方向
    3. optical y(下) -> base_link z(上) 的负方向
    """
    return (
        float(camera_point[2]) + CAMERA_BACKWARD_BIAS_M,
        -float(camera_point[0]),
        -float(camera_point[1]),
    )


def base_link_point_to_map_point(robot_pose, base_link_point):
    """把 base_link 坐标系下的点，用当前机器人位姿换算到 odom 坐标系。

    这里直接使用当前 odom -> base_link 给出的二维位姿：
    1. base_link x 前进方向映射到 odom 平面；
    2. base_link y 左向映射到 odom 平面；
    3. z 直接叠加。
    """
    quaternion = (
        robot_pose.pose.orientation.x,
        robot_pose.pose.orientation.y,
        robot_pose.pose.orientation.z,
        robot_pose.pose.orientation.w,
    )
    _, _, yaw = euler_from_quaternion(quaternion)
    base_x, base_y, base_z = base_link_point

    map_point = Point()
    map_point.x = robot_pose.pose.position.x + math.cos(yaw) * base_x - math.sin(yaw) * base_y
    map_point.y = robot_pose.pose.position.y + math.sin(yaw) * base_x + math.cos(yaw) * base_y
    map_point.z = robot_pose.pose.position.z + base_z
    return map_point


def build_target_marker(person_point_map, marker_id=0):
    """为当前目标人构造 RViz marker。"""
    marker = Marker()
    marker.header.stamp = rospy.Time(0)
    marker.header.frame_id = ODOM_FRAME
    marker.ns = MARKER_NS
    marker.id = int(marker_id)
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = person_point_map.x
    marker.pose.position.y = person_point_map.y
    marker.pose.position.z = person_point_map.z
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.20
    marker.scale.y = 0.20
    marker.scale.z = 0.20
    marker.color.r = 0.1
    marker.color.g = 1.0
    marker.color.b = 0.2
    marker.color.a = 0.95
    return marker


def build_delete_marker(marker_id=0):
    """构造删除 marker 的消息，避免 RViz 残留旧点。"""
    marker = Marker()
    marker.header.stamp = rospy.Time(0)
    marker.header.frame_id = ODOM_FRAME
    marker.ns = MARKER_NS
    marker.id = int(marker_id)
    marker.action = Marker.DELETE
    return marker


def build_target_marker_array(person_points_map):
    """把多个目标点打包成 MarkerArray。"""
    marker_array = MarkerArray()
    for marker_id, person_point_map in enumerate(person_points_map or []):
        marker_array.markers.append(build_target_marker(person_point_map, marker_id=marker_id))
    return marker_array


def deduplicate_nearby_candidates(candidates, same_person_distance_m=SAME_PERSON_DISTANCE_M, max_count=2):
    """按 odom 平面距离去重，40cm 内认作同一个人。

    规则：
    1. 先按稳定深度从近到远排序；
    2. 若当前点与已保留点的平面距离不超过阈值，则跳过；
    3. 这样同一个人在整圈扫描里即便产生多个 track，也只保留一个代表点。
    """
    unique_candidates = []
    distance_threshold_sq = float(same_person_distance_m) * float(same_person_distance_m)

    for candidate in sorted(candidates or [], key=lambda item: float(item.get("depth_m", float("inf")))):
        person_point_map = candidate.get("person_point_map")
        if person_point_map is None:
            continue

        is_same_person = False
        for unique_candidate in unique_candidates:
            unique_point = unique_candidate.get("person_point_map")
            if unique_point is None:
                continue
            delta_x = float(person_point_map.x) - float(unique_point.x)
            delta_y = float(person_point_map.y) - float(unique_point.y)
            planar_distance_sq = delta_x * delta_x + delta_y * delta_y
            if planar_distance_sq <= distance_threshold_sq:
                is_same_person = True
                break

        if is_same_person:
            continue

        unique_candidates.append(candidate)
        if len(unique_candidates) >= max(0, int(max_count)):
            break

    return unique_candidates


class FindWavingCustomerActionServer:
    """挥手搜索动作服务端。"""

    def __init__(self):
        self.camera_frame = rospy.get_param("~camera_frame", DEFAULT_CAMERA_FRAME)
        self.rate_hz = rospy.get_param("~rate_hz", DEFAULT_RATE_HZ)

        self.cmd_vel_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)
        self.debug_image_pub = rospy.Publisher(DEBUG_IMAGE_TOPIC, Image, queue_size=1)
        self.marker_pub = rospy.Publisher(MARKER_TOPIC, MarkerArray, queue_size=1)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self._last_marker_count = 0

        self.server = actionlib.SimpleActionServer(
            ACTION_NAME,
            FindWavingCustomerAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.server.start()
        rospy.loginfo("[restaurant_wave] action ready: %s", ACTION_NAME)

    def _stop_robot(self):
        """无条件停车。"""
        self.cmd_vel_pub.publish(Twist())

    def _get_current_pose(self):
        """直接从 TF 读取 odom -> base_link 当前位姿。"""
        transform_message = self.tf_buffer.lookup_transform(
            ODOM_FRAME,
            ROBOT_BASE_FRAME,
            rospy.Time(0),
            rospy.Duration(2.0),
        )
        return self._transform_to_pose_stamped(transform_message)

    def _camera_point_to_map(self, camera_point):
        """把检测点直接按 base_link 共点假设转到 odom。"""
        robot_pose = self._get_current_pose()
        base_link_point = camera_point_to_base_link_point(camera_point)
        return base_link_point_to_map_point(robot_pose, base_link_point)

    @staticmethod
    def _transform_to_pose_stamped(transform_message):
        """把 TF 变换提取成 PoseStamped。"""
        pose = PoseStamped()
        pose.header.stamp = transform_message.header.stamp
        pose.header.frame_id = transform_message.header.frame_id
        pose.pose.position.x = transform_message.transform.translation.x
        pose.pose.position.y = transform_message.transform.translation.y
        pose.pose.position.z = transform_message.transform.translation.z
        pose.pose.orientation = transform_message.transform.rotation
        return pose

    def _publish_debug_image(self, debug_image):
        """把 BGR numpy 图像转成 ROS Image 并发布。"""
        if debug_image is None:
            return

        message = Image()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.camera_frame
        message.height = int(debug_image.shape[0])
        message.width = int(debug_image.shape[1])
        message.encoding = "bgr8"
        message.is_bigendian = 0
        message.step = int(debug_image.shape[1] * debug_image.shape[2])
        message.data = debug_image.tobytes()
        self.debug_image_pub.publish(message)

    def _publish_target_markers(self, person_points_map):
        """发布当前目标顾客集合的 MarkerArray。"""
        person_points_map = list(person_points_map or [])
        marker_array = build_target_marker_array(person_points_map)

        for marker_id in range(len(person_points_map), self._last_marker_count):
            marker_array.markers.append(build_delete_marker(marker_id=marker_id))

        self.marker_pub.publish(marker_array)
        self._last_marker_count = len(person_points_map)

    def execute_cb(self, goal):
        """执行一次完整的“原地转一圈找最近挥手人”流程。"""
        result = FindWavingCustomerResult(success=False, error_msg="")
        angular_speed = goal.angular_speed_rad_s if abs(goal.angular_speed_rad_s) > 1e-6 else 0.15
        timeout_sec = goal.timeout_sec if goal.timeout_sec > 0.0 else (2.0 * math.pi / abs(angular_speed))
        scan_duration = min(timeout_sec, 2.0 * math.pi / abs(angular_speed))
        detector = None
        best_candidates_by_track = {}

        try:
            detector = WaveDetectorRuntime()
            rate = rospy.Rate(self.rate_hz)
            start_time = rospy.Time.now()

            while not rospy.is_shutdown():
                elapsed = (rospy.Time.now() - start_time).to_sec()
                if elapsed >= scan_duration:
                    break

                if self.server.is_preempt_requested():
                    self._stop_robot()
                    self.server.set_preempted()
                    return

                twist = Twist()
                twist.angular.z = angular_speed
                self.cmd_vel_pub.publish(twist)

                frame_result = detector.process_once()
                if frame_result is not None:
                    self._publish_debug_image(frame_result.get("debug_image"))

                selected_candidate = None
                current_candidates = []
                if frame_result is not None:
                    selected_candidate = frame_result.get("selected_candidate")
                    current_candidates = frame_result.get("current_candidates") or []

                if selected_candidate is not None:
                    try:
                        person_point_map = self._camera_point_to_map(selected_candidate["camera_point"])
                    except Exception:
                        person_point_map = None
                    if person_point_map is not None:
                        self._publish_target_markers([person_point_map])
                    else:
                        self._publish_target_markers([])
                else:
                    self._publish_target_markers([])

                for candidate in current_candidates:
                    if not candidate.get("is_waving", False):
                        continue
                    if float(candidate.get("depth_m", 0.0)) <= 0.0:
                        continue
                    if candidate.get("camera_point") is None:
                        continue
                    try:
                        person_point_map = self._camera_point_to_map(candidate["camera_point"])
                    except Exception:
                        continue

                    track_id = candidate.get("track_id")
                    if track_id is None:
                        continue

                    current_best = best_candidates_by_track.get(track_id)
                    if current_best is None or float(candidate["depth_m"]) < float(current_best["depth_m"]):
                        best_candidates_by_track[track_id] = {
                            "track_id": track_id,
                            "depth_m": float(candidate["depth_m"]),
                            "person_point_map": person_point_map,
                        }
                rate.sleep()

            self._stop_robot()
            best_candidates = deduplicate_nearby_candidates(
                best_candidates_by_track.values(),
                same_person_distance_m=SAME_PERSON_DISTANCE_M,
                max_count=2,
            )

            if not best_candidates:
                self._publish_target_markers([])
                result.error_msg = "no waving customer found during one full scan"
                self.server.set_aborted(result, result.error_msg)
                return

            result.success = True
            result.person_points_map = [candidate["person_point_map"] for candidate in best_candidates]
            self._publish_target_markers(result.person_points_map)
            self.server.set_succeeded(result)
        except Exception as exc:
            self._stop_robot()
            self._publish_target_markers([])
            result.error_msg = "find waving customer failed: {}".format(exc)
            self.server.set_aborted(result, result.error_msg)
        finally:
            self._stop_robot()
            if not best_candidates_by_track:
                self._publish_target_markers([])
            if detector is not None:
                detector.close()


def main():
    """节点入口。"""
    rospy.init_node("find_waving_customer_action_server")
    FindWavingCustomerActionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
