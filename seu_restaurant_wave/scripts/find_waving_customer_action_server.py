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
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from seu_restaurant_msgs.msg import FindWavingCustomerAction, FindWavingCustomerResult
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker

# 这个入口脚本依赖同目录的 `wave_detector_core.py`。
# 这里主动补上当前脚本目录，保证直接运行 scripts 文件时导入稳定。
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from wave_detector_core import WaveDetectorRuntime


ACTION_NAME = "find_waving_customer"
# 当前导航系统输出的是 map 坐标系下的 Odometry，而不是 `/robot_pose`。
ROBOT_POSE_TOPIC = "/map_to_baselink"
CMD_VEL_TOPIC = "/cmd_vel"
DEBUG_IMAGE_TOPIC = "/restaurant_wave/debug_image"
MARKER_TOPIC = "/restaurant_wave/target_marker"
MARKER_NS = "restaurant_wave_target"
DEFAULT_CAMERA_FRAME = "camera1_color_optical_frame"
DEFAULT_RATE_HZ = 10.0


def camera_point_to_base_link_point(camera_point):
    """把 RealSense 光学坐标系下的点，按“与 base_link 共点”假设转成 base_link 坐标。

    这里不再依赖相机 TF，只做最小轴变换：
    1. optical z(前) -> base_link x(前)
    2. optical x(右) -> base_link y(左) 的负方向
    3. optical y(下) -> base_link z(上) 的负方向
    """
    return (
        float(camera_point[2]),
        -float(camera_point[0]),
        -float(camera_point[1]),
    )


def base_link_point_to_map_point(robot_pose, base_link_point):
    """把 base_link 坐标系下的点，用当前机器人位姿换算到 map 坐标系。

    这里直接使用当前 `/map_to_baselink` 给出的二维位姿：
    1. base_link x 前进方向映射到 map 平面；
    2. base_link y 左向映射到 map 平面；
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


def build_target_marker(person_point_map):
    """为当前目标人构造 RViz marker。"""
    marker = Marker()
    marker.header.stamp = rospy.Time(0)
    marker.header.frame_id = "map"
    marker.ns = MARKER_NS
    marker.id = 0
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


def build_delete_marker():
    """构造删除 marker 的消息，避免 RViz 残留旧点。"""
    marker = Marker()
    marker.header.stamp = rospy.Time(0)
    marker.header.frame_id = "map"
    marker.ns = MARKER_NS
    marker.id = 0
    marker.action = Marker.DELETE
    return marker


class FindWavingCustomerActionServer:
    """挥手搜索动作服务端。"""

    def __init__(self):
        self.camera_frame = rospy.get_param("~camera_frame", DEFAULT_CAMERA_FRAME)
        self.rate_hz = rospy.get_param("~rate_hz", DEFAULT_RATE_HZ)
        self.current_pose = None

        rospy.Subscriber(ROBOT_POSE_TOPIC, Odometry, self._pose_callback, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)
        self.debug_image_pub = rospy.Publisher(DEBUG_IMAGE_TOPIC, Image, queue_size=1)
        self.marker_pub = rospy.Publisher(MARKER_TOPIC, Marker, queue_size=1)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.server = actionlib.SimpleActionServer(
            ACTION_NAME,
            FindWavingCustomerAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.server.start()
        rospy.loginfo("[restaurant_wave] action ready: %s", ACTION_NAME)

    def _pose_callback(self, message):
        """缓存当前机器人位姿。"""
        self.current_pose = self._odometry_to_pose_stamped(message)

    def _stop_robot(self):
        """无条件停车。"""
        self.cmd_vel_pub.publish(Twist())

    def _camera_point_to_map(self, camera_point):
        """把检测点直接按 base_link 共点假设转到 map。"""
        if self.current_pose is None:
            raise RuntimeError("robot pose is unavailable")
        base_link_point = camera_point_to_base_link_point(camera_point)
        return base_link_point_to_map_point(self.current_pose, base_link_point)

    @staticmethod
    def _odometry_to_pose_stamped(odometry_message):
        """把导航系统的 Odometry 提取成 PoseStamped。"""
        pose = PoseStamped()
        pose.header = odometry_message.header
        pose.pose = odometry_message.pose.pose
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

    def _publish_target_marker(self, person_point_map):
        """持续发布当前选中顾客的 marker。"""
        if person_point_map is None:
            self.marker_pub.publish(build_delete_marker())
            return
        self.marker_pub.publish(build_target_marker(person_point_map))

    def execute_cb(self, goal):
        """执行一次完整的“原地转一圈找最近挥手人”流程。"""
        result = FindWavingCustomerResult(success=False, error_msg="", depth_m=0.0)
        angular_speed = goal.angular_speed_rad_s if abs(goal.angular_speed_rad_s) > 1e-6 else 0.15
        timeout_sec = goal.timeout_sec if goal.timeout_sec > 0.0 else (2.0 * math.pi / abs(angular_speed))
        scan_duration = min(timeout_sec, 2.0 * math.pi / abs(angular_speed))
        detector = None
        best_candidate = None

        try:
            if self.current_pose is None:
                odometry_message = rospy.wait_for_message(ROBOT_POSE_TOPIC, Odometry, timeout=2.0)
                self.current_pose = self._odometry_to_pose_stamped(odometry_message)

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

                candidate = None
                if frame_result is not None:
                    candidate = frame_result.get("selected_candidate")

                if candidate is not None:
                    try:
                        person_point_map = self._camera_point_to_map(candidate["camera_point"])
                    except Exception:
                        person_point_map = None
                    if person_point_map is not None:
                        self._publish_target_marker(person_point_map)
                        candidate["person_point_map"] = person_point_map
                        if best_candidate is None or candidate["depth_m"] < best_candidate["depth_m"]:
                            best_candidate = candidate
                    else:
                        self._publish_target_marker(None)
                else:
                    self._publish_target_marker(None)
                rate.sleep()

            self._stop_robot()
            if best_candidate is None:
                self._publish_target_marker(None)
                result.error_msg = "no waving customer found during one full scan"
                self.server.set_aborted(result, result.error_msg)
                return

            result.success = True
            result.depth_m = float(best_candidate["depth_m"])
            result.person_point_map = best_candidate["person_point_map"]
            self._publish_target_marker(best_candidate["person_point_map"])
            self.server.set_succeeded(result)
        except Exception as exc:
            self._stop_robot()
            self._publish_target_marker(None)
            result.error_msg = "find waving customer failed: {}".format(exc)
            self.server.set_aborted(result, result.error_msg)
        finally:
            self._stop_robot()
            if best_candidate is None:
                self._publish_target_marker(None)
            if detector is not None:
                detector.close()


def main():
    """节点入口。"""
    rospy.init_node("find_waving_customer_action_server")
    FindWavingCustomerActionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
