#!/home/duo/anaconda3/envs/gpsr_perception310/bin/python
# -*- coding: utf-8 -*-
"""餐厅题挥手搜索 action server。"""

import math
import os
import sys

import actionlib
import rospy
import tf2_ros

from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from seu_restaurant_msgs.msg import FindWavingCustomerAction, FindWavingCustomerResult

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
DEFAULT_CAMERA_FRAME = "camera1_color_optical_frame"
DEFAULT_RATE_HZ = 10.0


class FindWavingCustomerActionServer:
    """挥手搜索动作服务端。"""

    def __init__(self):
        self.camera_frame = rospy.get_param("~camera_frame", DEFAULT_CAMERA_FRAME)
        self.rate_hz = rospy.get_param("~rate_hz", DEFAULT_RATE_HZ)
        self.current_pose = None

        rospy.Subscriber(ROBOT_POSE_TOPIC, Odometry, self._pose_callback, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)
        self.debug_image_pub = rospy.Publisher(DEBUG_IMAGE_TOPIC, Image, queue_size=1)
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
        """把相机坐标系下的 3D 点转到 map。"""
        stamped = PointStamped()
        stamped.header.stamp = rospy.Time(0)
        stamped.header.frame_id = self.camera_frame
        stamped.point.x = camera_point[0]
        stamped.point.y = camera_point[1]
        stamped.point.z = camera_point[2]
        transformed = self.tf_buffer.transform(stamped, "map", rospy.Duration(0.2))
        return transformed.point

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
                        candidate["person_point_map"] = person_point_map
                        if best_candidate is None or candidate["depth_m"] < best_candidate["depth_m"]:
                            best_candidate = candidate
                rate.sleep()

            self._stop_robot()
            if best_candidate is None:
                result.error_msg = "no waving customer found during one full scan"
                self.server.set_aborted(result, result.error_msg)
                return

            result.success = True
            result.depth_m = float(best_candidate["depth_m"])
            result.person_point_map = best_candidate["person_point_map"]
            self.server.set_succeeded(result)
        except Exception as exc:
            self._stop_robot()
            result.error_msg = "find waving customer failed: {}".format(exc)
            self.server.set_aborted(result, result.error_msg)
        finally:
            self._stop_robot()
            if detector is not None:
                detector.close()


def main():
    """节点入口。"""
    rospy.init_node("find_waving_customer_action_server")
    FindWavingCustomerActionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
