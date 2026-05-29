#!/home/duo/anaconda3/envs/gpsr_perception310/bin/python
# -*- coding: utf-8 -*-
"""餐厅题挥手搜索 action server。"""

import math
import os
import sys
import time

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

from wave_detector_core import WaveDetectorRuntime, average_camera_points, render_debug_image


ACTION_NAME = "find_waving_customer"
ODOM_FRAME = "odom"
ROBOT_BASE_FRAME = "base_link"
CMD_VEL_TOPIC = "/cmd_vel"
DEBUG_IMAGE_TOPIC = "/restaurant_wave/debug_image"
MARKER_TOPIC = "/restaurant_wave/target_marker"
MARKER_NS = "restaurant_wave_target"
DEFAULT_CAMERA_FRAME = "camera1_color_optical_frame"
DEFAULT_RATE_HZ = 10.0
# realsense坐标偏移
CAMERA_BACKWARD_BIAS_M = 0.1
SAME_PERSON_DISTANCE_M = 0.60
DEFAULT_SWING_TIMEOUT_SEC = 30.0
# 旋转幅度
SWING_HALF_RANGE_RAD = math.radians(10.0)
SWING_TARGET_OFFSETS_RAD = (0.0, SWING_HALF_RANGE_RAD, -SWING_HALF_RANGE_RAD)
SWING_POSITION_DETECT_FRAMES = 60
SWING_TRACK_HIT_REQUIRED_FRAMES = 4
POST_HIT_EXTRA_POSITION_SCAN_COUNT = 1
RESTORE_ROTATE_TIMEOUT_SEC = 8.0
RESTORE_ROTATE_CONTROL_HZ = 20.0
RESTORE_YAW_TOLERANCE_RAD = math.radians(2.0)
SWING_ENDPOINT_PAUSE_SEC = 1.0


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


def normalize_angle_rad(angle_rad):
    """把角度归一化到 [-pi, pi]。"""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def pose_to_yaw(robot_pose):
    """从 PoseStamped 中提取平面 yaw。"""
    quaternion = (
        robot_pose.pose.orientation.x,
        robot_pose.pose.orientation.y,
        robot_pose.pose.orientation.z,
        robot_pose.pose.orientation.w,
    )
    return euler_from_quaternion(quaternion)[2]


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
    3. 这样同一个人在整次扫描里即便产生多个 track，也只保留一个代表点。
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

    def _rotate_to_yaw(
        self,
        target_yaw,
        angular_speed_rad_s,
        timeout_sec=RESTORE_ROTATE_TIMEOUT_SEC,
        log_context="target yaw",
        stop_requested=None,
    ):
        """原地转到指定 yaw，转动过程中不做视觉检测。"""
        if target_yaw is None:
            return False

        rotate_speed = abs(float(angular_speed_rad_s)) if abs(float(angular_speed_rad_s)) > 1e-6 else 0.15
        rate = rospy.Rate(RESTORE_ROTATE_CONTROL_HZ)
        deadline = time.monotonic() + max(0.0, float(timeout_sec))

        try:
            while not rospy.is_shutdown():
                if stop_requested is not None and stop_requested():
                    return False

                try:
                    current_yaw = pose_to_yaw(self._get_current_pose())
                except Exception as exc:
                    rospy.logwarn("[restaurant_wave] rotate to %s skipped: %s", log_context, str(exc))
                    return False

                yaw_error = normalize_angle_rad(float(target_yaw) - float(current_yaw))
                if abs(yaw_error) <= RESTORE_YAW_TOLERANCE_RAD:
                    return True
                if time.monotonic() >= deadline:
                    rospy.logwarn("[restaurant_wave] rotate to %s timed out", log_context)
                    return False

                twist = Twist()
                twist.angular.z = math.copysign(rotate_speed, yaw_error)
                self.cmd_vel_pub.publish(twist)
                rate.sleep()
            return False
        finally:
            self._stop_robot()

    def _restore_initial_yaw(self, initial_yaw, angular_speed_rad_s):
        """退出 action 前，原地回到扫描开始时的朝向。"""
        if initial_yaw is None:
            return

        self._rotate_to_yaw(
            initial_yaw,
            angular_speed_rad_s,
            timeout_sec=RESTORE_ROTATE_TIMEOUT_SEC,
            log_context="initial yaw",
        )

    def execute_cb(self, goal):
        """执行一次完整的“三点离散角度检测找挥手人”流程。"""
        result = FindWavingCustomerResult(success=False, error_msg="")
        angular_speed = abs(goal.angular_speed_rad_s) if abs(goal.angular_speed_rad_s) > 1e-6 else 0.15
        timeout_sec = goal.timeout_sec if goal.timeout_sec > 0.0 else DEFAULT_SWING_TIMEOUT_SEC
        detector = None
        confirmed_candidates_by_track = {}
        confirmed_track_ids = set()
        initial_yaw = None
        swing_target_index = 0
        pending_return_to_initial = False
        first_hit_registered = False
        extra_position_scans_after_first_hit = 0
        finish_search = False
        rospy.loginfo(
            "[restaurant_wave] action start | timeout=%.2fs angular_speed=%.3frad/s detect_frames=%d hit_required=%d offsets_deg=%s",
            float(timeout_sec),
            float(angular_speed),
            int(SWING_POSITION_DETECT_FRAMES),
            int(SWING_TRACK_HIT_REQUIRED_FRAMES),
            ["{:.1f}".format(math.degrees(offset)) for offset in SWING_TARGET_OFFSETS_RAD],
        )

        try:
            detector = WaveDetectorRuntime()
            rate = rospy.Rate(self.rate_hz)
            start_time = rospy.Time.now()
            initial_yaw = pose_to_yaw(self._get_current_pose())
            self._publish_target_markers([])

            while not rospy.is_shutdown() and not finish_search:
                elapsed = (rospy.Time.now() - start_time).to_sec()
                if elapsed >= timeout_sec:
                    break

                if self.server.is_preempt_requested():
                    self._stop_robot()
                    self._restore_initial_yaw(initial_yaw, angular_speed)
                    self.server.set_preempted()
                    return

                if pending_return_to_initial:
                    rotate_timeout_sec = min(
                        RESTORE_ROTATE_TIMEOUT_SEC,
                        max(0.0, timeout_sec - elapsed),
                    )
                    reached_initial = self._rotate_to_yaw(
                        initial_yaw,
                        angular_speed,
                        timeout_sec=rotate_timeout_sec,
                        log_context="initial yaw after first hit",
                        stop_requested=self.server.is_preempt_requested,
                    )
                    if self.server.is_preempt_requested():
                        self._stop_robot()
                        self._restore_initial_yaw(initial_yaw, angular_speed)
                        self.server.set_preempted()
                        return
                    if not reached_initial:
                        rospy.logwarn("[restaurant_wave] return to initial yaw after first hit failed; stop search")
                        break

                    self._stop_robot()
                    rospy.sleep(SWING_ENDPOINT_PAUSE_SEC)
                    pending_return_to_initial = False
                    continue

                target_offset = SWING_TARGET_OFFSETS_RAD[swing_target_index]
                target_yaw = normalize_angle_rad(initial_yaw + target_offset)
                rotate_timeout_sec = min(
                    RESTORE_ROTATE_TIMEOUT_SEC,
                    max(0.0, timeout_sec - elapsed),
                )
                reached_target = self._rotate_to_yaw(
                    target_yaw,
                    angular_speed,
                    timeout_sec=rotate_timeout_sec,
                    log_context="swing offset {:.3f} rad".format(target_offset),
                    stop_requested=self.server.is_preempt_requested,
                )
                if self.server.is_preempt_requested():
                    self._stop_robot()
                    self._restore_initial_yaw(initial_yaw, angular_speed)
                    self.server.set_preempted()
                    return
                if not reached_target:
                    rospy.logwarn(
                        "[restaurant_wave] rotate to scan offset %.1fdeg failed; stop search",
                        math.degrees(target_offset),
                    )
                    break

                self._stop_robot()
                rospy.sleep(SWING_ENDPOINT_PAUSE_SEC)

                current_scan_is_post_hit_followup = (
                    first_hit_registered and extra_position_scans_after_first_hit > 0
                )
                first_hit_happened_this_position = False
                position_detection_counts = {}
                position_camera_points_by_track = {}
                position_frame_count = 0
                position_none_frame_count = 0
                position_max_person_count = 0
                position_max_waving_count = 0
                scan_start_monotonic = time.monotonic()

                for _ in range(SWING_POSITION_DETECT_FRAMES):
                    elapsed = (rospy.Time.now() - start_time).to_sec()
                    if elapsed >= timeout_sec:
                        finish_search = True
                        break

                    if self.server.is_preempt_requested():
                        self._stop_robot()
                        self._restore_initial_yaw(initial_yaw, angular_speed)
                        self.server.set_preempted()
                        return

                    position_frame_count += 1
                    frame_result = detector.process_once()
                    color_image = None
                    current_candidates = []
                    if frame_result is not None:
                        color_image = frame_result.get("color_image")
                        current_candidates = frame_result.get("current_candidates") or []
                    else:
                        position_none_frame_count += 1

                    current_waving_count = sum(1 for candidate in current_candidates if candidate.get("is_waving", False))
                    position_max_person_count = max(position_max_person_count, len(current_candidates))
                    position_max_waving_count = max(position_max_waving_count, current_waving_count)

                    counted_track_ids_this_frame = set()
                    for candidate in current_candidates:
                        if not candidate.get("is_waving", False):
                            continue

                        track_id = candidate.get("track_id")
                        if track_id is None:
                            continue
                        if track_id in counted_track_ids_this_frame:
                            continue
                        counted_track_ids_this_frame.add(track_id)

                        position_detection_counts[track_id] = position_detection_counts.get(track_id, 0) + 1
                        camera_point = candidate.get("camera_point")
                        if camera_point is not None:
                            position_camera_points_by_track.setdefault(track_id, []).append(camera_point)
                        if (
                            not first_hit_registered
                            and not first_hit_happened_this_position
                            and position_detection_counts[track_id] >= SWING_TRACK_HIT_REQUIRED_FRAMES
                        ):
                            first_hit_happened_this_position = True

                    if color_image is not None:
                        hit_track_ids = set(confirmed_track_ids)
                        for track_id, detection_count in position_detection_counts.items():
                            if detection_count >= SWING_TRACK_HIT_REQUIRED_FRAMES:
                                hit_track_ids.add(track_id)
                        debug_image = render_debug_image(color_image, current_candidates, hit_track_ids)
                        self._publish_debug_image(debug_image)

                    rate.sleep()

                hit_track_count = sum(
                    1
                    for detection_count in position_detection_counts.values()
                    if detection_count >= SWING_TRACK_HIT_REQUIRED_FRAMES
                )
                camera_point_track_count = sum(
                    1 for camera_points in position_camera_points_by_track.values() if camera_points
                )
                rospy.loginfo(
                    "[restaurant_wave] scan offset %.1fdeg done | frames=%d none=%d elapsed=%.2fs max_persons=%d max_waving=%d waving_counts=%s hit_tracks=%d camera_tracks=%d confirmed_total=%d post_hit_followup=%s timeout_break=%s",
                    math.degrees(target_offset),
                    int(position_frame_count),
                    int(position_none_frame_count),
                    float(time.monotonic() - scan_start_monotonic),
                    int(position_max_person_count),
                    int(position_max_waving_count),
                    str(position_detection_counts),
                    int(hit_track_count),
                    int(camera_point_track_count),
                    int(len(confirmed_candidates_by_track)),
                    str(bool(current_scan_is_post_hit_followup)),
                    str(bool(finish_search)),
                )

                if finish_search:
                    break

                for track_id, detection_count in position_detection_counts.items():
                    if detection_count < SWING_TRACK_HIT_REQUIRED_FRAMES:
                        continue

                    confirmed_track_ids.add(track_id)
                    if track_id in confirmed_candidates_by_track:
                        continue

                    average_camera_point = average_camera_points(
                        position_camera_points_by_track.get(track_id, [])
                    )
                    if average_camera_point is None:
                        rospy.logwarn(
                            "[restaurant_wave] hit track %s skipped: no valid camera point samples",
                            str(track_id),
                        )
                        continue

                    try:
                        person_point_map = self._camera_point_to_map(average_camera_point)
                    except Exception as exc:
                        rospy.logwarn(
                            "[restaurant_wave] hit track %s skipped: map conversion failed: %s",
                            str(track_id),
                            str(exc),
                        )
                        continue

                    confirmed_candidates_by_track[track_id] = {
                        "track_id": track_id,
                        "depth_m": float(average_camera_point[2]),
                        "person_point_map": person_point_map,
                    }
                    rospy.loginfo(
                        "[restaurant_wave] confirmed track %s | depth=%.3fm map=(%.3f, %.3f, %.3f)",
                        str(track_id),
                        float(average_camera_point[2]),
                        float(person_point_map.x),
                        float(person_point_map.y),
                        float(person_point_map.z),
                    )

                confirmed_candidates = deduplicate_nearby_candidates(
                    confirmed_candidates_by_track.values(),
                    same_person_distance_m=SAME_PERSON_DISTANCE_M,
                    max_count=2,
                )
                rospy.loginfo(
                    "[restaurant_wave] confirmed candidates after dedup: %d",
                    int(len(confirmed_candidates)),
                )
                self._publish_target_markers(
                    [candidate["person_point_map"] for candidate in confirmed_candidates]
                )

                if first_hit_happened_this_position:
                    first_hit_registered = True
                    # 当前扫描角度一旦首次命中，就把它作为最后一个识别角度；
                    # 后面不再继续扫下一个角度，直接在统一收尾阶段回正并返回结果。
                    finish_search = True
                elif current_scan_is_post_hit_followup:
                    extra_position_scans_after_first_hit -= 1
                    if extra_position_scans_after_first_hit <= 0:
                        finish_search = True

                swing_target_index = (swing_target_index + 1) % len(SWING_TARGET_OFFSETS_RAD)

            self._stop_robot()
            confirmed_candidates = deduplicate_nearby_candidates(
                confirmed_candidates_by_track.values(),
                same_person_distance_m=SAME_PERSON_DISTANCE_M,
                max_count=2,
            )
            self._restore_initial_yaw(initial_yaw, angular_speed)

            if not confirmed_candidates:
                self._publish_target_markers([])
                result.error_msg = "no waving customer found within search timeout"
                rospy.logwarn("[restaurant_wave] action abort | %s", result.error_msg)
                self.server.set_aborted(result, result.error_msg)
                return

            result.success = True
            result.person_points_map = [candidate["person_point_map"] for candidate in confirmed_candidates]
            self._publish_target_markers(result.person_points_map)
            rospy.loginfo("[restaurant_wave] action success | targets=%d", len(result.person_points_map))
            self.server.set_succeeded(result)
        except Exception as exc:
            self._stop_robot()
            self._restore_initial_yaw(initial_yaw, angular_speed)
            self._publish_target_markers([])
            result.error_msg = "find waving customer failed: {}".format(exc)
            rospy.logerr("[restaurant_wave] action exception | %s", result.error_msg)
            self.server.set_aborted(result, result.error_msg)
        finally:
            self._stop_robot()
            if not confirmed_candidates_by_track:
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
