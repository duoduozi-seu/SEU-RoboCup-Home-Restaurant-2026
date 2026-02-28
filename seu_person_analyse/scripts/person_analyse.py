#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import sys
import time
import os

import cv2

# 动态获取路径
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PACKAGE_DIR = os.path.dirname(_SCRIPT_DIR)
_SRC_DIR = os.path.dirname(_PACKAGE_DIR)
sys.path.insert(0, _SRC_DIR)
from common.camera import Camera, predict_gender
import rospy
import tf2_ros
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped, Pose, Quaternion, Twist
from tf2_geometry_msgs import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from cv_bridge import CvBridge
import sensor_msgs.msg
from seu_robot_msgs.msg import object_position
from seu_person_analyse.srv import *

# move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

class Person_analyse_service:
    def __init__(self):
        self.start_pose = None
        self.camera = Camera(serial_number=1)
        #ros接收初始化
        rospy.Subscriber('/robot_pose', PoseStamped, self.pose_callback)
        self.current_pose = PoseStamped()
        self.target_point = Point()
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose = {}
        self.gender = {}

        self.cloth_color = {}
        self.bridge = CvBridge()
        self.name = None
        self.image_pub = rospy.Publisher("/gpsr_img", sensor_msgs.msg.Image, queue_size=10)
    def person_analyse_callback(self,req):
        feature = req.feature
        rospy.loginfo(f"start person analyse:{feature}")
        self.camera.start_camera()

        # 初始化服务返回消息
        response = seu_person_analyseResponse()
        self.start_pose = rospy.wait_for_message('/robot_pose', PoseStamped)
        self.adjust_distance()
        start_time = time.time()

        while time.time() - start_time < 5:
            color_image, depth_image, depth_intrin, depth_frame = self.camera.get_frame()
            if color_image is not None:
                color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
                pose = self.camera.pose.get_pose(color_image, )
                if pose != 'none':
                    if pose in self.pose.keys():
                        self.pose[pose] += 1
                    else:
                        self.pose[pose] = 1
                gender = predict_gender(color_image)
                if gender in self.gender.keys():
                    self.gender[gender] += 1
                else:
                    self.gender[gender] = 1
                cv2.putText(color_image, f"pose:{pose},gender:{gender}",
                            (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                ros_image = self.bridge.cv2_to_imgmsg(color_image, encoding="rgb8")
                # 发布图片消息
                self.image_pub.publish(ros_image)
        print("\nPose counts:")
        for pose, count in self.pose.items():
            print(f"{pose}: {count}")
        # Print gender counts
        print("\nGender counts:")
        for gender, count in self.gender.items():
            print(f"{gender}: {count}")
        # Find most frequent pose
        if self.pose:
            most_frequent_pose = max(self.pose.items(), key=lambda x: x[1])
            pose = most_frequent_pose[0]
            print(f"\nMost frequent pose: {most_frequent_pose[0]} (count: {most_frequent_pose[1]})")
        else:
            pose = "uncertain"
            print("\nNo pose data collected")
        # Find most frequent gender
        if self.gender:
            most_frequent_gender = max(self.gender.items(), key=lambda x: x[1])
            gender = most_frequent_gender[0]
            print(f"Most frequent gender: {most_frequent_gender[0]} (count: {most_frequent_gender[1]})")
        else:
            gender = "uncertain"
            print("No gender data collected")
        rospy.set_param("gesture", pose)
        rospy.set_param("gender",gender)
        rospy.set_param("person_info",f"with pose {pose} and gender is {gender}")
        response.state = True
        # 返回消息
        return response

    def adjust_distance(self):
        self.start_pose = rospy.wait_for_message('/robot_pose', PoseStamped)
        rospy.loginfo("Start_pose:%s,%s", self.start_pose.pose.position.x, self.start_pose.pose.position.y)
        detect_times = 0
        # 找最近的人
        while True:
            color_image, depth_image, depth_intrin, depth_frame = self.camera.get_frame()
            if color_image is not None:
                color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
                results = self.camera.people_track.detect_people(color_image, depth_frame=depth_frame,
                                                                 depth_intrin=depth_intrin)
                nearest = None
                min_distnce = 100
                for res in results:
                    x1, y1, x2, y2, pos = res
                    if pos is not None:
                        if pos[1][2] < min_distnce:
                            min_distnce = pos[1][2]
                            nearest = pos[1]
                detect_times += 1
                if nearest is not None and detect_times >= 5:
                    break
                cv2.putText(color_image, f"distance:{min_distnce}",
                            (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                ros_image = self.bridge.cv2_to_imgmsg(color_image, encoding="rgb8")
                # 发布图片消息
                self.image_pub.publish(ros_image)

        ps = tf2_geometry_msgs.PointStamped()

        # 设置原坐标相关参数：时间戳、原坐标系、坐标
        ps.header.stamp = rospy.Time()
        ps.header.frame_id = "camera1_color_optical_frame"
        ps.point.x = nearest[0]
        ps.point.y = nearest[1]
        ps.point.z = nearest[2]
        # tf坐标转换：指定转换后坐标系
        ps_out = buffer.transform(ps, "map")
        self.target_point.x = ps_out.point.x
        self.target_point.y = ps_out.point.y
        rospy.loginfo("Target_pose:%s,%s", self.target_point.x, self.target_point.y)
        #对齐角度
        while not rospy.is_shutdown():
            # 给定目标位置
            # Calculate the direction from robot current position to target point
            target_yaw = math.atan2(self.target_point.y - self.start_pose.pose.position.y,
                                    self.target_point.x - self.start_pose.pose.position.x)
            current_yaw = euler_from_quaternion([self.current_pose.pose.orientation.x,
                                                 self.current_pose.pose.orientation.y,
                                                 self.current_pose.pose.orientation.z,
                                                 self.current_pose.pose.orientation.w])[2]
            yaw_error = target_yaw - current_yaw
            if yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            elif yaw_error < -math.pi:
                yaw_error += 2 * math.pi
            # Stop sending control commands if robot is already facing the target point
            rospy.loginfo("yaw_error:%s", yaw_error)
            if abs(yaw_error) < 0.05:
                cmd_vel_stop = Twist()
                cmd_vel_stop.linear.x = 0
                cmd_vel_stop.angular.z = 0

                self.cmd_vel_pub.publish(cmd_vel_stop)
                break
            # Calculate the robot control command
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.3 * yaw_error / abs(yaw_error)

            # Publish the robot control command
            self.cmd_vel_pub.publish(cmd_vel)
            # Wait for a while to let the robot rotate to the new direction
            rospy.sleep(0.1)

        # 调整距离
        while not rospy.is_shutdown():
            distance = math.sqrt(pow(self.target_point.x - self.current_pose.pose.position.x, 2) +
                                 pow(self.target_point.y - self.current_pose.pose.position.y, 2))
            # Stop sending control commands if robot is already facing the target point
            if distance > 1.5:
                rospy.logwarn('Stop!')
                cmd_vel_stop = Twist()
                cmd_vel_stop.linear.x = 0
                cmd_vel_stop.angular.z = 0
                self.cmd_vel_pub.publish(cmd_vel_stop)
                break
            # Calculate the robot control command
            cmd_vel = Twist()
            cmd_vel.linear.x = -0.1
            cmd_vel.angular.z = 0
            rospy.loginfo("distance:%s", distance)
            # Publish the robot control command
            self.cmd_vel_pub.publish(cmd_vel)
            # Wait for a while to let the robot rotate to the new direction
            rospy.sleep(0.1)
        return True

    def pose_callback(self, msg):
        # Update robot current pose
        self.current_pose = msg

if __name__ == "__main__":
    rospy.init_node('person_analyse_service_node')
    buffer = tf2_ros.Buffer()
    sub = tf2_ros.TransformListener(buffer)
    person_analyse_service = Person_analyse_service()
    server = rospy.Service("person_analyse_service", seu_person_analyse, person_analyse_service.person_analyse_callback)
    rospy.loginfo("Waiting for move_base action server...")
    # move_base.wait_for_server(rospy.Duration(60))
    # Wait 60 seconds for the action server to become available
    rospy.loginfo("Connected to move base server")
    rospy.loginfo("person analyse service ready")
    #循环接收消息
    rospy.spin()
