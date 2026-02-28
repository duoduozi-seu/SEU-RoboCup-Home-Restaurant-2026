#! /home/syw/anaconda3/envs/GPSR/bin/python
# -*- coding: UTF-8 -*-
import sys
import time

sys.path.append("/home/syw/code/robocup_nation/2024code/GPSR/src/common")
from common import *
import rospy
import tf2_ros
import actionlib
from smach import State, StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped, Pose, Quaternion, Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

import math

from seu_robot_msgs.msg import object_position
from seu_lead.srv import *

move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
rospy.loginfo("Waiting for move_base action server...")
# Wait 60 seconds for the action server to become available
rospy.loginfo("Connected to move base server")

class LeadService:
    def __init__(self):
        # 上一次询问是否到达目的地的时间
        self.last_check_time = time.time()
        self.start_time = time.time()
        # 检测是否达到目的地的时间间隔
        self.check_time = 3
        # 距离多少算到达
        self.min_distance = 0.5
        self.word = ""
        self.current_pose =  PoseStamped()
        self.target_point = PoseStamped()
        self.Goal = MoveBaseGoal()

    def is_need_check(self):
        if time.time() - self.last_check_time > self.check_time:
            return True
        else:
            return False

    def lead_check(self,req):
        if self.is_need_check():
            speech_to_text("please follow me")
            # if self.check_corners():
            #     self.last_check_time = time.time()
                
            #     move_base.cancel_goal()
            #     time.sleep(2)
            #     move_base.send_goal(self.Goal)
        return
    def check_corners(self):
        distance = 100
        for pos in locations['corners']:
            distance = min(math.sqrt(pow(pos.position.x - self.current_pose.pose.position.x, 2) +
                             pow(pos.position.y - self.current_pose.pose.position.y, 2)),distance)
        if distance < self.min_distance:
            return True
        else:
            return False

    def find_person(self,req):

        return

    def lead_callback(self,req):
        rospy.loginfo("start lead")
        # 提取入参中的des字符作为目标点
        location = req.location
        # 初始化服务返回消息
        response = seu_leadResponse()
        # 设置目标点
        self.Goal = MoveBaseGoal()
        rospy.loginfo(location)
        if location in locations.keys():
            self.Goal.target_pose.pose = locations[location]
            rospy.loginfo(locations[location])
        else:
            rospy.logerror("Unknown destination")
            response.state = False
            return response
        self.last_check_time = time.time()
        self.start_time = time.time()
        # Subscribe to the move_base action server
        rospy.Subscriber('/robot_pose', PoseStamped, self.pose_callback)
        self.Goal.target_pose.header.frame_id = 'map'
        self.Goal.target_pose.header.stamp = rospy.Time.now()

        # 打印目标点名称
        rospy.loginfo("Going to: " + str(location))
        speech_to_text("please follow me")
        # 发送目标点，启动导航
        move_base.send_goal(self.Goal)
        suc = False
        while time.time() - self.start_time < 60:
            if move_base.get_state() == actionlib.SimpleGoalState.DONE:
                rospy.loginfo("finish lead")
                suc = True
                break
            self.lead_check(req)
        # 检查导航结果
        if not suc:
            move_base.cancel_goal()
            rospy.logwarn("Timed Out Achieving Goal")
            response.state = False
        else:
            rospy.logwarn('Nav Success')
            response.state = True
        # 返回消息
        return response

    def pose_callback(self, msg):
        self.current_pose = msg

if __name__ == "__main__":
    rospy.init_node('lead_service_node')
    lead_service = LeadService()
    server = rospy.Service("lead_service", seu_lead, lead_service.lead_callback)
    move_base.wait_for_server(rospy.Duration(60))
    rospy.loginfo("lead service ready")
    #循环接收消息
    rospy.spin()
