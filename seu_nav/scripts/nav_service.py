#! /home/syw/anaconda3/envs/GPSR/bin/python
import sys
sys.path.append("/home/syw/code/robocup_nation/2024code/GPSR/src/common")
from common import *
import rospy
import actionlib
import collections
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from seu_nav.srv import *
import re
import json
# 服务名称
seuhome_nav_name = 'nav_service'

# 预定义目标点位姿列表（在这里添加目标点的位姿）
#################################
#########检查地点字符串是否有空格！！！############
#################################

# Nav服务执行函数
def Nav_Service(req):
    # 提取入参中的des字符作为目标点
    location = req.des
    location = location.lower().replace('_',' ')
    print(location)
    # Subscribe to the move_base action server
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    # Wait 60 seconds for the action server to become available
    move_base.wait_for_server(rospy.Duration(60))
    rospy.loginfo("Connected to move base server")
    # 设置目标点
    Goal = MoveBaseGoal()
    rospy.loginfo(location)
    Goal.target_pose.pose = locations[location]
    rospy.loginfo(locations[location])
    Goal.target_pose.header.frame_id = 'map'
    Goal.target_pose.header.stamp = rospy.Time.now()

    # 打印目标点名称
    rospy.loginfo("Going to: " + str(location))
    # 发送目标点，启动导航
    move_base.send_goal(Goal)

    # 设置60s为超时时间
    finished_within_time = move_base.wait_for_result(rospy.Duration(60))
    # 初始化服务返回消息
    response = seu_navResponse()
    # 检查导航结果
    if not finished_within_time:
        move_base.cancel_goal()
        rospy.logwarn("Timed Out Achieving Goal")
        response.state = False
    else:
        rospy.logwarn('Nav Success')
        response.state = True
    # 返回消息
    return response

if __name__ == '__main__':
    rospy.init_node('nav_service_node')
    service = rospy.Service(seuhome_nav_name, seu_nav, Nav_Service)
    rospy.loginfo('Nav Service Ready!')
    rospy.spin()