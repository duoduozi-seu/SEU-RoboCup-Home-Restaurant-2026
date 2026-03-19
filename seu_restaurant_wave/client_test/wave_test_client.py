#!/home/duo/anaconda3/envs/gpsr_perception310/bin/python
# -*- coding: utf-8 -*-
import rospy, actionlib
from seu_restaurant_msgs.msg import FindWavingCustomerAction, FindWavingCustomerGoal

rospy.init_node('test_find_waving_customer')
client = actionlib.SimpleActionClient('find_waving_customer', FindWavingCustomerAction)
client.wait_for_server()

goal = FindWavingCustomerGoal()
goal.timeout_sec = 20.0
goal.angular_speed_rad_s = 0.15

client.send_goal(goal)
client.wait_for_result()
print(client.get_result())
