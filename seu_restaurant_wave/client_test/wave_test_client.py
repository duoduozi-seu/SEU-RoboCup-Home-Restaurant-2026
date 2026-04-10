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
result = client.get_result()
print("success:", getattr(result, "success", False))
print("person count:", len(getattr(result, "person_points_map", [])))
for index, point in enumerate(getattr(result, "person_points_map", []), start=1):
    print("person {}: ({:.3f}, {:.3f}, {:.3f})".format(index, point.x, point.y, point.z))
