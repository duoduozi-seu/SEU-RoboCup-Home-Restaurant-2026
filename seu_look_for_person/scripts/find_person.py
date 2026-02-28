#! /home/syw/anaconda3/envs/GPSR/bin/python
# -*- coding: UTF-8 -*-

import rospy
import tf2_ros
import actionlib
from smach import State, StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import * 
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped, Pose, Quaternion, Twist
from tf2_geometry_msgs import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

import math
#import face_recognition

from seu_robot_msgs.msg import object_position
from seu_look_for_person.srv import *

seuhome_find_person_name = 'find_person_service'
rospy.init_node('FIND_PERSON_NODE')

move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
rospy.loginfo("Waiting for move_base action server...")  
move_base.wait_for_server(rospy.Duration(60))  
rospy.loginfo("Connected to move base server") 

buffer = tf2_ros.Buffer()
sub = tf2_ros.TransformListener(buffer)
   
class ExploreRoom(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'timeout'])
        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)              
        self.camera_state_pub = rospy.Publisher('/findperson_camera_state', String, queue_size=10)

        rospy.Subscriber('/find_person_result', object_position, self.pose_callback)

        self.timeout = False
        self.is_find_person = False

        #旋转条件改变后就可去掉
        self.rotate_count = 0
        self.camera_count = 0

    def execute(self, userdata):
        rospy.logerr("######### ExploreRoom #########")

        # Issue robot rotation command
        cmd_vel = Twist()
        cmd_vel.angular.z = -0.18

        camera_state = String()
        camera_state.data = "camera_open"

        # Waiting for object detection to complete
        while not rospy.is_shutdown():

            #旋转条件改变后就可去掉
            self.rotate_count = self.rotate_count + 1
            rospy.loginfo("rotate_count:%s", self.rotate_count)

            if self.is_find_person:
                rospy.loginfo("is_find_person:%s", self.is_find_person)
                break
            #之后旋转条件需要改成转到原始朝向就停止
            elif self.rotate_count > 300:
                rospy.logwarn('No Person !!!!!!')
                self.timeout = True                
                break
                
            rospy.sleep(0.1)
            self.cmd_vel_pub.publish(cmd_vel)
            self.camera_state_pub.publish(camera_state)


        #停止旋转和识别
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

        camera_state = String()
        camera_state.data = "camera_close"
        self.camera_state_pub.publish(camera_state)

        #记录当前朝向，用于误识别情况下重新旋转
        # self.current_pose = rospy.wait_for_message('/robot_pose', PoseStamped)
        # global start_twist_pose
        # start_twist_pose.orientation.x = self.current_pose.pose.orientation.x
        # start_twist_pose.orientation.y = self.current_pose.pose.orientation.y
        # start_twist_pose.orientation.z = self.current_pose.pose.orientation.z
        # start_twist_pose.orientation.w = self.current_pose.pose.orientation.w
        rospy.loginfo('Start_twist_pose save success')

        self.rotate_count = 0
        self.camera_count = 0
        self.is_find_person = False

        if self.timeout:
            self.timeout = False
            return 'timeout'
        else:
            return 'success'


    def pose_callback(self, msg):
        global ps_out

        if msg.label == 'person':
            ps = tf2_geometry_msgs.PointStamped()
            ps.header.stamp = rospy.Time()
            ps.header.frame_id = "camera1_color_optical_frame"
            ps.point.x = msg.x
            ps.point.y = msg.y
            ps.point.z = msg.z

            ps_out = buffer.transform(ps,"map")

            self.camera_count = self.camera_count + 1
            if self.camera_count > 3:
                self.is_find_person = True
            #需要改成根据ps_out的x,y结果判断是否在场地中，判断是否真的找到了人
            


class FaceToPerson(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

        rospy.Subscriber('/robot_pose', PoseStamped, self.pose_callback)
        
        # Initialize robot current pose and target point
        self.current_pose = PoseStamped()
        self.target_point = PoseStamped()


    def execute(self, userdata):
        rospy.logerr("######### FaceToPerson #########")

        #获取机器人当前位置
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.start_pose = rospy.wait_for_message('/robot_pose', PoseStamped)
        rospy.loginfo("Start_pose:%s,%s", self.start_pose.pose.position.x, self.start_pose.pose.position.y)
        
        #获取机器人目标位置
        self.target_point.pose.position.x = ps_out.point.x
        self.target_point.pose.position.y = ps_out.point.y
        rospy.loginfo("Target_pose:%s,%s", self.target_point.pose.position.x, self.target_point.pose.position.y)
    
        while not rospy.is_shutdown():
            # Calculate the direction from robot current position to target point
            target_yaw = math.atan2(self.target_point.pose.position.y - self.start_pose.pose.position.y,
                                    self.target_point.pose.position.x - self.start_pose.pose.position.x)
            
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
            if abs(yaw_error) < 0.05:
                cmd_vel_stop = Twist()
                cmd_vel_stop.linear.x = 0
                cmd_vel_stop.angular.z = 0

                cmd_vel_pub.publish(cmd_vel_stop)
                break

            # Calculate the robot control command
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.3 * yaw_error / abs(yaw_error)

            # Publish the robot control command
            cmd_vel_pub.publish(cmd_vel)

            # Wait for a while to let the robot rotate to the new direction
            rospy.sleep(0.1)

        return 'success'

    def pose_callback(self, msg):
        # Update robot current pose
        self.current_pose = msg


class MoveToPerson(State):
    def __init__(self):
        
        #timeout状态并没有用到
        State.__init__(self, outcomes=['success', 'timeout'])

        # Subscribe to robot current pose topic
        rospy.Subscriber('/robot_pose', PoseStamped, self.pose_callback)

        # Initialize robot current pose and target point
        self.current_pose = PoseStamped()
        self.target_point = Point()
        
    def execute(self, userdata):
        rospy.logerr("######### MoveToPerson #########")
        
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.start_pose = rospy.wait_for_message('/robot_pose', PoseStamped)
        rospy.loginfo("Start_pose:%s,%s", self.start_pose.pose.position.x, self.start_pose.pose.position.y) 
        

        while not rospy.is_shutdown():

            #找人部分感觉误识别的概率比较低，先不回到上个位置继续旋转

            #给定目标位置
            self.target_point.x = ps_out.point.x
            self.target_point.y = ps_out.point.y
            rospy.loginfo("Target_pose:%s,%s", self.target_point.x, self.target_point.y) 

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
            if abs(yaw_error) < 0.05:
                cmd_vel_stop = Twist()
                cmd_vel_stop.linear.x = 0
                cmd_vel_stop.angular.z = 0

                cmd_vel_pub.publish(cmd_vel_stop)
                break

            # Calculate the robot control command
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.3 * yaw_error / abs(yaw_error)

            # Publish the robot control command
            cmd_vel_pub.publish(cmd_vel)

            # Wait for a while to let the robot rotate to the new direction
            rospy.sleep(0.1)
        
        while not rospy.is_shutdown():
            distance = math.sqrt(pow(self.target_point.x - self.current_pose.pose.position.x, 2) +  
                            pow(self.target_point.y - self.current_pose.pose.position.y, 2)) 

            # laser_msg = rospy.wait_for_message('/scan', LaserScan)
            # if min(laser_msg.ranges[200:313]) < 0.18:
            #     rospy.logwarn('Too close!')
            #     cmd_vel_pub.publish(cmd_vel_stop)
            #     break

            # Stop sending control commands if robot is already facing the target point
            if distance < 0.6:
                rospy.logwarn('Stop!')
                cmd_vel_pub.publish(cmd_vel_stop)
                break

            # Calculate the robot control command
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = 0
            rospy.loginfo("distance:%s", distance)

            # Publish the robot control command
            cmd_vel_pub.publish(cmd_vel)

            # Wait for a while to let the robot rotate to the new direction
            rospy.sleep(0.1) 

        return 'success'
    
    def pose_callback(self, msg):
        # Update robot current pose
        self.current_pose = msg


def findperson_callback(req):
    rospy.loginfo('FINDING PERSON !!!!!!')
        
    result = sm.execute()
    rospy.loginfo("result:%s", result)

    response = seu_findpersonResponse()
    if result == 'Finish':
        response.errormsg = 'success'
        response.state = True
    else:
        response.errormsg = 'fail'
        response.state = False

    return response


if __name__ == "__main__":
    server = rospy.Service(seuhome_find_person_name, seu_findperson, findperson_callback)
        
    sm = StateMachine(outcomes=['Finish','Fail'])
    with sm:
        StateMachine.add('EXPLORE_ROOM', ExploreRoom(), transitions={'success': 'FACE_TO_PERSON', 'timeout': 'Fail'})
        StateMachine.add('FACE_TO_PERSON', FaceToPerson(), transitions={'success': 'MOVE_TO_PERSON'})
        StateMachine.add('MOVE_TO_PERSON', MoveToPerson(), transitions={'success': 'Finish', 'timeout': 'EXPLORE_ROOM'})

    rospy.loginfo("Find Person Main Programm Start !!!!!!!")
    rospy.spin()




