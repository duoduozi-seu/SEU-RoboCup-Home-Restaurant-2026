#! /home/syw/anaconda3/envs/GPSR/bin/python
# -*- coding: UTF-8 -*-
import sys
import time

import numpy as np
import rospy
import tf2_ros
import actionlib
from pybullet_examples.jacobian import result
from smach import State, StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped, Pose, Quaternion, Twist
from sympy import false
from tf2_geometry_msgs import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

import math
from seu_robot_msgs.msg import object_position
from seu_look_for_object.srv import *

seuhome_find_object_name = 'find_object_service'

#启动节点
rospy.init_node('FIND_OBJECT')
#启动move_base服务
move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
rospy.loginfo("Waiting for move_base action server...")  
move_base.wait_for_server(rospy.Duration(60))  
rospy.loginfo("Connected to move base server") 
#初始化tf坐标转换
buffer = tf2_ros.Buffer()
sub = tf2_ros.TransformListener(buffer)
ps_out = None

food_list = ['chip', 'biscuit', 'bread','cookie', 'lays']
drink_list = ['sprite', 'cola', 'water']
cleaning_stuff = ['dishsoap', 'handwash', 'shampoo']
def is_target_kind(kind,obj):
    if kind == 'food':
        return obj.lower() in food_list
    elif kind == 'drink':
        return obj.lower() in drink_list
    elif kind == 'cleaning stuff':
        return obj.lower() in cleaning_stuff
    elif kind.lower() in ['object','objects','any','good','goods','things','thing','item','items']:
        return True
    elif kind == obj:
        return True
    else:
        # 如果kind不是上述任何一种，可以返回False或抛出异常
        return False

def get_info(key):
    try:
        result = rospy.get_param(key)
    except:
        result = None
    return result

class ObjAnalyse:
    def __init__(self):
        self.object = None
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.camera_state_pub = rospy.Publisher('/findobject_camera_state', String, queue_size=10)
        #ros接收初始化
        rospy.Subscriber('/find_object_result', object_position, self.object_callback)
        self.feature = ""
        self.target_p = ""
        self.obj_list = {}
        self.count = {}

    def obj_analyse(self,feature):
        #开启相机
        camera_state = String()
        camera_state.data = "camera_open"
        self.camera_state_pub.publish(camera_state)
        self.feature = feature
        if feature == "count":
            rospy.set_param("obj_count", "none")
            res = get_info("obj_count")
            t = 1
            while res == "none" and t < 300:
                rospy.sleep(0.1)
                t = t + 1
                res = get_info("obj_count")
            if t < 300:
                rospy.loginfo(f"obj counted is {res}")
                rospy.set_param("obj_feature", "none")
                result = True
            else:
                result = False
        elif feature in ["biggest", "largest", "smallest", "heaviest", "lightest", "thinnest"]:
            rospy.set_param("obj_name", "none")
            res = get_info("obj_name")
            t = 1
            while res == "none" and t < 300:
                rospy.sleep(0.1)
                t = t + 1
                res = get_info("obj_name")
            if t < 300:
                rospy.loginfo(f"the target obj is {res}")
                rospy.set_param("obj_feature", "none")
                result = True
            else:
                result = False

        elif feature in ["left", "right", 'top', "above", 'behind', 'under']:
            rospy.set_param("obj_name", "none")
            res = get_info("obj_name")
            t = 1
            while res == "none" and t < 300:
                rospy.sleep(0.1)
                t = t + 1
                res = get_info("obj_name")
            if t < 300:
                rospy.loginfo(f"the target obj is {res}")
                rospy.set_param("obj_feature", "none")
                rospy.loginfo('FACING %s', object)
                result = sm2.execute()
            else:
                result = False

        elif feature in ['leftmost', 'rightmost']:
            rospy.set_param("obj_name", "none")
            res = get_info("obj_name")
            t = 1
            while res == "none" and t < 300:
                rospy.sleep(0.1)
                t = t + 1
                res = get_info("obj_name")
            if t < 300:
                rospy.loginfo(f"the target obj is {res}")
                rospy.set_param("obj_feature", "none")
                rospy.loginfo('FACING %s', object)
                result = sm2.execute()
            else:
                result = False
        else:
            rospy.loginfo('FINDING %s', object)
            # 运行状态机
            result = sm.execute()
        #关闭摄像头
        camera_state = String()
        camera_state.data = "camera_close"

        self.camera_state_pub.publish(camera_state)
        return result
    def object_callback(self, msg):
        global ps_out
        global object
        if self.feature != "none":
            ps = tf2_geometry_msgs.PointStamped()
            # 设置原坐标相关参数：时间戳、原坐标系、坐标
            ps.header.stamp = rospy.Time()
            ps.header.frame_id = "camera1_color_optical_frame"
            ps.point.x = msg.x
            ps.point.y = msg.y
            ps.point.z = msg.z
            # tf坐标转换：指定转换后坐标系
            ps_out = buffer.transform(ps, "map")
            object = msg.label



#如果因为误识别再次旋转，感觉再次误识别的概率非常高
class ExploreRoom(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'timeout'])
        #ros发布初始化
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)              
        self.camera_state_pub = rospy.Publisher('/findobject_camera_state', String, queue_size=10)
        #ros接收初始化
        rospy.Subscriber('/find_object_result', object_position, self.object_callback)

        #初始化参数
        self.timeout = False
        self.is_find_object = False

        self.object_count = 0
        #旋转条件改变后就可去掉
        self.rotate_count = 0

    def execute(self, userdata):
        rospy.logerr("######### ExploreRoom #########")

        #设置旋转速度
        cmd_vel = Twist()

        #打开摄像头
        camera_state = String()
        camera_state.data = "camera_open"
        cmd_vel.angular.z = 0.5
        self.cmd_vel_pub.publish(cmd_vel)
        time.sleep(4)

        cmd_vel.angular.z = -0.18
        self.is_find_object = False
        
        #找物
        while not rospy.is_shutdown():

            #旋转计数
            self.rotate_count = self.rotate_count + 1
            #rospy.loginfo('rotate_count:%s',self.rotate_count)

            if self.is_find_object:
                break

            #之后旋转条件建议改成转到原始朝向就停止
            elif self.rotate_count > 200:
                rospy.logwarn('No %s',object)
                self.timeout = True                
                break

            #ros消息发送
            rospy.sleep(0.1)
            self.cmd_vel_pub.publish(cmd_vel)
            self.camera_state_pub.publish(camera_state)


        #停止旋转
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)
        #关闭摄像头
        camera_state = String()
        camera_state.data = "camera_close"
        self.camera_state_pub.publish(camera_state)

        #参数重置
        self.object_count = 0
        self.rotate_count = 0
        self.is_find_object = False

        if self.timeout:
            self.timeout = False
            return 'timeout'
        else:
            return 'success'


    def object_callback(self, msg):
        global ps_out
        #rospy.loginfo('object_count:%s',self.object_count)
        #rospy.loginfo('object:%s',object)
        if msg.label == object:
            self.object_count = self.object_count + 1
            #用于防止单帧误识别
            if self.object_count > 5:
                ps = tf2_geometry_msgs.PointStamped()
                #设置原坐标相关参数：时间戳、原坐标系、坐标
                ps.header.stamp = rospy.Time()
                ps.header.frame_id = "camera1_color_optical_frame"
                ps.point.x = msg.x
                ps.point.y = msg.y
                ps.point.z = msg.z
                #tf坐标转换：指定转换后坐标系
                ps_out = buffer.transform(ps,"map")
                #需要改成根据ps_out的x,y结果判断是否在场地中，判断是否真的找到物体
                self.is_find_object = True
        # else:
        #     '''
        #     当同一张照片出现多个物体时,这个else会不会出问题建议多测
        #     情况如下:
        #         画面存在两个物体,物体A满足条件,连续5帧图像,识别先发送物体A信息,再发送物体B信息,
        #         当物体A的回传信息触发is_find_object = True后,物体B的信息会将其重新变为False
        #         考虑要将这个else删除
        #         '''
        #     self.is_find_object = False
            

class FaceToObject(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'timeout'])
        #ros发布初始化
        self.camera_state_pub = rospy.Publisher('/findobject_camera_state', String, queue_size=10)
        #ros接收初始化
        rospy.Subscriber('/robot_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/find_object_result', object_position, self.object_callback)

        # 初始化当前位置和目标位置
        self.current_pose = PoseStamped()
        self.target_point = PoseStamped()

        #初始化参数
        self.timeout = False
        self.is_find_object = False

        self.object_count = 0
        #等待识别
        self.wait_count = 0

    def execute(self, userdata):
        rospy.logerr("######### FaceToObject #########")
        #打开摄像头
        camera_state = String()
        camera_state.data = "camera_open"
        self.camera_state_pub.publish(camera_state)

        #ros发布初始化
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #获取机器人当前位置
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

        #打开摄像头
        camera_state = String()
        camera_state.data = "camera_open"

        #朝向物体时，进一步确认
        #实践证明：探索过程中的误识别概率较高，需要摄像头视角正对物体进行确认
        while not rospy.is_shutdown():
            self.wait_count = self.wait_count + 1

            if self.is_find_object:
                break
            elif self.wait_count > 400:
                rospy.logwarn('No %s',object)
                self.timeout = True                
                break
                
            rospy.sleep(0.1)
            self.camera_state_pub.publish(camera_state)
        #关闭摄像头
        camera_state = String()
        camera_state.data = "camera_close"
        self.camera_state_pub.publish(camera_state)
        #参数重置
        self.object_count = 0
        self.wait_count = 0
        self.is_find_object = False

        if self.timeout:
            self.timeout = False
            return 'timeout'
        else:
            return 'success'
    #获取当前机器人位置
    def pose_callback(self, msg):
        # Update robot current pose
        self.current_pose = msg

    def object_callback(self, msg):
        global ps_out
        if msg.label == object or is_target_kind(object,msg.label):
            self.object_count = self.object_count + 1
            #用于防止单帧误识别
            if self.object_count > 10:
                #同探索房间tf转换处理
                ps = tf2_geometry_msgs.PointStamped()
                ps.header.stamp = rospy.Time()
                ps.header.frame_id = "camera1_color_optical_frame"
                ps.point.x = msg.x
                ps.point.y = msg.y
                ps.point.z = msg.z

                ps_out = buffer.transform(ps,"map")
                #需要改成根据ps_out的x,y结果判断是否在场地中，判断是否真的找到物体
                self.is_find_object = True
                obj_pos_data = {
                    'label':msg.label,
                    'x': msg.x,
                    'y': msg.y,
                    'z': msg.z,
                    # 其他你需要的字段
                }
                rospy.set_param("obj_pos", obj_pos_data)
        # else:
        #     '''
        #     当同一张照片出现多个物体时,这个else会不会出问题建议多测
        #     情况如下:
        #         画面存在两个物体,物体A满足条件,连续5帧图像,识别先发送物体A信息,再发送物体B信息,
        #         当物体A的回传信息触发is_find_object = True后,物体B的信息会将其重新变为False
        #         考虑要将这个else删除
        #         '''
        #     self.is_find_object = False


class MoveToObject(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'timeout'])

        #ros接收初始化
        rospy.Subscriber('/robot_pose', PoseStamped, self.pose_callback)

        #初始化参数
        self.current_pose = PoseStamped()
        self.target_point = Point()
        
    def execute(self, userdata):
        rospy.logerr("######### MoveToOej #########")
        #ros发布初始化
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #获取机器人当前位置
        self.start_pose = rospy.wait_for_message('/robot_pose', PoseStamped)
        rospy.loginfo("Start_pose:%s,%s", self.start_pose.pose.position.x, self.start_pose.pose.position.y) 


        while not rospy.is_shutdown():
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

            # Stop sending control commands if robot is already facing the target point
            if distance < 0.5:
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

#服务回调函数
def findobject_callback(req):
    global object
    global feature
    object = req.object
    feature = req.feature
    rospy.set_param("obj_target", object)
    rospy.set_param("obj_feature",feature)
    result = obj_analyse.obj_analyse(feature)
    #根据状态机运行结果应答
    response = seu_findobjectResponse()
    if result == 'Finish' or result == True:
        response.errormsg = 'success'
        response.state = True
    else:
        response.errormsg = 'fail'
        response.state = False

    return response

if __name__ == "__main__":
    #ros启动服务
    server = rospy.Service(seuhome_find_object_name, seu_findobject, findobject_callback)
    obj_analyse = ObjAnalyse()
    rospy.set_param("obj_feature","none")
    #ros初始化状态机
    sm = StateMachine(outcomes=['Finish','Fail'])
    with sm:
        StateMachine.add('EXPLORE_ROOM', ExploreRoom(), transitions={'success': 'FACE_TO_OBJECT', 'timeout': 'Fail'})
        StateMachine.add('FACE_TO_OBJECT', FaceToObject(), transitions={'success': 'MOVE_TO_OBJECT', 'timeout': 'EXPLORE_ROOM'})
        StateMachine.add('MOVE_TO_OBJECT', MoveToObject(), transitions={'success': 'Finish', 'timeout': 'EXPLORE_ROOM'})

    sm2 = StateMachine(outcomes=['Finish','Fail'])
    with sm2:
        StateMachine.add('FACE_TO_OBJECT', FaceToObject(), transitions={'success': 'MOVE_TO_OBJECT', 'timeout': 'FACE_TO_OBJECT'})
        StateMachine.add('MOVE_TO_OBJECT', MoveToObject(), transitions={'success': 'Finish', 'timeout': 'FACE_TO_OBJECT'})
    rospy.loginfo("Find Object Main Programm Start !!!!!!!")
    #循环接收消息
    rospy.spin()
