#! /usr/bin/python3
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

from seu_robot_msgs.msg import object_position
from seu_look_for_person.srv import *

seuhome_find_person_name = 'find_person_service'

#启动节点
rospy.init_node('FIND_PERSON')
#启动move_base服务
move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
rospy.loginfo("Waiting for move_base action server...")  
move_base.wait_for_server(rospy.Duration(60))  
rospy.loginfo("Connected to move base server") 
#初始化tf坐标转换
buffer = tf2_ros.Buffer()
sub = tf2_ros.TransformListener(buffer)

person_list = []
keyword = ''

class ExploreRoom(State):
    def __init__(self,count = False):
        State.__init__(self, outcomes=['success', 'timeout'])
        #ros发布初始化
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)              
        self.camera_state_pub = rospy.Publisher('/findperson_camera_state', String, queue_size=10)
        #ros接收初始化
        rospy.Subscriber('/find_person_result', object_position, self.person_callback)
        #初始化参数
        self.timeout = False
        self.is_find_person = False

        #旋转条件改变后就可去掉
        self.rotate_count = 0
        self.camera_count = 0
        self.count = count

    def execute(self, userdata):
        rospy.logerr("######### ExploreRoom #########")
        person_list = []
        #设置旋转速度
        cmd_vel = Twist()
        cmd_vel.angular.z = 0.18

        #打开摄像头:根据特征确定摄像头载入的模型
        camera_state = String()
        if feature == 'wave' or feature == 'raise_left' or feature == 'point_left' or feature == 'raise_right' or feature == 'point_right' :
            camera_state.data = "camera_feature"
        else:
            camera_state.data = "camera_open"
        rospy.sleep(3)
        # Waiting for object detection to complete
        while not rospy.is_shutdown():

            #旋转计数
            self.rotate_count = self.rotate_count + 1
            #rospy.loginfo("rotate_count:%s", self.rotate_count)
            print(self.count)
            if self.is_find_person and not self.count:
                rospy.loginfo("is_find_person:%s", self.is_find_person)
                break
            #之后旋转条件需要改+成转到原始朝向就停止
            elif self.rotate_count > 350:
                rospy.logwarn('No Person !!!!!!')
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
        self.rotate_count = 0
        self.camera_count = 0
        self.is_find_person = False
        if self.count:
            rospy.set_param("person_count",len(self.person_list))
        else:
            if self.timeout:
                self.timeout = False
                return 'timeout'
            else:
                return 'success'

    #判断该人是否曾经被识别：用于防止一直识别干扰人
    def in_list(self, ps):
        is_in_list_flag = False

        for location in person_list:
            if math.sqrt((location.point.x-ps.point.x)**2+(location.point.y-ps.point.y)**2) < 0.5:
                is_in_list_flag = True
                break
        
        return is_in_list_flag

    def person_callback(self, msg):
        global ps_out

        if feature == 'wave' or feature == 'raise_left' or feature == 'point_left' or feature == 'raise_right' or feature == 'point_right' :
            if msg.label == feature:
                ps = tf2_geometry_msgs.PointStamped()
                 #设置原坐标相关参数：时间戳、原坐标系、坐标
                ps.header.stamp = rospy.Time()
                ps.header.frame_id = "camera1_color_optical_frame"
                ps.point.x = msg.x
                ps.point.y = msg.y
                ps.point.z = msg.z
                #tf坐标转换：指定转换后坐标系
                ps_out = buffer.transform(ps,"map")
                rospy.loginfo('%s:(%f,%f,%f)' %(msg.label,ps.point.x,ps.point.y,ps.point.z))
                if self.in_list(ps_out) is False:
                    self.camera_count = self.camera_count + 1
                if self.camera_count > 3:
                    person_list.append(ps_out)
                    #需要改成根据ps_out的x,y结果判断是否在场地中，判断是否真的找到物体
                    self.is_find_person = True
        else:
            if msg.label == 'person':
                ps = tf2_geometry_msgs.PointStamped()
                ps.header.stamp = rospy.Time()
                ps.header.frame_id = "camera1_color_optical_frame"
                ps.point.x = msg.x
                ps.point.y = msg.y
                ps.point.z = msg.z

                ps_out = buffer.transform(ps,"map")
                if self.in_list(ps_out) is False:
                    self.camera_count = self.camera_count + 1

                if self.camera_count > 3:
                    person_list.append(ps_out)
                    self.is_find_person = True


class FaceToPerson(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'timeout'])
        #ros发布初始化
        self.camera_state_pub = rospy.Publisher('/findperson_camera_state', String, queue_size=10)
        #ros接收初始化
        rospy.Subscriber('/robot_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/find_person_result', object_position, self.person_callback)
        
        # 初始化当前位置和目标位置
        self.current_pose = PoseStamped()
        self.target_point = PoseStamped()
        #初始化参数
        self.timeout = False
        self.is_find_feature = False

        #等待识别
        self.wait_count = 0


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
            self.wait_count = self.wait_count + 1
            if self.is_find_feature:
                break
            elif self.wait_count > 50:
                rospy.logwarn('No %s',feature)
                self.timeout = True                
                break


        #寻找性别特征需要朝向物体进一步确认
        if feature == 'woman' or feature == 'man':
            camera_state = String()
            camera_state.data = "camera_analyze"
            self.wait_count = 0
            # Waiting for object detection to complete
            while not rospy.is_shutdown():
                self.wait_count = self.wait_count + 1

                if self.is_find_feature:
                    break
                elif self.wait_count > 50:
                    rospy.logwarn('No %s',feature)
                    self.timeout = True                
                    break
                    
                rospy.sleep(0.1)
                self.camera_state_pub.publish(camera_state)
            #关闭摄像头
            camera_state = String()   
            camera_state.data = "camera_close"
            self.camera_state_pub.publish(camera_state)
        #参数重置
        self.wait_count = 0
        self.is_find_feature = False

        if self.timeout:
            self.timeout = False
            return 'timeout'
        else:
            return 'success'

    def pose_callback(self, msg):
        # Update robot current pose
        self.current_pose = msg


    def person_callback(self, msg):
        global ps_out
        if msg.label == feature:
                self.is_find_feature = True
        else:
            '''当同一张照片出现多个人时,这个else会不会出问题建议多测'''
            self.is_find_feature = False





class MoveToPerson(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'timeout'])

        #ros接收初始化
        rospy.Subscriber('/robot_pose', PoseStamped, self.pose_callback)

        #初始化参数
        self.current_pose = PoseStamped()
        self.target_point = Point()
        
    def execute(self, userdata):
        rospy.logerr("######### MoveToPerson #########")
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

#关键词提取函数
def keyword_extraction(msg):
    global keyword
    #转变为小写
    msg = msg.lower()  

    #提取关键词
    if 'woman' in msg or 'female' in msg or 'girl' in msg or 'lady' in msg:
        #keyword = 'woman'  
        keyword = 'person'
    elif 'man' in msg or 'male' in msg or 'boy' in msg:
        #keyword = 'man'
        keyword = 'person'
    elif 'wave' in msg:
        keyword = 'wave'
    elif 'left' in msg:
        if 'raise' in msg:
            keyword = 'raise_left'
        elif 'point' in msg:
            keyword = 'point_left'
    elif 'right' in msg:
        if 'raise' in msg:
            keyword = 'raise_right'
        elif 'point' in msg:
            keyword = 'point_right'
    else:
        keyword = 'person'
    keyword = 'person'
    rospy.logwarn('%s',keyword)
    return keyword


#服务回调函数
def findperson_callback(req):
    global feature
    feature = keyword_extraction(req.feature)
    count = req.count
    rospy.loginfo('FINDING PERSON !!!!!!')

    #清空人物列表，防止多次调用不识别该位置的人
    person_list.clear()
    #运行状态机
    if count == "count":
        result = sm2.execute()
    else:
        result = sm.execute()
    rospy.loginfo("result:%s", result)
    #根据状态机运行结果应答
    response = seu_findpersonResponse()
    if result == 'Finish':
        response.errormsg = 'success'
        response.state = True
    else:
        response.errormsg = 'fail'
        response.state = False

    return response


if __name__ == "__main__":
    #ros启动服务
    server = rospy.Service('find_person_service', seu_findperson, findperson_callback)
    #ros初始化状态机
    sm = StateMachine(outcomes=['Finish','Fail'])
    with sm:
        StateMachine.add('EXPLORE_ROOM', ExploreRoom(), transitions={'success': 'FACE_TO_PERSON', 'timeout': 'Fail'})
        StateMachine.add('FACE_TO_PERSON', FaceToPerson(), transitions={'success': 'MOVE_TO_PERSON', 'timeout': 'EXPLORE_ROOM'})
        StateMachine.add('MOVE_TO_PERSON', MoveToPerson(), transitions={'success': 'Finish', 'timeout': 'EXPLORE_ROOM'})
    sm2 = StateMachine(outcomes=['Finish','Fail'])
    with sm2:
        StateMachine.add('EXPLORE_ROOM', ExploreRoom(True), transitions={'success': 'Finish', 'timeout': 'Fail'})
    rospy.loginfo("Find Person Main Programm Start !!!!!!!")
    #循环接收消息
    rospy.spin()




