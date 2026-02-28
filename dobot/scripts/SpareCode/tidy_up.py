#!/usr/bin/env python
import rospy
import math
import tf
import actionlib
from math import pow, sqrt  
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import *  
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped, Pose, Quaternion, Twist
from smach import State, StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from robot_msgs.msg import tidyup_chassis, object_position
import signal
import threading
from sensor_msgs.msg import LaserScan
import numpy as np

def distance(x1, y1, x2, y2, x, y):
    return abs((y2 - y1) * x - (x2 - x1) * y + x2 * y1 - y2 * x1) / math.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)

def nearest_line(points, x, y):
    min_distance = float('inf')
    nearest_line_index = -1
    for i in range(4):
        x1, y1 = points[i]
        x2, y2 = points[(i+1)%4]
        d = distance(x1, y1, x2, y2, x, y)
        if d < min_distance:
            min_distance = d
            nearest_line_index = i
    return nearest_line_index

def dining_room_nearest_line(points1, points2, x, y):
    min_distance = float('inf')
    nearest_line_index = -1
    for i in range(4):
        x1, y1 = points1[i]
        x2, y2 = points1[(i+1)%4]
        d = distance(x1, y1, x2, y2, x, y)
        if d < min_distance:
            min_distance = d
            nearest_line_index = i
    for i in range(4):
        x1, y1 = points2[i]
        x2, y2 = points2[(i+1)%4]
        d = distance(x1, y1, x2, y2, x, y)
        if d < min_distance:
            min_distance = d
            nearest_line_index = i + 4
    return nearest_line_index

def find_point_on_line(a_pose, b_pose):
    a_x, a_y = a_pose.pose.position.x, a_pose.pose.position.y
    b_x, b_y = b_pose.pose.position.x, b_pose.pose.position.y

    d = ((b_x - a_x) ** 2 + (b_y - a_y) ** 2) ** 0.5

    x = a_x + 0.5 * (b_x - a_x) / d
    y = a_y + 0.5 * (b_y - a_y) / d

    theta = math.atan2(a_y - b_y, a_x - b_x)

    point_pose = PoseStamped()
    point_pose.pose.position.x = x
    point_pose.pose.position.y = y
    quaternion = quaternion_from_euler(0, 0, theta)
    point_pose.pose.orientation.x = quaternion[0]
    point_pose.pose.orientation.y = quaternion[1]
    point_pose.pose.orientation.z = quaternion[2]
    point_pose.pose.orientation.w = quaternion[3]

    return point_pose

def signal_handler(signal, frame):
    for thread in threading.enumerate():
        if thread.is_alive():
            thread.do_runn = False
        rospy.loginfo('exit')
        sys.exit(0)
    rospy.shutdown('ROS shutdown')

rospy.init_node('room_explorer')

move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
rospy.loginfo("Waiting for move_base action server...")  
move_base.wait_for_server(rospy.Duration(60))  
rospy.loginfo("Connected to move base server") 

Chassis_State_Stop = 0
Chassis_State_Room = 1
Chassis_State_Rotate = 2
Chassis_State_Towards = 3
Chassis_State_Catch = 4
Chassis_State_Place = 5
Chassis_State_Finish = 6

Arrive_Room = 1
Finish_Face = 2
Approach_Item = 3
Arrive_Place = 4

cmd_vel_stop = Twist()
cmd_vel_stop.linear.x = 0
cmd_vel_stop.angular.z = 0

corner_points = {
    'Kitchen_1':[(1, -2), (4, -2), (4, 3), (1, 3)],
    'Kitchen_2':[(1, -2), (4, -2), (4, 3), (1, 3)],
    'Bedroom_1': [(1, -2), (4, -2), (4, 3), (1, 3)],
    'Bedroom_2': [(1, -2), (4, -2), (4, 3), (1, 3)],
    # 'Livingroom_1': [(1, -2), (4, -2), (4, 3), (1, 3)],
    # 'Livingroom_2': [(1, -2), (4, -2), (4, 3), (1, 3)],
    # 'Diningroom_1': [(0, -5.78), (4.32, -5.78), (4.32, 1.20), (0, 1.20)],
    # 'Diningroom_2': [(0, -5.78), (4.32, -5.78), (4.32, 1.20), (0, 1.20)],
    }

table_points = [(1.52, -3.36), (2.14, -3.36), (2.14, -2.04), (1.52, -2.04)]

# Define target rooms and corresponding positions and orientations
target_poses = { 
    'Kitchen_1': [(1.59, -2.03), (0.0, 0.0, -0.725, 0.688)],
    'Kitchen_2': [(1.82, -0.83), (0.0, 0.0, 0.681, 0.732)],
    'Bedroom_1': [(2.26, 1.24), (0.0, 0.0, 0.999, 0.033)],
    'Bedroom_2': [(1.54, 1.29), (0.0, 0.0, 0.999, 0.027)],
    # 'Livingroom_1': [(0, 0.5), (0.0, 0.0, 0.0, 1.0)],
    # 'Livingroom_2': [(2.5, -0.2), (0.0, 0.0, 0.707, -0.707)],
    # 'Diningroom_1': [(2.76, -0.77), (0.0, 0.0, -0.350, 0.936)],
    # 'Diningroom_2': [(2.02, -4.14), (0.0, 0.0, -0.810, 0.580)],
    'Trash': [(1.59, -1.55), (0.0, 0.0, 0.955, -0.293)],
    'Biscuit_Place': [(2.26, 1.25), (0.0, 0.0, 0.707, -0.707)],     # Nescafe
    'Chip_Place': [(-0.5, 0), (0.0, 0.0, 0.707, -0.707)],
    'Lays_Place': [(1.59,  -2.03), (0.0, 0.0, 0.0, 1.0)],       # fan
    'Bread_Place': [(1.82,  -0.83), (0.0, 0.0, 0.0, 1.0)],      # water glass
    'Cookie_Place': [(3.2,  -2.4), (0.0, 0.0, 0.0, 1.0)],
    'Handwash_Place': [(1.0, -2.4), (0.0, 0.0, 0.0, 1.0)],
    'Dishsoap_Place': [(-0.5, 0), (0.0, 0.0, 0.707, -0.707)],
    'Water_Place': [(-0.5, 0), (0.0, 0.0, 0.707, -0.707)],
    'Sprite_Place': [(-0.5, 0), (0.0, 0.0, 0.707, -0.707)],
    'Cola_Place': [(3.2,  -2.0), (0.0, 0.0, 0.0, 1.0)],
    'Orange_juice_Place': [(2.8, -3.0), (0.0, 0.0, 1.0, 0.0)],
    'Shampoo_Place': [(-0.5, 0), (0.0, 0.0, 0.707, -0.707)],
    }

# Global Param
work_room = None
start_twist_pose = Pose()

class Begin(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
    
        self.obstacle_detected = False
        self.initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)  
        
        
    def execute(self, userdata):
        rospy.logerr("######### Begin #########")
        while not rospy.is_shutdown():
            scan = rospy.wait_for_message('/scan', LaserScan)
            self.ranges = scan.ranges[240:273]
            
            # Convert scan data to a 2D point cloud
            line_ranges = np.array(self.ranges)
            angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))[240:273]
            points = np.vstack((line_ranges * np.cos(angles), line_ranges * np.sin(angles))).T

            # Perform linear regression on the point cloud
            x = points[:, 0]
            y = points[:, 1]
            A = np.vstack([x, np.ones(len(x))]).T
            k, b = np.linalg.lstsq(A, y)[0]

            # Compute the distance from each point to the fitted line
            distances = abs(k * x - y + b) / np.sqrt(k ** 2 + 1)

            # Count the number of points within the threshold distance
            threshold = 0.1  # Example threshold value
            self.num_points_on_line = np.sum(distances < threshold)
            
            # Detect obstacles in front
            if max(self.ranges) < 1.0 and self.num_points_on_line > 32:
                rospy.logwarn('max_range:%s', max(self.ranges))
                rospy.logwarn('num_points_on_line:%s', self.num_points_on_line)
                self.obstacle_detected = True
                # Stop moving when obstacle is detected
                rospy.loginfo('Detect the door')
            else:
                if self.obstacle_detected:
                    rospy.sleep(2)
                    rospy.logwarn('The door open')

                    self.initialpose = PoseWithCovarianceStamped()
                    self.initialpose.pose.pose = Pose(Point(0.20, 2.96, 0.00), Quaternion(0.000, 0.000, -0.707, 0.707)) 
                    self.initialpose.header.frame_id = 'map'  
                    self.initialpose.header.stamp = rospy.Time.now() 
                    for i in range(0,10):
                        self.initialpose_pub.publish(self.initialpose)
                        rospy.sleep(0.1)

                    rospy.logwarn('Begin')
                    self.obstacle_detected = False
                    return 'success'
                else:
                    rospy.loginfo('Wait...')
        
    # def laser_callback(self, scan):
    #     # Get laser scan data (513 points)
    #     ranges = scan.ranges[240:273]
    #     self.ranges = ranges

    #     # Convert scan data to a 2D point cloud
    #     line_ranges = np.array(ranges)
    #     angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))[240:273]
    #     points = np.vstack((line_ranges * np.cos(angles), line_ranges * np.sin(angles))).T

    #     # Perform linear regression on the point cloud
    #     x = points[:, 0]
    #     y = points[:, 1]
    #     A = np.vstack([x, np.ones(len(x))]).T
    #     k, b = np.linalg.lstsq(A, y)[0]

    #     # Compute the distance from each point to the fitted line
    #     distances = abs(k * x - y + b) / np.sqrt(k ** 2 + 1)

    #     # Count the number of points within the threshold distance
    #     threshold = 0.1  # Example threshold value
    #     num_points_on_line = np.sum(distances < threshold)
    #     self.num_points_on_line = num_points_on_line


class MoveToRoom(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

        # Default target room is None
        self.target_room = None

        # Subscribe to topic '/chassis_begin' to receive target room ID
        self.sub = rospy.Subscriber("/chassis_begin", tidyup_chassis, self.begin_callback)

        # Define publisher to notify when robot reaches target position
        self.pub = rospy.Publisher("/chassis_finish", tidyup_chassis, queue_size=10)

        # Flag to indicate if target position has been sent to avoid repeated sending
        self.is_send_goal = False

    def execute(self, userdata):
        rospy.logerr("######### MoveToRoom #########")

        while not rospy.is_shutdown():
            if self.target_room == None:
                rospy.logwarn("Wait for target room ID") 
                rospy.sleep(1)
                continue

            # Send target position if not sent yet
            if not self.is_send_goal:
                # Define target position
                goal = MoveBaseGoal()   
                goal.target_pose.header.frame_id = 'map'  
                goal.target_pose.header.stamp = rospy.Time.now()  
                goal.target_pose.pose.position.x = target_poses[self.target_room][0][0]
                goal.target_pose.pose.position.y = target_poses[self.target_room][0][1]
                goal.target_pose.pose.orientation.x = target_poses[self.target_room][1][0]
                goal.target_pose.pose.orientation.y = target_poses[self.target_room][1][1]
                goal.target_pose.pose.orientation.z = target_poses[self.target_room][1][2]
                goal.target_pose.pose.orientation.w = target_poses[self.target_room][1][3]
                
                # Print target room name
                rospy.loginfo("Going to: " + str(self.target_room))  

                # Send goal
                move_base.send_goal(goal)  
                self.is_send_goal = True

            # Check if robot has reached target position
            state = move_base.get_state()  
            if state == GoalStatus.SUCCEEDED:  
                # If robot has reached target position, send notification
                chassis_finish = tidyup_chassis()
                chassis_finish.state = Arrive_Room
                for i in range(0, 10):
                    self.pub.publish(chassis_finish)
                    rospy.sleep(0.1)
                self.is_send_goal = False
                self.target_room = None
                return 'success'
            
            rospy.sleep(1)
     
    def begin_callback(self, msg):
        global work_room
        if msg.state == Chassis_State_Room:
            if msg.room_id == 1:
                self.target_room = 'Kitchen_1'
            elif msg.room_id == 2:
                self.target_room = 'Kitchen_2'
            elif msg.room_id == 3:
                self.target_room = 'Bedroom_1'
            elif msg.room_id == 4:
                self.target_room = 'Bedroom_2'
            elif msg.room_id == 5:
                self.target_room = 'Livingroom_1'
            elif msg.room_id == 6:
                self.target_room = 'Livingroom_2'
            elif msg.room_id == 7:
                self.target_room = 'Diningroom_1'
            elif msg.room_id == 8:
                self.target_room = 'Diningroom_2'
            elif msg.room_id == 9:
                self.target_room = 'Trash'
            elif msg.room_id == 10:
                self.target_room = 'Biscuit_Place'
            elif msg.room_id == 11:
                self.target_room = 'Chip_Place'
            elif msg.room_id == 12:
                self.target_room = 'Lays_Place'
            elif msg.room_id == 13:
                self.target_room = 'Bread_Place'
            elif msg.room_id == 14:
                self.target_room = 'Cookie_Place'
            elif msg.room_id == 15:
                self.target_room = 'Handwash_Place'
            elif msg.room_id == 16:
                self.target_room = 'Dishsoap_Place'
            elif msg.room_id == 17:
                self.target_room = 'Water_Place'
            elif msg.room_id == 18:
                self.target_room = 'Sprite_Place'
            elif msg.room_id == 19:
                self.target_room = 'Cola_Place'
            elif msg.room_id == 20:
                self.target_room = 'Orange_juice_Place'
            elif msg.room_id == 21:
                self.target_room = 'Shampoo_Place'
            work_room = self.target_room
        else:
            self.target_room = None


class ExploreRoom(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'timeout'])

        self.pub = rospy.Publisher("/chassis_finish", tidyup_chassis, queue_size=10)
        
        self.initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)  
        
        self.timeout = False

    def execute(self, userdata):
        global twist_point
        rospy.logerr("######### ExploreRoom #########")
        self.sta_pose = rospy.wait_for_message('/robot_pose', PoseStamped)
        # Issue robot rotation command
        cmd_vel = Twist()
        if work_room == 'Diningroom_2':
            cmd_vel.angular.z = -0.18
        else:
            cmd_vel.angular.z = 0.18
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Waiting for object detection to complete
        while not rospy.is_shutdown():
            msg = rospy.wait_for_message('/chassis_begin', tidyup_chassis)
            if msg.state == Chassis_State_Towards:
                break
            elif msg.state == Chassis_State_Room:
                rospy.logerr('Timeout')
                self.timeout = True
                break
                
            rospy.sleep(0.1)
            cmd_vel_pub.publish(cmd_vel)
            
        # Issue robot stop command
        cmd_vel = Twist()
        cmd_vel_pub.publish(cmd_vel)

        self.end_pose = rospy.wait_for_message('/robot_pose', PoseStamped)

        # correct robot pose
        self.initialpose = PoseWithCovarianceStamped()
        self.initialpose.pose.pose = Pose(Point(self.start_pose.pose.position.x,
                                                self.start_pose.pose.position.y,
                                                self.start_pose.pose.position.z), 
                                        Quaternion(0.000, 0.000,
                                                self.end_pose.pose.orientation.z,
                                                self.end_pose.pose.orientation.w)) 
        self.initialpose.header.frame_id = 'map'  
        self.initialpose.header.stamp = rospy.Time.now() 
        # for i in range(0, 10):
        #     self.initialpose_pub.publish(self.initialpose)
        #     rospy.sleep(0.1)

        self.current_pose = rospy.wait_for_message('/robot_pose', PoseStamped)
        global start_twist_pose
        start_twist_pose.orientation.x = self.current_pose.pose.orientation.x
        start_twist_pose.orientation.y = self.current_pose.pose.orientation.y
        start_twist_pose.orientation.z = self.current_pose.pose.orientation.z
        start_twist_pose.orientation.w = self.current_pose.pose.orientation.w
        rospy.loginfo('Start_twist_pose save success')

        if self.timeout:
            self.timeout = False
            return 'timeout'
        else:
            return 'success'


class FaceToItem(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

        # Subscribe to topic '/chassis_begin' to receive 
        self.sub = rospy.Subscriber("/chassis_begin", tidyup_chassis, self.item_callback)
        self.sub = rospy.Subscriber("/detect_result2",object_position,self.arm_callpack,queue_size=10)
        # Define publisher to notify when robot reaches target position
        self.pub = rospy.Publisher("/chassis_finish", tidyup_chassis, queue_size=10)

        # Subscribe to robot current pose topic
        rospy.Subscriber('/robot_pose', PoseStamped, self.pose_callback)
        
        # Initialize robot current pose and target point
        self.current_pose = PoseStamped()
        self.target_point = PoseStamped()

        self.goal = MoveBaseGoal()
        self.is_send_goal = False


    def execute(self, userdata):
        rospy.logerr("######### FaceToItem #########")

        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.start_pose = rospy.wait_for_message('/robot_pose', PoseStamped)
        rospy.loginfo("Start_pose:%s,%s", self.start_pose.pose.position.x, self.start_pose.pose.position.y)
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

        for i in range(0, 10):
            chassis_finish = tidyup_chassis()
            chassis_finish.state = Finish_Face
            self.pub.publish(chassis_finish)
            rospy.sleep(0.1)

        self.is_send_goal = False
        return 'success'

    def pose_callback(self, msg):
        # Update robot current pose
        self.current_pose = msg

    def item_callback(self, msg):
        # Update robot current pose
        self.target_point.pose = msg.goal


class MoveToItem(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'timeout'])

        # Subscribe to robot current pose topic
        rospy.Subscriber('/robot_pose', PoseStamped, self.pose_callback)

        # Define publisher to notify when robot reaches target position
        self.pub = rospy.Publisher("/chassis_finish", tidyup_chassis, queue_size=10)

        # Initialize robot current pose and target point
        self.current_pose = PoseStamped()
        self.target_point = Point()
        
        #0704 plan B param
        self.move_goal = [0, 0 ,0 ,0]
        self.is_send_goal = False
        #0704 plan B param
        
    def execute(self, userdata):
        rospy.logerr("######### MoveToItem #########")
        
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.start_pose = rospy.wait_for_message('/robot_pose', PoseStamped)
        rospy.loginfo("Start_pose:%s,%s", self.start_pose.pose.position.x, self.start_pose.pose.position.y) 
        
        #0704 plan B
        while not rospy.is_shutdown():
            msg = rospy.wait_for_message("/chassis_begin", tidyup_chassis)
            
            if msg.state == Chassis_State_Rotate:
                rospy.logerr('Timeout')
                return 'timeout'
            elif msg.state != Chassis_State_Catch:
                rospy.logwarn("Wait for target pose")
                rospy.sleep(1)
                continue
            
            self.target_point.x = msg.goal.position.x
            self.target_point.y = msg.goal.position.y
            print("0000000000000000000000000",msg.goal.position.x, msg.goal.position.y)
            if work_room == 'Diningroom_1' or work_room == 'Diningroom_2':
                nearest_wall_index = dining_room_nearest_line(corner_points[work_room], table_points, self.target_point.x, self.target_point.y)
            else:
                nearest_wall_index = nearest_line(corner_points[work_room], self.target_point.x, self.target_point.y)
            print("1111111111111111111111111111111", nearest_wall_index)
            if nearest_wall_index == 0:
                self.move_goal[0] = self.target_point.x
                self.move_goal[1] = self.target_point.y + 1
                self.move_goal[2] = 0.707
                self.move_goal[3] = -0.707
            elif nearest_wall_index == 1:
                self.move_goal[0] = self.target_point.x - 1
                self.move_goal[1] = self.target_point.y
                self.move_goal[2] = 0.0
                self.move_goal[3] = 1.0
            elif nearest_wall_index == 2:
                self.move_goal[0] = self.target_point.x
                self.move_goal[1] = self.target_point.y - 1
                self.move_goal[2] = 0.707
                self.move_goal[3] = 0.707
            elif nearest_wall_index == 3:
                self.move_goal[0] = self.target_point.x + 1
                self.move_goal[1] = self.target_point.y
                self.move_goal[2] = 1.0
                self.move_goal[3] = 0.0
            elif nearest_wall_index == 4:
                self.move_goal[0] = self.target_point.x
                self.move_goal[1] = self.target_point.y - 0.3
                self.move_goal[2] = 0.707
                self.move_goal[3] = 0.707
            elif nearest_wall_index == 5:
                self.move_goal[0] = self.target_point.x + 0.3
                self.move_goal[1] = self.target_point.y
                self.move_goal[2] = 1.0
                self.move_goal[3] = 0.0
            elif nearest_wall_index == 6:
                self.move_goal[0] = self.target_point.x
                self.move_goal[1] = self.target_point.y + 0.3
                self.move_goal[2] = 0.707
                self.move_goal[3] = -0.707
            elif nearest_wall_index == 7:
                self.move_goal[0] = self.target_point.x - 0.3
                self.move_goal[1] = self.target_point.y
                self.move_goal[2] = 0.0
                self.move_goal[3] = 1.0
            print("222222222222222222222222",self.target_point.x, self.target_point.y)
            if not self.is_send_goal:
                rospy.loginfo('move_goal:%s', self.move_goal)
                # Define target position
                goal = MoveBaseGoal()   
                goal.target_pose.header.frame_id = 'map'  
                goal.target_pose.header.stamp = rospy.Time.now()  
                goal.target_pose.pose.position.x = self.move_goal[0]
                goal.target_pose.pose.position.y = self.move_goal[1]
                goal.target_pose.pose.orientation.z = self.move_goal[2]
                goal.target_pose.pose.orientation.w = self.move_goal[3]

                # Send goal
                move_base.send_goal(goal)  
                self.is_send_goal = True
                
            state = move_base.get_state()
            if state == GoalStatus.SUCCEEDED:  
                break

        self.is_send_goal = False

        while not rospy.is_shutdown():
            distance = sqrt(pow(self.target_point.x - self.current_pose.pose.position.x, 2) +  
                            pow(self.target_point.y - self.current_pose.pose.position.y, 2)) 

            laser_msg = rospy.wait_for_message('/scan', LaserScan)
            if min(laser_msg.ranges[200:313]) < 0.18:
                rospy.logwarn('Too close!')
                cmd_vel_pub.publish(cmd_vel_stop)
                break

            # Stop sending control commands if robot is already facing the target point
            if distance < 0.45:
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

        for i in range(0, 10):
            chassis_finish = tidyup_chassis()
            chassis_finish.state = Approach_Item
            self.pub.publish(chassis_finish)
            rospy.sleep(0.1)
        #0704 plan B
        
        #plan A
        # while not rospy.is_shutdown():
        #     msg = rospy.wait_for_message("/chassis_begin", tidyup_chassis)
        #     if msg.state == Chassis_State_Rotate:
        #         rospy.logerr('Timeout')
        #         return 'timeout'
        #     elif msg.state != Chassis_State_Catch:
        #         rospy.logwarn("Wait for target pose")
        #         rospy.sleep(1)
        #         continue

        #     self.target_point.x = msg.goal.position.x
        #     self.target_point.y = msg.goal.position.y
        #     rospy.loginfo("Target_pose:%s,%s", self.target_point.x, self.target_point.y)   
        #     # Calculate the direction from robot current position to target point
        #     target_yaw = math.atan2(self.target_point.y - self.start_pose.pose.position.y,
        #                             self.target_point.x - self.start_pose.pose.position.x)
        #     current_yaw = euler_from_quaternion([self.current_pose.pose.orientation.x,
        #                                         self.current_pose.pose.orientation.y,
        #                                         self.current_pose.pose.orientation.z,
        #                                         self.current_pose.pose.orientation.w])[2]

        #     yaw_error = target_yaw - current_yaw
        #     if yaw_error > math.pi:
        #         yaw_error -= 2 * math.pi
        #     elif yaw_error < -math.pi:
        #         yaw_error += 2 * math.pi

        #     # Stop sending control commands if robot is already facing the target point
        #     if abs(yaw_error) < 0.05:
        #         cmd_vel_pub.publish(cmd_vel_stop)
        #         break

        #     # Calculate the robot control command
        #     cmd_vel = Twist()
        #     cmd_vel.linear.x = 0.0
        #     cmd_vel.angular.z = 0.3 * yaw_error / abs(yaw_error)

        #     # Publish the robot control command
        #     cmd_vel_pub.publish(cmd_vel)

        #     # Wait for a while to let the robot rotate to the new direction
        #     rospy.sleep(0.1)
        
        # while not rospy.is_shutdown():
        #     distance = sqrt(pow(self.target_point.x - self.current_pose.pose.position.x, 2) +  
        #                     pow(self.target_point.y - self.current_pose.pose.position.y, 2)) 

        #     laser_msg = rospy.wait_for_message('/scan', LaserScan)
        #     if min(laser_msg.ranges[200:313]) < 0.18:
        #         rospy.logwarn('Too close!')
        #         cmd_vel_pub.publish(cmd_vel_stop)
        #         break

        #     # Stop sending control commands if robot is already facing the target point
        #     if distance < 0.45:
        #         rospy.logwarn('Stop!')
        #         cmd_vel_pub.publish(cmd_vel_stop)
        #         break

        #     # Calculate the robot control command
        #     cmd_vel = Twist()
        #     cmd_vel.linear.x = 0.1
        #     cmd_vel.angular.z = 0
        #     rospy.loginfo("distance:%s", distance)

        #     # Publish the robot control command
        #     cmd_vel_pub.publish(cmd_vel)

        #     # Wait for a while to let the robot rotate to the new direction
        #     rospy.sleep(0.1) 

        # for i in range(0, 10):
        #     chassis_finish = tidyup_chassis()
        #     chassis_finish.state = Approach_Item
        #     self.pub.publish(chassis_finish)
        #     rospy.sleep(0.1)
        #plan A
        
        return 'success'
    
    def pose_callback(self, msg):
        # Update robot current pose
        self.current_pose = msg



class GraspItem(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.logerr("######### GraspItem #########")
        while not rospy.is_shutdown():
            msg = rospy.wait_for_message('/chassis_begin', tidyup_chassis)
            rospy.logwarn("Wait for grasp item") 
            if msg.state == Chassis_State_Place:
                return 'success'
            rospy.sleep(1)


class MoveToPlace(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

        # Subscribe to topic '/chassis_begin' to receive target room ID
        self.sub = rospy.Subscriber("/chassis_begin", tidyup_chassis, self.begin_callback)

        # Define publisher to notify when robot reaches target position
        self.pub = rospy.Publisher("/chassis_finish", tidyup_chassis, queue_size = 10)

        # Default target room is None
        self.target_room = None

        # Flag to indicate if target position has been sent to avoid repeated sending
        self.is_send_goal = False

    def execute(self, userdata):
        rospy.logerr("######### MoveToPlace #########")

        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        while 1:
            laser_msg = rospy.wait_for_message('/scan', LaserScan)
            cmd_vel = Twist()
            cmd_vel.linear.x = - 0.1
            cmd_vel.angular.z = 0
            cmd_vel_pub.publish(cmd_vel)
            if min(laser_msg.ranges[200:313]) > 0.3:
                rospy.logwarn('OK!')
                break

        while not rospy.is_shutdown():
            if self.target_room == None:
                rospy.logwarn("Wait for target room ID") 
                rospy.sleep(1)
                continue

            # Send target position if not sent yet
            if not self.is_send_goal:
                # Define target position
                goal = MoveBaseGoal()   
                goal.target_pose.header.frame_id = 'map'  
                goal.target_pose.header.stamp = rospy.Time.now()  
                goal.target_pose.pose.position.x = target_poses[self.target_room][0][0]
                goal.target_pose.pose.position.y = target_poses[self.target_room][0][1]
                goal.target_pose.pose.orientation.x = target_poses[self.target_room][1][0]
                goal.target_pose.pose.orientation.y = target_poses[self.target_room][1][1]
                goal.target_pose.pose.orientation.z = target_poses[self.target_room][1][2]
                goal.target_pose.pose.orientation.w = target_poses[self.target_room][1][3]

                # Print target room name
                rospy.loginfo("Going to " + str(self.target_room))  

                # Send goal
                move_base.send_goal(goal)  
                self.is_send_goal = True

            # Check if robot has reached target position
            state = move_base.get_state()
            if state == GoalStatus.SUCCEEDED:  
                # If robot has reached target position, send notification
               
                while not rospy.is_shutdown():
                    laser_msg = rospy.wait_for_message('/scan', LaserScan)
                    if self.target_room == 'Trash':
                        if min(laser_msg.ranges[200:313]) < 0.4:
                            rospy.logwarn('Too close!')
                            cmd_vel_pub.publish(cmd_vel_stop)
                            break
                    else:
                        if min(laser_msg.ranges[200:313]) < 0.2:
                            rospy.logwarn('Too close!')
                            cmd_vel_pub.publish(cmd_vel_stop)
                            break

                    # Calculate the robot control command
                    cmd_vel = Twist()
                    cmd_vel.linear.x = 0.15
                    cmd_vel.angular.z = 0
                    cmd_vel_pub.publish(cmd_vel)

                    # Wait for a while to let the robot rotate to the new direction
                    rospy.sleep(0.1) 

                chassis_finish = tidyup_chassis()
                chassis_finish.state = Arrive_Place
                for i in range(0, 20):
                    self.pub.publish(chassis_finish)
                    rospy.sleep(0.1)
                self.target_room = None
                self.is_send_goal = False
                return 'success'
                
            rospy.sleep(1)
    
    def begin_callback(self, msg):
        if msg.state == Chassis_State_Place:
            if msg.room_id == 1:
                self.target_room = 'Kitchen_1'
            elif msg.room_id == 2:
                self.target_room = 'Kitchen_2'
            elif msg.room_id == 3:
                self.target_room = 'Bedroom_1'
            elif msg.room_id == 4:
                self.target_room = 'Bedroom_2'
            elif msg.room_id == 5:
                self.target_room = 'Livingroom_1'
            elif msg.room_id == 6:
                self.target_room = 'Livingroom_2'
            elif msg.room_id == 7:
                self.target_room = 'Diningroom_1'
            elif msg.room_id == 8:
                self.target_room = 'Diningroom_2'
            elif msg.room_id == 9:
                self.target_room = 'Trash'
            elif msg.room_id == 10:
                self.target_room = 'Biscuit_Place'
            elif msg.room_id == 11:
                self.target_room = 'Chip_Place'
            elif msg.room_id == 12:
                self.target_room = 'Lays_Place'
            elif msg.room_id == 13:
                self.target_room = 'Bread_Place'
            elif msg.room_id == 14:
                self.target_room = 'Cookie_Place'
            elif msg.room_id == 15:
                self.target_room = 'Handwash_Place'
            elif msg.room_id == 16:
                self.target_room = 'Dishsoap_Place'
            elif msg.room_id == 17:
                self.target_room = 'Water_Place'
            elif msg.room_id == 18:
                self.target_room = 'Sprite_Place'
            elif msg.room_id == 19:
                self.target_room = 'Cola_Place'
            elif msg.room_id == 20:
                self.target_room = 'Orange_juice_Place'
            elif msg.room_id == 21:
                self.target_room = 'Shampoo_Place'
        else:
            self.target_room = None


class GoBack(State):
    def __init__(self):
        State.__init__(self, outcomes=['success', 'finish'])

        # Default target room is None
        self.target_room = None

        # Define publisher to notify when robot reaches target position
        self.pub = rospy.Publisher("/chassis_finish", tidyup_chassis, queue_size = 10)

        # Flag to indicate if target position has been sent to avoid repeated sending
        self.is_send_goal = False

        # self.is_palce == False

    def execute(self, userdata):
        rospy.logerr("######### GoBack #########")

        while not rospy.is_shutdown():
            msg = rospy.wait_for_message('/chassis_begin', tidyup_chassis)
            if msg.state == Chassis_State_Finish:
                return 'finish'
            elif msg.state != Chassis_State_Room:
                rospy.logwarn("Wait for place item") 
                rospy.sleep(1)
                continue

            if work_room:
                self.target_room = work_room
            else:
                rospy.logerr("Target room is none") 

            # Send target position if not sent yet
            if not self.is_send_goal:
                # Define target position
                goal = MoveBaseGoal()   
                goal.target_pose.header.frame_id = 'map'  
                goal.target_pose.header.stamp = rospy.Time.now()  
                goal.target_pose.pose.position.x = target_poses[self.target_room][0][0]
                goal.target_pose.pose.position.y = target_poses[self.target_room][0][1]
                goal.target_pose.pose.orientation.x = start_twist_pose.orientation.x
                goal.target_pose.pose.orientation.y = start_twist_pose.orientation.y
                goal.target_pose.pose.orientation.z = start_twist_pose.orientation.z
                goal.target_pose.pose.orientation.w = start_twist_pose.orientation.w
                
                # Print target room name
                rospy.loginfo("Go back to " + str(self.target_room))  

                # Send goal
                for i in range(0, 2):
                    move_base.send_goal(goal)
                    rospy.sleep(1)  
                self.is_send_goal = True

            # Check if robot has reached target position
            state = move_base.get_state()  
            if state == GoalStatus.SUCCEEDED:  
                # If robot has reached target position, send notification
                chassis_finish = tidyup_chassis()
                chassis_finish.state = Arrive_Room
                for i in range(0, 20):
                    self.pub.publish(chassis_finish)
                    rospy.sleep(0.1)
                self.is_send_goal = False
                return 'success'
            
            rospy.sleep(1)


class Stop(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

        # Define publisher to notify when robot reaches target position
        self.pub = rospy.Publisher("/chassis_finish", tidyup_chassis, queue_size = 10)

    def execute(self, userdata):
        rospy.logerr("######### Finish #########")
        rospy.sleep(1)
        return 'success'


if __name__ == '__main__':
    try:
        signal.signal(signal.SIGINT, signal_handler)
        # Create a state machine and add states
        sm = StateMachine(outcomes=['success'])
        with sm:
            # StateMachine.add('BEGIN', Begin(), transitions={'success': 'MOVE_TO_ROOM'})
            StateMachine.add('MOVE_TO_ROOM', MoveToRoom(), transitions={'success': 'EXPLORE_ROOM'})
            StateMachine.add('EXPLORE_ROOM', ExploreRoom(), transitions={'success': 'FACE_TO_ITEM', 'timeout': 'MOVE_TO_ROOM'})
            StateMachine.add('FACE_TO_ITEM', FaceToItem(), transitions={'success': 'MOVE_TO_ITEM'})
            StateMachine.add('MOVE_TO_ITEM', MoveToItem(), transitions={'success': 'GRASP_ITEM', 'timeout': 'EXPLORE_ROOM'})
            StateMachine.add('GRASP_ITEM', GraspItem(), transitions={'success': 'MOVE_TO_PLACE'})
            StateMachine.add('MOVE_TO_PLACE', MoveToPlace(), transitions={'success': 'GO_BACK'})
            StateMachine.add('GO_BACK', GoBack(), transitions={'success': 'EXPLORE_ROOM', 'finish': 'STOP'})
            StateMachine.add('STOP', Stop(), transitions={'success': 'STOP'})

        # Start state machine
        sm.execute()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Tidy up finished!")
