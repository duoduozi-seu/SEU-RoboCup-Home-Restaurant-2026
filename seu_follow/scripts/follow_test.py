#! /home/syw/anaconda3/envs/GPSR/bin/python
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import pyrealsense2 as rs
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from pid import PID
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from seu_follow.srv import *
# 服务名称
seuhome_follow_name = 'follow_service'
# 模型加载路径
model_path = '/home/syw/code/robocup_nation/2024code/GPSR/src/seu_follow/model/gesture_recognizer.task'

# mediapipe官网:https://developers.google.com/mediapipe
# 加载mediapipe手势识别gesture_recognizer模型
BaseOptions = python.BaseOptions
options = vision.GestureRecognizerOptions(
    base_options=BaseOptions(model_asset_path = model_path),
    running_mode=vision.RunningMode.IMAGE)
recognizer = vision.GestureRecognizer.create_from_options(options)

# 初始化mediapipe骨架跟随
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

# PID控制器
def PID_controller(center, dis):
    # 初始化PID控制器（需要根据实际应用对参数进行调整）
    angular_pid = PID(kp=1/800, ki=0, kd=1/200, dz=15, th=0.9) # (kp=1/800,kd=1/200)   kd=1/500,1/400效果良好，kd=1/200原地振荡，kd=1/300运动时振荡
    linear_pid = PID(kp=1/2, ki=0, kd=1, dz=0.05, th=0.8)  # kd=1;kp=1/2高速，kp=1/4低速

    # 计算角速度和线速度的误差
    error_angle = center[0] - 320  # 320是图像中心的x坐标
    error_distance = dis - 1.1  # 1.1是期望的距离

    # 使用PID控制器计算角速度和线速度控制量
    angular_cmd_PD = -angular_pid.control_action(error_angle, dt=1)
    linear_cmd_PD = linear_pid.control_action(error_distance, dt=1)
    
    return angular_cmd_PD, linear_cmd_PD

# 跟随服务执行函数
def Follow_Service(req):
    global frame
    command = None
    
    # ROS发布器
    image_pub = rospy.Publisher("gpsr_img", Image, queue_size=10)
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    response = seu_followResponse()
    response.state = False

    # 打开摄像头
    #cap = cv2.VideoCapture(0)  # 笔记本摄像头（测试用）
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(rs.context().query_devices()[1].get_info(rs.camera_info.serial_number))
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    align_to = rs.stream.color      # align_to 是计划对齐深度帧的流类型
    align = rs.align(align_to) 
    
    while True:
        # 读取视频流
        # ret, frame = cap.read()  # 笔记本摄像头（测试用）
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)      # 获取对齐帧，将深度框与颜色框对齐
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()       
        if not depth_frame or not color_frame:
            continue
        frame = np.asanyarray(color_frame.get_data())
        # 将颜色空间从BGR转换为RGB
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # 进行手势识别
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
        gesture_results = recognizer.recognize(mp_image)
        

        # 获取手势结果
        if len(gesture_results.gestures):
            # 获取手势标签，有👍👎✋🤟👆等手势，具体请参考mediapipe官网
            gesture_label = gesture_results.gestures[0][0].category_name
            # 判断手势，实现状态转移
            if gesture_label=='Thumb_Up':   # 朝镜头竖起大拇指👍
                command = 'Follow'
            if gesture_label=='Open_Palm':   # 朝镜头张开手✋
                command = 'Stop'
            if gesture_label=='ILoveYou':   # 朝镜头弯下中指和无名指🤟
                command = 'Exit'
            # 在图像上显示手势标签
            cv2.putText(frame, f"Gesture: {gesture_label}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # 暂停状态，可通过手势恢复至跟随状态
        if command == 'Stop':
            vel = Twist()
            # vel_pub.publish(vel)
            cv2.putText(frame, command, (540, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # 跟随状态
        if command == 'Follow':
            vel = Twist()
            center = None
            
            # 处理图像并查找姿势
            results = pose.process(image)

            # 绘制骨架（可选）
            # mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

            # 获取肩部关键点
            if results.pose_landmarks:
                left_shoulder = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER]
                right_shoulder = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER]
                
                # 计算两肩的中点坐标
                mid_point_x = (left_shoulder.x + right_shoulder.x) / 2
                mid_point_y = (left_shoulder.y + right_shoulder.y) / 2
                
                # 将归一化坐标转换为像素坐标
                image_width = 640
                image_height = 480
                pixel_x = int(mid_point_x * image_width)
                pixel_y = int(mid_point_y * image_height)
                center = (pixel_x, pixel_y)
                # rospy.loginfo(center)
                
                # 在图像上绘制肩部关键点和中点
                image_height, image_width, _ = frame.shape
                cv2.circle(frame, (int(left_shoulder.x * image_width), int(left_shoulder.y * image_height)), 5, (0, 255, 0), -1)
                cv2.circle(frame, (int(right_shoulder.x * image_width), int(right_shoulder.y * image_height)), 5, (0, 255, 0), -1)
                cv2.circle(frame, (int(mid_point_x * image_width), int(mid_point_y * image_height)), 5, (255, 0, 0), -1)

                # 获取中心点深度值（中心点可能在图像外部，需过滤这种情况）
                if center[0] >= 0 and center[0] < 640 and center[1] >= 0 and center[1] < 480:
                    dis = depth_frame.get_distance(center[0], center[1])
                    if dis == 0:  # 读取的深度值可能会为0，需过滤这种情况
                        dis = 1.1

                    vel.angular.z, vel.linear.x = PID_controller(center, dis)
                else:
                    rospy.logwarn('center out of range')

            else:
                rospy.loginfo('Can not find person')
                vel.angular.z, vel.linear.x = (0, 0)

            # 绘制状态、角速度、线速度
            cv2.putText(frame, command, (540, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(frame, 'ang:'+str(round(float(vel.angular.z), 2)), (0, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(frame, 'lin:'+str(round(float(vel.linear.x), 2)), (0, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            # vel_pub.publish(vel)
        
        # 终止跟随，服务完成
        if command == 'Exit':
            rospy.loginfo('Follow Service Finish!')
            # cap.release()  # 笔记本摄像头（测试用）
            pipeline.stop()  
            
            response.state = True
            return response
        
        # 由于cv2.imshow在第二次调用服务时会出现阻塞问题，故改为将图片以ROS消息发送，在rviz中订阅显示
        bridge = CvBridge()  # 创建CvBridge对象
        ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")  # cv转img
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', frame)
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
        image_pub.publish(ros_image)  # 发布图片消息

if __name__ == '__main__':
    Follow_Service('test')
