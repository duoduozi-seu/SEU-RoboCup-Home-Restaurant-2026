#!/usr/bin/env python3
import threading
import time
import pyrealsense2 as rs
import numpy as np
import cv2
# 提示没有aruco的看问题汇总
import cv2.aruco as aruco
import DobotDllType as dType

# 手眼标定用，需要 python3.6

# 配置摄像头与开启pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)
align_to = rs.stream.color
align = rs.align(align_to)

center=[]

# 获取对齐的rgb和深度图
def get_aligned_images( ):
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    # 获取intelrealsense参数
    intr = color_frame.profile.as_video_stream_profile().intrinsics
    # 内参矩阵，转ndarray方便后续opencv直接使用
    intr_matrix = np.array([
        [intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]
    ])
    # 深度图-16位
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    # 深度图-8位
    depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)
    pos = np.where(depth_image_8bit == 0)
    depth_image_8bit[pos] = 255
    # rgb图
    color_image = np.asanyarray(color_frame.get_data())
    # return: rgb图，深度图，相机内参，相机畸变系数(intr.coeffs)
    return color_image, depth_image,intr, intr_matrix, np.array(intr.coeffs),aligned_depth_frame


def center_aruco():
    rgb, depth, intr,intr_matrix, intr_coeffs,aligned_depth_frame = get_aligned_images()
    # cv2.imshow("Image", rgb)
    # 获取dictionary, 5x5的码，指示位50个，这个看你打出来的二维码是选哪些参数打的来设置
    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    # aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    # 创建detector parameters
    parameters = aruco.DetectorParameters_create()
    # parameters = aruco.DetectorParameters()
    # 输入rgb图, aruco的dictionary, 相机内参, 相机的畸变参数
    corners, ids, rejected_img_points = aruco.detectMarkers(rgb, aruco_dict, parameters=parameters,
                                                            cameraMatrix=intr_matrix, distCoeff=intr_coeffs)
    # detector = aruco.ArucoDetector(aruco_dict, parameters)
    # corners, ids, rejected_img_points = detector.detectMarkers(rgb)
    # print("corner is ", corners)
    if len(corners) != 0:
        point = np.average(corners[0][0], axis=0)
        depth = aligned_depth_frame.get_distance(point[0], point[1])
        point = np.append(point, depth)
        print("depth=",depth)
        if depth != 0:
            global center
            global color_image
            #                     print("center:%f %f, depth:%f m" %(point[0], point[1], point[2]))
            x = point[0]
            y = point[1]
            z = point[2]
            ## see rs2 document:
            ## https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0#point-coordinates
            ## and example: https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0#point-coordinates
            x, y, z = rs.rs2_deproject_pixel_to_point(intr, [x, y], z)
            center = [x, y, z]
            # print("center is ", center)
            color_image = aruco.drawDetectedMarkers(rgb, corners)

    else:
        center, color_image = None, None
        print("not found aruco!")
    return center, color_image, rgb



def dobot_connect( ):
    # dobot init
    CON_STR = {
        dType.DobotConnect.DobotConnect_NoError: "DobotConnect_NoError",
        dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
        dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

    # Load Dll
    api = dType.load()


    # Connect Dobot
    state = dType.ConnectDobot(api, "", 115200)[0]

    print("Connect status:", CON_STR[state])
    return api, state

def dobot_init(api,state):
    if (state == dType.DobotConnect.DobotConnect_NoError):
        dType.SetQueuedCmdClear(api)
        dType.SetQueuedCmdStartExec(api)
        dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
        dType.SetPTPCommonParams(api, 100, 100, isQueued = 1)
        # 是否设偏移量
        dType.SetEndEffectorParams(api,0,0,0,isQueued=1)

if __name__ == "__main__":
    n = 0
    i = 0

    # 不偏移，设几个能看见的点
    default_cali_points = [[118, 53, 30, 0], 
                           [141, -35, 65, 0],
                           [144, 85, 20, 0],
                           [130, -30, 40, 0], 
                           [140, 70, 34, 0],
                           [116, 57, 52, 0], 
                           [120, -48, 30, 0],
                           [149, 32, 40, 0]]
    np_cali_points = np.array(default_cali_points)
    arm_cord = np.column_stack(
        (np_cali_points[:, 0:3], np.ones(np_cali_points.shape[0]).T)).T
    centers = np.ones(arm_cord.shape)

    api, state = dobot_connect()
    dobot_init(api, state)

    if (state == dType.DobotConnect.DobotConnect_NoError):
        i=0
        for ind, pt in enumerate(default_cali_points):
            print("Current points:", pt)
            queuedCmdIndex = dType.SetPTPCmd(api, 1, pt[0], pt[1], pt[2], pt[3], isQueued=1)
            while dType.GetQueuedCmdCurrentIndex(api) != queuedCmdIndex:
                time.sleep(2)
            time.sleep(3)
            center, color_image, rgb = center_aruco()
            print("save")
            # cv2.imwrite('src/dobot/'+str(i)+".jpg",rgb)
            cv2.imwrite(str(i) + ".jpg", rgb)
            i=i+1
            if center is not None:
                centers[0:3, ind] = center
                print("centers is ", center)
            else:
                print("no aruco!")

    image_to_arm = np.dot(arm_cord, np.linalg.pinv(centers))
    arm_to_image = np.linalg.pinv(image_to_arm)
    # dType.SetPTPCmd(api, 1, 217, 0, 154, 0, isQueued=0)
    dType.SetQueuedCmdStopExec(api)

    print("Finished")
    print("Image to arm transform:\n", image_to_arm)
    print("Arm to Image transform:\n", arm_to_image)
    print("Sanity Test:")

    print("-------------------")
    print("Image_to_Arm")
    print("-------------------")
    for ind, pt in enumerate(centers.T):
        print("Expected:", default_cali_points[ind][0:3])
        print("camera:",np.array(pt))
        print("Result:", np.dot(image_to_arm, np.array(pt))[0:3])


    print("-------------------")
    print("Arm_to_Image")
    print("-------------------")
    for ind, pt in enumerate(default_cali_points):
        print("Expected:", centers.T[ind][0:3])
        pt[3] = 1
        print("Result:", np.dot(arm_to_image, np.array(pt))[0:3])

    #cv2.destroyAllWindows()
    pipeline.stop()
    time.sleep(1)
    dType.DisconnectDobot(api)
