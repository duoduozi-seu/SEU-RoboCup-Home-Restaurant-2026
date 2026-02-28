#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import DobotDllType as dType
from robot_msgs.msg import object_position
import pyttsx3
import time

i = 0
item = ''


position = {
    'Nescafe':[238.76515197753906, -0.5515472292900085, -4.673133850097656],
    'water glass':[170.10922241210938, 74.45411682128906, -46.987640380859375],
    'toilet water':[198.84803771972656, -100.27887725830078, -55.610267639160156]
}

# 辅助抓取时需要语音播报
def speech_to_text(data):
    speech=pyttsx3.init()
    voices=speech.getProperty('voices')
    speech.setProperty('voice',voices[10].id)
    speech.setProperty('rate',160)
    speech.say(data)
    speech.runAndWait()


def arm_callback(msg):

    global item
    item = msg.label
    dt = 0.75
    if item == 'water glass':
        dt = 0.7


    time.sleep(2)
    # 清空队列
    dType.SetQueuedCmdClear(api)

    # 设置运动参数
    dType.SetHOMEParams(api, 200, 200, 200, 200, isQueued=1)
    dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued=1)
    dType.SetPTPCommonParams(api, 100, 100, isQueued=1)

    # 开始执行指令队列
    dType.SetQueuedCmdStartExec(api)

    # 设置ptpcmd内容并将命令发送给dobot
    result = dType.SetCmdTimeout(api, 100)
    dType.SetEndEffectorParams(api, 0, 0, 0, isQueued=0)
    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, 104, -93, 42, 0, isQueued=1)[0]

    lastIndex = dType.SetWAITCmd(api, 1, isQueued=1)[0]

    # 张开手爪
    lastIndex = dType.SetIOMultiplexing(api, 4, 2, isQueued=1)
    lastIndex = dType.SetIOPWM(api, 4, 5, 0.25, isQueued=1)[0]
    lastIndex = dType.SetWAITCmd(api, 2, isQueued=1)[0]

    # 伸向物体
    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, position[item][0], position[item][1], position[item][2], 0, isQueued=1)[0]

    # 闭合手爪
    lastIndex = dType.SetIOMultiplexing(api, 4, 2, isQueued=1)
    lastIndex = dType.SetIOPWM(api, 4, 5, dt, isQueued=1)[0]
    speech_to_text('I have catch the item you were pointing to')
    time.sleep(6)
    lastIndex = dType.SetWAITCmd(api, 1, isQueued=1)[0]

    # 稍微上升手爪
    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, position[item][0], position[item][1], position[item][2] + 50, 0, isQueued=1)
    lastIndex = dType.SetWAITCmd(api, 1.5, isQueued=1)[0]

    # 回位
    dType.SetEndEffectorParams(api, 0, 0, 0, isQueued=0)
    # lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPJUMPXYZMode, 86.56906127929688, -14.573505401611328, -1.376800537109375, 0, isQueued=1)[0]
    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, 138.59526062011721, -10.6500120162963874, 4.926086425781252, 0, isQueued=1)[0]
    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, 83.48648071289062, -126.71294403076172, 34.51599884033203, 0, isQueued=1)[0]
    lastIndex = dType.SetWAITCmd(api, 2, isQueued=1)[0]
    speech_to_text('Please take the item')

    # 张开手爪
    lastIndex = dType.SetIOMultiplexing(api, 4, 2, isQueued=1)
    lastIndex = dType.SetIOPWM(api, 4, 5, 0.25, isQueued=1)[0]
    lastIndex = dType.SetWAITCmd(api, 1.5, isQueued=1)[0]

    # 闭合手爪
    lastIndex = dType.SetIOMultiplexing(api, 4, 2, isQueued=1)
    lastIndex = dType.SetIOPWM(api, 4, 5, 0.75, isQueued=1)[0]
    # lastIndex = dType.SetWAITCmd(api, 1.5, isQueued=1)[0]
    pos = dType.GetPose(api)
    print("pos", pos)
    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, 38.78642272949219, -88.50606536865234, -6.8310089111328125, 0, isQueued=1)[0]


    # 如果还未完成指令队列则等待
    while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(10)

    print('timeout:', result)

    # 停止执行指令
    dType.SetQueuedCmdStopExec(api)
    dType.SetQueuedCmdClear(api)

    if result != None:
        return False
    else:
        return True



def catch():
    # ROS节点初始化
    rospy.init_node('object_pos', anonymous=True)
    # 创建一个Subscriber，订阅名为/object_pos的topic，注册回调函数arm_callback
    rospy.Subscriber('object_pos', object_position, arm_callback, queue_size=10)
    # 循环等待回调函数
    rospy.spin()


if __name__ == '__main__':
    CON_STR = {
        dType.DobotConnect.DobotConnect_NoError: "DobotConnect_NoError",
        dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
        dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

    # 将dll读取到内存中并获取对应的CDLL实例
    # Load Dll and get the CDLL object
    api = dType.load()
    # 建立与dobot的连接
    # Connect Dobot
    state = dType.ConnectDobot(api, "", 115200)[0]
    print("Connect status:", CON_STR[state])

    if (state == dType.DobotConnect.DobotConnect_NoError):
        catch()
        # get_item()

    # 断开连接
    # Disconnect Dobot
    dType.DisconnectDobot(api)
    print("1111111111111")

