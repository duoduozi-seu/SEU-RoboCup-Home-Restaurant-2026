#!/usr/bin/env python
# 语音识别有时候会出问题，本代码用于测试语音识别服务
import rospy
from seu_speech_recognition.srv import *
import sys
import pyttsx3

# 服务名称
seuhome_speech_recognition_name = 'speech_recognition_service'

# 语音识别
def srv_speech_recognition():
    srv_name = seuhome_speech_recognition_name  # 当前服务的名称
    client = rospy.ServiceProxy(srv_name, seu_speech_recognition)  # 当前操作的服务
    rospy.wait_for_service(srv_name,timeout = 1.0)  # 等待服务
    rospy.loginfo(srv_name + ' available')
    try:
        request = seu_speech_recognitionRequest()
        request.enable = False  
        response = client.call(request)  # 返回值
        if response.state == True:
            rospy.loginfo('Service call Succeeded: ' + srv_name)  # 调用成功
            return response.word
        else:
            rospy.loginfo("Service call Failed: " + srv_name)  # 调用失败
            return -1  # 调用失败，返回-1

    except rospy.ServiceException as e:  # 调用失败，输出具体信息
        print("Service call Failed: " + srv_name + ": %s" % e)
        return -1


# 初始化 TTS 引擎 (全局单例，避免重复初始化导致的 ReferenceError)
try:
    engine = pyttsx3.init()
    voices = engine.getProperty('voices')
    engine.setProperty('voice', voices[10].id)
    engine.setProperty('rate', 160)
except Exception as e:
    print(f"TTS初始化失败: {e}")
    engine = None

# 语音合成
def speech_to_text(data):
    if engine is None:
        print(f"[TTS Skipped]: {data}")
        return

    # 播报
    engine.say(data)
    engine.runAndWait()
    
if __name__ == "__main__":
    rospy.init_node("speech_test_node")
    # 语音播报提示词
    wakeup = "please say the command"
    speech_to_text(wakeup)
    # 语音识别
    word = srv_speech_recognition()
    print(word)
    # 语音播报识别结果
    if word != -1:
        speech_to_text(word)
    