#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
语音识别启动节点
启动命令：

    LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6:/usr/lib/x86_64-linux-gnu/libtiff.so.5 rosrun seu_speech_recognition speech_service.py

'''
import os
import rospy 
from seu_speech_recognition.srv import *
from std_msgs.msg import String
import pyttsx3
from my_whisper import recognizer
from datetime import datetime

def get_info(key):
    try:
        result = rospy.get_param(key)
    except:
        result = None
    return result

def speech_to_text(data):
    """语音合成播报"""
    # 初始化
    speech=pyttsx3.init()
    voices=speech.getProperty('voices')
    # 音色
    speech.setProperty('voice',voices[10].id)
    # 频率
    speech.setProperty('rate',160)
    # 播报
    speech.say(data)
    speech.runAndWait()
    log_file = get_info('log')
    # 获取当前时间戳
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
    # 确保日志目录存在
    log_dir = os.path.dirname(log_file)
    if log_dir and not os.path.exists(log_dir):
        os.makedirs(log_dir)
    # 追加写入日志文件
    with open(f"{log_file}.txt", "a", encoding="utf-8") as f:
        f.write(f"[{timestamp}] {data}\n")

def main():
    rospy.init_node('speech_recognition_node')
    s = rospy.Service('speech_recognition_service', seu_speech_recognition, recognizer.recognition)
    rospy.loginfo("Ready to recognize speech (Whisper)")
    rospy.spin()

if __name__ == '__main__':
    main()

