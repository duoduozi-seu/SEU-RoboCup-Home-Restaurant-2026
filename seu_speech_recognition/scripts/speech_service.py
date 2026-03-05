#!/home/duo/anaconda3/envs/gpsr_whisper310/bin/python
# -*- coding: utf-8 -*-
'''
语音识别启动节点
启动命令：

    LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6:/usr/lib/x86_64-linux-gnu/libtiff.so.5 rosrun seu_speech_recognition speech_service.py

'''
import rospy 
from seu_speech_recognition.srv import *
from my_whisper import recognizer

def main():
    # 注册语音识别服务：识别逻辑在 my_whisper.recognizer.recognition 中实现。
    rospy.init_node('speech_recognition_node')
    s = rospy.Service('speech_recognition_service', seu_speech_recognition, recognizer.recognition)
    rospy.loginfo("Ready to recognize speech (Whisper)")
    rospy.spin()

if __name__ == '__main__':
    main()
