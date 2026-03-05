#!/usr/bin/env python
# 语音识别有时候会出问题，本代码用于测试语音识别服务
import rospy
from seu_speech_recognition.srv import *
import sys
from seu_voice.srv import seu_voice as seu_voice_srv, seu_voiceRequest

# 服务名称
seuhome_speech_recognition_name = 'speech_recognition_service'
seuhome_voice_name = "voice_service"

# busy 重试策略：每 1s 查询一次，最多查询 5 次。
VOICE_BUSY_ERROR_CODE = 1002
VOICE_BUSY_QUERY_INTERVAL_SEC = 1.0
VOICE_BUSY_MAX_QUERY_COUNT = 5
PKG_NAME_SPEECH = "seu_speech_recognition"
_voice_client = None

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


def _print_voice_busy_skip(pkg_name):
    """按需求输出红色 busy 提示。"""
    msg = f"[{pkg_name}]：voice service busy，skip speech。"
    sys.stderr.write(f"\033[31m{msg}\033[0m\n")
    sys.stderr.flush()


def _get_voice_client():
    """获取 voice_service 客户端单例。"""
    global _voice_client
    if _voice_client is None:
        _voice_client = rospy.ServiceProxy(seuhome_voice_name, seu_voice_srv)
    return _voice_client


def _call_voice_service_blocking(text):
    """调用 voice_service 进行阻塞播报，并处理 busy 轮询逻辑。"""
    if text is None:
        return False
    text = str(text).strip()
    if not text:
        return False

    try:
        rospy.wait_for_service(seuhome_voice_name, timeout=1.0)
    except rospy.ROSException as e:
        rospy.logerr("voice service unavailable: %s", e)
        return False

    client = _get_voice_client()
    req = seu_voiceRequest(text=text)

    def _call_once():
        try:
            return client.call(req)
        except rospy.ServiceException as call_error:
            rospy.logerr("voice service call failed: %s", call_error)
            return None

    response = _call_once()
    if response is None:
        return False
    if response.state:
        return True
    if response.errorcode != VOICE_BUSY_ERROR_CODE:
        rospy.logerr("voice service failed: errorcode=%s, errormsg=%s",
                     response.errorcode, response.errormsg)
        return False

    for _ in range(VOICE_BUSY_MAX_QUERY_COUNT):
        rospy.sleep(VOICE_BUSY_QUERY_INTERVAL_SEC)
        response = _call_once()
        if response is None:
            return False
        if response.state:
            return True
        if response.errorcode != VOICE_BUSY_ERROR_CODE:
            rospy.logerr("voice service failed after busy retry: errorcode=%s, errormsg=%s",
                         response.errorcode, response.errormsg)
            return False

    _print_voice_busy_skip(PKG_NAME_SPEECH)
    return False

# 语音合成
def speech_to_text(data):
    # 通过 voice_service 阻塞播报；失败时按策略跳过。
    _call_voice_service_blocking(data)
    
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
    
