import sys
sys.path.append("/home/syw/code/robocup_nation/2024code/GPSR/src/common")
import collections
import json
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
import pyttsx3
import rospy
from seu_smach_main.srv import *

def look_for_person(target):  # 调用找人服务
    rospy.loginfo("Looking for person {}".format(target))
    srv_name = 'find_person_service' # 当前服务的名称
    client = rospy.ServiceProxy(srv_name, seu_findperson)  # 当前操作的服务
    rospy.wait_for_service(srv_name)  # 等待服务
    rospy.loginfo(srv_name + ' available')
    try:
        request = seu_findpersonRequest()
        request.feature = target
        response = client.call(request)  # 返回值
        if response.state == True:
            rospy.loginfo('Service call Succeeded: ' + srv_name)  # 调用成功
            return True
        else:
            rospy.logerr("Service call Failed: " + srv_name)  # 调用失败
            rospy.logerr("errormsg: " + response.errormsg)
            return False

    except rospy.ServiceException as e:  # 调用失败，输出具体信息
        print("Service call Failed: " + srv_name + ": %s" % e)
        return False

def speech_to_text(data):
    # 初始化
    speech=pyttsx3.init()
    voices=speech.getProperty('voices')
    # 音色
    speech.setProperty('voice',voices[2].id)
    # 频率
    speech.setProperty('rate',120)
    # 播报
    speech.say(data)
    speech.runAndWait()
def read_locations_from_file(filename):
    """从JSON文件读取并重建locations字典"""
    with open(filename, 'r') as f:
        data = json.load(f, object_pairs_hook=collections.OrderedDict)

    locations = collections.OrderedDict()
    locations["corners"] = []
    for name, pose_dict in data.items():
        if name != "corners":
            pos = Point(**pose_dict['position'])
            orient = Quaternion(**pose_dict['orientation'])
            locations[name] = Pose(pos, orient)
        else:
            for p in pose_dict:
                pos = Point(**p['position'])
                orient = Quaternion(**p['orientation'])
                locations["corners"].append(Pose(pos, orient))
    return locations

locations = read_locations_from_file('/home/syw/code/robocup_nation/2024code/GPSR/src/const/locations.json')

def write_locations_to_file(filename, locations):
    """将locations字典写入JSON文件"""
    # 转换Pose对象为可序列化的字典
    serializable_locations = collections.OrderedDict()
    for name, pose in locations.items():
        serializable_locations[name] = {
            'position': {'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z},
            'orientation': {'x': pose.orientation.x, 'y': pose.orientation.y,
                            'z': pose.orientation.z, 'w': pose.orientation.w}
        }

    # 写入文件
    with open(filename, 'w') as f:
        json.dump(serializable_locations, f, indent=4)