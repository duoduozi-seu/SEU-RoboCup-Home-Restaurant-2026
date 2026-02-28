# 面向look for person功能包

## 目录结构描述
    │  CMakeLists.txt
    │  package.xml
    │  README.md
    │  
    ├─include
    │  └─look_for_person
    ├─scripts
    │  │  face.py
    │  │  find_person.py
    │  │  find_person_camera.py
    │  │  find_person_feature_strong.py
    │  │  find_person_feature_strong_camera.py
    │  │  obama-1080p.jpg
    │  │  
    │  ├─model
    │  │  └─weights
    │  │          gender_model_weights.h5
    │  │          
    │  └─__pycache__
    │          find_person.cpython-38.pyc
    │          find_person.cpython-39.pyc
    │          
    ├─src
    └─srv
            seu_findperson.srv



## 文件用途
- **face.py**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;用于测试face_recognition包
- **find_person.py**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;只支持找人（无特征），为人物识别模块主程序
- **find_person_camera.py**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;只支持找人（无特征），为人物识别模块摄像头控制程序，主程序采用消息控制本程序
- **find_person_feature_strong.py**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;最终采用，支持找人（本体特征或动作特征），为人物识别模块主程序
- **find_person_feature_strong_camera.py**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;最终采用，只支持找人（本体特征或动作特征），为人物识别模块摄像头控制程序，主程序采用消息控制本程序
- **seu_findperson.srv**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;上层任务调度找人任务采用的服务
- <span style="color: red;">**gender_model_weights.h5**</span>
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;人类本体特征模型



## 消息
- **seu_robot_msgs/object_position.msg**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;本消息用于实现将摄像头识别内容发送给人物识别主程序
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;注：不在本功能包下
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**string label**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;用于发送相机识别到的人的状态
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**float32 x**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**float32 y**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**float32 z**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;用于发送基于realsense特定坐标系下的物体位置信息

- **std_msgs/String.msg**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;本消息用于实现人物识别主程序对于摄像头的控制

- **geometry_msgs/PoseStamped.msg**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;本消息用于获取底盘当前位置

- **geometry_msgs/Twist.msg**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;本消息用于实现人物识别主程序对于底盘的速度控制

- **sensor_msgs/Image.msg**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;本消息用于传输摄像头数据


## 服务
- **seu_look_for_person/seu_findperson.srv**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;本服务用于上层任务调度机调用找人任务
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**string feature**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;上层任务调度机发送人的特征，启动找人任务
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;---
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**bool state**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;找人任务完成返回任务状态
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**string errormsg**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;返回找人任务错误信息


## 找人主程序节点
（以find_person_feature_strong.py为例）
### 发布
    /cmd_vel                    消息    Twist.msg               发布底盘速度
    /findperson_camera_state    消息    String.msg              控制相机

### 接收/服务器
    seuhome_find_person_name    服务    seu_findperson.srv      启动找人
    /find_person_result         消息    object_position.msg     识别结果
    /robot_pose                 消息    PoseStamped.msg         机器人位置


## 摄像头控制程序节点
（以find_person_feature_strong_camera.py为例）
### 发布
    /find_person_result         消息    object_position.msg     识别结果
### 接收
    /findperson_camera_state    消息    String.msg              控制相机


## 启动
### 单模块调试
    启动底盘（见底盘相关）:导航、机器人模型
    rosrun seu_look_for_person find_person_feature_strong_camera.py
    rosrun seu_look_for_person find_person_feature_strong.py
    rosservice call /find_person_service "feature: 'person'" #可能有错建议查一下
### gpsr整体启动
    位于seu_smach_main 中init.launch和export.launch
    <node pkg="seu_look_for_person" type="find_person_feature_strong.py" name="find_person_service" output="screen" />
    <node pkg="seu_look_for_person" type="find_person_feature_strong_camera.py" name="find_person_camera" output="screen" />

## 其他
### <span style="color: red;">模型载入</span>
deepface识别人物性别在主目录下创建.deepface/weights文件夹，鉴于模型下载较慢，需要手动将gender_model_weights.h5文件拷贝至weights文件夹下

```c
如：/home/zzn/.deepface/weights/gender_model_weights.h5
```
### 思路
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;1.根据上位机判断特征内容，动作特征&rarr;3，本体特征&rarr;2
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;2.找人并面向人，性别&rarr;4，无特征结束
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;3.使用mediapipe完成基于动作特征的找人，结束
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;4.使用deepface完成性别检测，结束

### 环境依赖
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;opencv-python
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;pyrealsense
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;numpy
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;smatch
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;ros
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;face_recognition
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;mediapipe
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;deepface
<br/> 
<span style="color: red;">**face_recognition,mediapipe,deepface需根据requirements.txt安装相关依赖**</span>
<br/> 
注：仅列出所用模块，具体模块名可能有稍许出入

### 绝对路径
face.py测试需要修改图像所在路径













        


   



