# 面向look for object功能包

## 目录结构描述
    │  CMakeLists.txt
    │  package.xml
    │  README.md
    │  
    ├─include
    │  └─look_for_object
    ├─scripts
    │  │  find_object.py
    │  │  
    │  ├─yolov7
    │  │  │  find_object_camera.py
    │  │  │  yolo.py
    │  │  │  
    │  │  ├─model
    │  │  │      gesture_recognizer.task
    │  │  │      selfie_segmenter.tflite
    │  │  │      
    │  │  ├─model_data
    │  │  │      gpsr_find_object.pth
    │  │  │      gpsr_find_object_classes.txt
    │  │  │      simhei.ttf
    │  │  │      voc_classes.txt
    │  │  │      yolo_anchors.txt
    │  │  │      
    │  │  ├─nets
    │  │  │  │  backbone.py
    │  │  │  │  yolo.py
    │  │  │  │  yolo_training.py
    │  │  │  │  __init__.py
    │  │  │  │  
    │  │  │  └─__pycache__
    │  │  │          backbone.cpython-38.pyc
    │  │  │          yolo.cpython-38.pyc
    │  │  │          __init__.cpython-38.pyc
    │  │  │          
    │  │  ├─utils
    │  │  │  │  callbacks.py
    │  │  │  │  dataloader.py
    │  │  │  │  utils.py
    │  │  │  │  utils_bbox.py
    │  │  │  │  utils_fit.py
    │  │  │  │  utils_map.py
    │  │  │  │  __init__.py
    │  │  │  │  
    │  │  │  └─__pycache__
    │  │  │          utils.cpython-38.pyc
    │  │  │          utils_bbox.cpython-38.pyc
    │  │  │          __init__.cpython-38.pyc
    │  │  │          
    │  │  └─__pycache__
    │  │          yolo.cpython-38.pyc
    │  │          
    │  └─__pycache__
    │          find_object.cpython-38.pyc
    │          
    ├─src
    └─srv
            seu_findobject.srv

## 文件用途
- **find_object.py**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;支持找物，为物品识别模块主程序
- **find_object_camera.py**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;支持找物，为物品识别模块摄像头控制程序，主程序采用消息控制本程序
- **seu_findobject.srv**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;上层任务调度找物任务采用的服务


## 消息
- **seu_robot_msgs/object_position.msg**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;本消息用于实现将摄像头识别内容发送给物品识别主程序
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;注：不在本功能包下
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**string label**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;用于发送相机识别到的物品名称
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
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;本消息用于实现物体识别主程序对于摄像头的控制

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
- **seu_look_for_object/seu_findobject.srv**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;本服务用于上层任务调度机调用找物任务
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**string object**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;上层任务调度机发送物品名，启动找物
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;---
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**bool state**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;找物任务完成返回任务状态
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**string errormsg**
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;返回找物任务错误信息



## 找物主程序节点
### 发布
    /cmd_vel                    消息    Twist.msg               发布底盘速度
    /findobject_camera_state    消息    String.msg              控制相机

### 接收/服务器
    seuhome_find_object_name    服务    seu_findobject.srv      启动找物
    /find_object_result         消息    object_position.msg     识别结果
    /robot_pose                 消息    PoseStamped.msg         机器人位置


## 摄像头控制程序节点
### 发布
    /find_object_result         消息    object_position.msg     识别结果
    /gpsr_img                   消息    Image.msg               图像数据

### 接收
    /findobject_camera_state    消息    String.msg              控制相机


## 启动
### 单模块调试
    启动底盘（见底盘相关）:导航、机器人模型
    rosrun seu_look_for_object find_object_camera_2025.py
    rosrun seu_look_for_object find_object.py
    rosservice call /find_object_service "object: 'fan'" #可能有错建议查一下

    rostopic pub /findobject_camera_state std_msgs/String "camera_open" #启动相机
    
### gpsr整体启动
    位于seu_smach_main 中init.launch和export.launch
    <node pkg="seu_look_for_object" type="find_object.py" name="find_object_service" output="screen" />
    <node pkg="seu_look_for_object" type="find_object_camera.py" name="find_object_camera" output="screen" />

## 其他
### 环境依赖
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;opencv-python
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;pyrealsense
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;numpy
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;PIL
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;yolov7
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;smatch
<br/> 
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;ros

注：仅列出所用模块，具体模块名可能有稍许出入
### 绝对路径
find_object_camera.py需要加载模型文件
<br/> 
文件中yolo环境初始化函数需要载入模型文件和类别，初始化函数位于yolo.py中
<br/> 
模型文件位于yolov7/model_data
