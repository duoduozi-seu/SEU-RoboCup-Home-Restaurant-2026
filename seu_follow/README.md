# Follow Service Package

## Overview

这是一个ROS功能包，用于实现人体跟踪服务的封装，自定义服务消息seu_follow.srv，编写服务脚本follow_service.py，以及pid控制脚本pid.py。

## Basic Usage

在启动底盘launch文件后，要启动该服务，运行如下命令：

    rosrun seu_follow follow_service.py

要在终端中测试该服务的调用，请运行：

    rosservice call /follow_service person None

可在rviz中订阅图像消息/gpsr_img

## Node: follow_service_node

这是一个人体跟踪节点，它利用手势识别和人体骨架跟踪实现人体跟踪功能，具体来说，通过手势识别控制程序的状态:跟踪（朝镜头竖起大拇指👍）、暂停（朝镜头张开手✋）和终止（朝镜头弯下中指和无名指🤟），通过骨架跟踪计算出人体两肩中点的像素坐标并结合其深度信息，计算出PD控制量发送给底盘控制节点，终止跟踪后返回跟踪结果及错误码（未使用）。

路径修改：在follow_service.py中修改加载模型的全局路径

环境：使用虚拟环境GPSR，可在代码第一行修改编译器路径。
### Subscribed Topics

* **`/cmd_vel`** ([geometry_msgs/Twist])

    向cmd_vel话题发送Twist消息以控制机器人运行。

### Services

* **`follow_service`** ([seu_follow_srv/Empty])

    调用该服务时参数可缺省。
