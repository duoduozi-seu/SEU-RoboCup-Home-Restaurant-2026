# Nav Service Package

## Overview

这是一个ROS功能包，用于实现导航服务的封装，自定义服务消息seu_nav.srv，编写服务脚本nav_service.py。

## Basic Usage

在启动底盘launch文件后，要启动该服务，运行如下命令：

    rosrun seu_nav nav_service.py

要在终端中测试该服务的调用，请运行：

    rosservice call /nav_service "{des}"

## Node: nav_service_node

这是一个导航服务节点，它接收用户提供的目的地名称，调用导航功能，控制机器人前往目标位姿，返回导航结果及错误码（未使用）。
目的地名称与坐标的对应关系需在 nav_service.py 中提前定义

### Subscribed Topics

* **`/move_base`** ([geometry_msgs/Pose])

    向move_base话题的MoveBaseAction发布目标点位姿Pose实现导航。

### Services

* **`nav_service`** ([seu_nav_srv/string])

    该服务接收目的地名称，调用导航功能，返回导航结果及错误码。