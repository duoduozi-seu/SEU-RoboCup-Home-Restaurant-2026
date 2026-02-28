# GPSR smach main Package

## Overview

这是GPSR题目的主体状态机功能包，包括启动各个基元服务的launch文件以及主程序gpsr_smach.py

## Basic Usage

在启动底盘launch文件后，首先启动各基元服务，使用脚本开启。进入/seu_smach_main/scripts路径后，运行如下命令：

    ./gpsr_bringup.sh

脚本会先后启动init.launch和export.launch，在启动后者之前需要对环境变量进行操作 export PYTHONPATH="${PYTHONPATH}:/usr/lib/python3/dist-packages/，这一操作在脚本中已经自动实现

再启动主程序，新开一个窗口，运行如下命令:

    rosrun seu_smach_main gpsr_smach.py

观测状态机运行情况可运行如下命令：（需要先退出anaconda环境）

    conda deactivate
    rosrun smach_viewer smach_viewer.py 

## Node: gpsr_smach_node

这是gpsr的主程序节点，它包括用户语音输入指令、大模型解析指令、根据解析结果创建smach状态机、驱动机器人按状态机顺序依次完成各项基元任务，从而完成各类复杂任务。本项目能够实现的基元任务有：导航、跟随、找人（正常找人、性别判断、人体姿态判断）、找物、语音交流、回答问题、抓取与传递（未实际实现），复杂任务可由这些组合而成。视频demo可见： 
    https://www.bilibili.com/video/BV1sN4y167kd/?share_source=copy_web&vd_source=f6674897b2b4292acafc8309d6808afd
smach状态机相关学习资料可见：
    https://www.guyuehome.com/42334
环境：使用虚拟环境GPSR，可在代码第一行修改编译器路径。
