简介：
    这是东南大学robocup校队@home组关于Robocup世界杯中国赛GPSR题目的源代码，由校队成员zzn，hongzhi，txy以及hcz、xxy合作编写。本功能包在ubuntu20.04与ros noetic环境下开发完成
文件组成：
    seu_gpsr:元功能包文件，无实际作用
    seu_smach_main:赛题主程序功能包，负责整个赛题流程，采用smach状态机编写
    seu_llm:大语言模型解析模块，基于百度文心一言4.0大语言模型，作为核心指令解析器，负责解析指令以及回答常见问题，在线
    seu_follow:跟随模块,基于mediapipe人体骨骼模型与手势模型，离线
    seu_look_for_object:寻找物体模块，基于yolov7物品识别模型，离线
    seu_look_for_person:寻找人模块，基于face_recognition(找人)、deepface(判断性别)、mediapipe(判断人体姿态)，离线
    seu_nav:导航模块，基于move_base，离线
    seu_speech_recognition:语音识别模块，基于百度语音识别技术，在线
    seu_robot_msgs:存放自定义消息
    seu_tracer_mini:存放松灵底盘的一些urdf文件，便于在分布式通信时在上位机能够查看
环境安装教程：
    1. 修改conda.yml最后一行虚拟环境路径，修改为目标anaconda的envs下就行
    2. 用如下命令创建新conda环境：
        conda env create -f  conda.yml
        此时会将yml中conda库进行安装,自测pip包安装时间过长，所以在第三步单独安装pip的包。
    3. 激活已创建的虚拟环境并安装pip的依赖包
        conda activate GPSR
        pip3 install torch==1.8.2 torchvision==0.9.2 torchaudio==0.8.2 --extra-index-url https://download.pytorch.org/whl/lts/1.8/cu111
        pip install -r pip_new.txt
使用方法：
    1.与控制底盘的jetson nano配置分布式通信：通过网线链接nano与笔记本电脑，将ipv4设为 与其他计算机共享。其余配置分布式通信教程可见如下链接：http://www.autolabor.com.cn/book/ROSTutorials/5/44-rosfen-bu-shi-tong-xin.html
    2.启动主机（笔记本）roscore
    3.采用ssh与nano建立远程连接：
        # user: robocup@10.42.0.169
        # password: seu2022

        # 生成密钥
        ssh-keygen
        # 上传公钥
        ssh-copy-id robocup@10.42.0.169
        # 连接nano并初始化环境 10.42.0.169
        ssh -X robocup@10.42.0.169
        cd tracer_ws
        catkin_make
        source devel/setup.bash 
        # 首次上电需执行，给雷达赋予权限
        sudo modprobe gs_usb 
        sudo ip link set can0 up type can bitrate 500000
        sudo chmod 777 /dev/ttyACM0  
    4.完成连接后，启动底盘
        # 启动底盘
        roslaunch tracer_bringup tracer_robot_base.launch
    5.另开一终端，重复连接nano并初始化环境 10.42.0.169，启动导航
        # 启动导航
        roslaunch tracer_bringup tidy_up_nav.launch
    6.进入/seu_smach_main/scripts路径后，使用脚本开启各基元模块服务：
        ./gpsr_bringup.sh
    7.启动主程序，新开一个终端，运行如下命令:
        rosrun seu_smach_main gpsr_smach.py
    8.观测状态机运行情况可运行如下命令：（需要先退出anaconda环境）
        conda deactivate
        rosrun smach_viewer smach_viewer.py 
视频demo：
    https://www.bilibili.com/video/BV1sN4y167kd/?share_source=copy_web&vd_source=f6674897b2b4292acafc8309d6808afd
注意事项：
    1. 每个功能模块都有各自的readme，使用前请耐心阅读
    2. 代码中有一些路径采用绝对路径，均已在代码开头处声明，请自行更换
    3. 一些功能模块需使用虚拟环境GPSR运行，请自行修改代码第一行的编译器路径
