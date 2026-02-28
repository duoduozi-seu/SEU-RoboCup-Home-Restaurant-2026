要是还用这个机械臂的话，这个软件包就能完成大部分任务，当然最好，请，尽量不要使用这个机械臂，除非放弃tidy up自己抓取，房老师实验室里还有一个比较好的机械臂
非要用的话，首先要把dobot的ros demo跑通了，确保配置没有问题，资料里有
然后跑一跑dobot_control.py,重点是把动态链接库成功导入，注意软链接问题 #(DobotControl.py,在DobotDllType.py第466行修改libDobotDll.so路径,注意可能出现文件软链接问题（file too short）)
要想抓取，需要做手眼标定，可参考https://blog.csdn.net/weixin_43030192/article/details/117662685，注意python要用3.6
dobot的功能已经写成了服务，直接请求就可以了

