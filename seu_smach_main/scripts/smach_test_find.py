#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""由 Antigravity 生成：用于验证从 SMACH 调用找物服务的极简测试脚本。"""

import rospy
import smach
import smach_ros
import sys
import os

# 将代码路径加入系统路径，确保能找到 gpsr_smach 及其消息定义
WORKSPACE_SRC = "/home/duo/GPSR_ws_zyd/catkin_robocup_2/src/GPSR_code/src"
SCRIPTS_DIR = os.path.join(WORKSPACE_SRC, "seu_smach_main/scripts")
sys.path.append(WORKSPACE_SRC)
sys.path.append(SCRIPTS_DIR)
os.chdir(SCRIPTS_DIR) # 切换工作目录到脚本所在目录，进一步确保相对导入正常

import traceback

try:
    import gpsr_smach
except Exception:
    print("--- 导入捕获异常 ---")
    traceback.print_exc()
    print("------------------")
    print("错误提示：无法加载 gpsr_smach。")
    print("原因分析：很可能是环境不对。请检查是否安装了 smach, smach_ros, fpdf 等依赖。")
    print(f"当前 Python 路径: {sys.executable}")
    sys.exit(1)

def main():
    rospy.init_node('smach_test_find_node')
    
    rospy.loginfo("--- SMACH 找物联调测试开始 ---")
    
    # 【核心步骤】强制重置 gpsr_smach 的 debug 标志
    # 否则状态机会因为 debug=True 而直接返回 success，不会产生真实服务请求。
    gpsr_smach.debug = False
    
    sm = smach.StateMachine(outcomes=['TEST_FINISHED'])
    
    # 准备测试参数
    # 参数格式：[物品类别, 搜索房间名, 超时时间]
    # 注意：房间名必须在 waypoints.yaml 中有对应的 <房间名>_0 锚点
    test_args = ['apple', 'Kitchen', '30.0']
    
    with sm:
        rospy.loginfo("添加 FindObjectState 到状态机...")
        smach.StateMachine.add('VERIFY_FIND_CHAIN', 
                               gpsr_smach.FindObjectState(test_args),
                               transitions={'success':'TEST_FINISHED', 
                                            'failure':'TEST_FINISHED'})

    rospy.loginfo("开始执行状态机...")
    outcome = sm.execute()
    rospy.loginfo("测试结束，结果状态: %s", outcome)
    rospy.loginfo("提示：如果 object_query_server 终端输出了请求日志，则调用链打通成功！")

if __name__ == '__main__':
    main()
