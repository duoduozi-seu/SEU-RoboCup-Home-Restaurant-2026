# 餐厅题开发交接记录（2026-03-19）

## 1. 本次工作的目标

原 GPSR 题目废弃后，继续在当前工作区内复用已有能力包，新增“餐厅”题完整流程代码。

用户要求：

- 不删除现有包。
- 复用 `seu_voice`、`seu_speech_recognition`。
- `seu_nav` 需要重写成 action 包装层，但下游真实导航调用方式不变。
- 新写的餐厅流程尽量使用 `actionlib` 通信。
- 正式脚本少写无意义的 `try/except`。
- 所有核心代码写详细中文注释。
- 正式入口直接使用各包 `scripts/` 下的脚本，不使用 `catkin_install_python`。


## 2. 已完成的包与职责

### 2.1 `seu_restaurant_msgs`

新建接口包，只放餐厅题 action：

- `action/FindWavingCustomer.action`
- `action/AlignAndTakeOrder.action`
- `action/RestaurantNav.action`

作用：

- 给挥手搜索、桌边接单、导航包装三类能力提供统一接口。


### 2.2 `seu_restaurant_wave`

新建挥手搜索包，核心文件：

- `seu_restaurant_wave/scripts/wave_detector_core.py`
- `seu_restaurant_wave/scripts/find_waving_customer_action_server.py`
- `seu_restaurant_wave/models/yolo26m-pose.pt`

实现内容：

- 以 `/home/duo/GPSR_ws_zyd/res_demo/realsense_mediapipe_wave.py` 为主要参考来源。
- 使用 RealSense + YOLO26 pose 做多人检测。
- 在 action server 内部一边缓慢原地转动底盘，一边实时检测挥手/举手候选人。
- 用深度选最近的人。
- 把相机系 3D 点通过 TF 转到 `map`。
- 返回最近挥手人的 `person_point_map` 和 `depth_m`。

注意：

- 这个包只负责“找到最近挥手顾客的地图点”。
- 不负责计算第一次导航停靠位姿。
- 不负责导航。


### 2.3 `seu_nav`

保留原包名，不删除旧内容。新增/重构：

- `seu_nav/scripts/nav_action_core.py`
- `seu_nav/scripts/nav_action_server.py`

保留：

- 旧 `seu_nav.srv`
- 旧 `scripts/nav_service.py`

实现内容：

- 新增 `RestaurantNav` action server，作为餐厅题上游统一导航入口。
- 支持双模式：
  - `TARGET_MODE_NAME`：从 `config/waypoints.yaml` 中按名字解析航点
  - `TARGET_MODE_POSE`：直接使用上游给定的 `nav_pose`
- 下游依旧通过 `task_nav` action 调真正导航，不改下游协议。
- 若 `pid_pose` 为空，则自动复制 `nav_pose`。
- 收到 cancel/preempt 时会同步取消下游导航目标。

兼容性说明：

- 旧 `seu_nav.srv` 没删，因为仓库里旧 SMACH 代码还在引用它。


### 2.4 `seu_restaurant_interaction`

新建桌边对正与接单包，核心文件：

- `seu_restaurant_interaction/scripts/person_align_core.py`
- `seu_restaurant_interaction/scripts/align_and_take_order_action_server.py`
- `seu_restaurant_interaction/models/yolo26m-pose.pt`

实现内容：

- 到达顾客附近后，只关注 2 米内最近的人。
- 使用 YOLO pose + RealSense 做“最近人”检测。
- 先做局部旋转对正，再做少量前后微调。
- 调用 `seu_voice` 播报固定问句。
- 调用 `seu_speech_recognition` 识别整句订单。
- 再播报复述句并等待 yes/no 类确认。
- 若识别到否定，则整轮重新问单，最多两轮。
- 成功后记录当前 `/robot_pose` 为 `service_pose_map` 返回。

确认词逻辑：

- 肯定词、否定词都集中写在 `person_align_core.py`。
- 已修复一个潜在问题：`incorrect` 不能被误判成 `correct`。


### 2.5 `seu_restaurant_smach`

新建主状态机包，核心文件：

- `seu_restaurant_smach/scripts/restaurant_states.py`
- `seu_restaurant_smach/scripts/restaurant_smach.py`

状态流程已经实现为：

1. `FIND_WAVING_CUSTOMER`
2. `BUILD_CUSTOMER_NAV_POSE`
3. `NAV_TO_CUSTOMER`
4. `ALIGN_AND_TAKE_ORDER`
5. `NAV_TO_BAR`
6. `SPEAK_ORDER_AT_BAR`
7. `SKIP_GRASP_PLACEHOLDER`
8. `NAV_BACK_TO_CUSTOMER`
9. `FINAL_DELIVER_SPEAK`

实现细节：

- `BUILD_CUSTOMER_NAV_POSE` 在 SMACH 包内计算第一次停靠位姿，不放到挥手搜索包里。
- 吧台固定使用 `StartPoint`。
- 吧台复述直接在状态机里调用 `seu_voice`。
- 抓取状态目前是占位状态，直接成功返回。
- 回顾客时直接回 `service_pose_map`，不做第二次局部对正。


## 3. 关键设计决定

### 3.1 不复用旧 `common` 感知封装

这次餐厅题的新功能没有建立在旧 `common` 包之上，而是直接在新包中写清楚感知流程，目的是提高可读性与可维护性。


### 3.2 脚本入口固定用 `scripts/`

根据用户最新要求，已经把以下包里的 `catkin_install_python(...)` 全部移除：

- `seu_restaurant_wave/CMakeLists.txt`
- `seu_restaurant_interaction/CMakeLists.txt`
- `seu_restaurant_smach/CMakeLists.txt`
- `seu_nav/CMakeLists.txt`

现在默认运行方式是直接使用源码中的脚本入口，例如：

- `rosrun seu_nav nav_action_server.py`
- `rosrun seu_restaurant_wave find_waving_customer_action_server.py`
- `rosrun seu_restaurant_interaction align_and_take_order_action_server.py`
- `rosrun seu_restaurant_smach restaurant_smach.py`

为了保证这种方式稳定可用，我已经把几个入口脚本改成“显式把当前脚本目录加入 `sys.path`”，这样同目录工具模块可以稳定导入。


## 4. 已完成的验证

### 4.1 单元测试

已新增并通过：

- `python3 -m unittest seu_restaurant_smach.tests.test_build_customer_nav_pose`
- `python3 -m unittest seu_restaurant_interaction.tests.test_confirmation_parser`
- `python3 -m unittest seu_restaurant_wave.tests.test_wave_geometry`
- `python3 -m unittest seu_nav.tests.test_nav_action_core`

合并执行结果：

- 共 10 个测试，全部通过。


### 4.2 Python 语法检查

已对以下脚本执行 `python3 -m py_compile`，通过：

- `seu_restaurant_wave/scripts/wave_detector_core.py`
- `seu_restaurant_wave/scripts/find_waving_customer_action_server.py`
- `seu_restaurant_interaction/scripts/person_align_core.py`
- `seu_restaurant_interaction/scripts/align_and_take_order_action_server.py`
- `seu_restaurant_smach/scripts/restaurant_states.py`
- `seu_restaurant_smach/scripts/restaurant_smach.py`
- `seu_nav/scripts/nav_action_core.py`
- `seu_nav/scripts/nav_action_server.py`


### 4.3 源码脚本直接导入检查

已验证以下源码脚本可直接从 `src/.../scripts/*.py` 路径导入，不依赖 `catkin_install_python` 包装：

- `seu_nav/scripts/nav_action_server.py`
- `seu_restaurant_smach/scripts/restaurant_smach.py`
- `seu_restaurant_interaction/scripts/align_and_take_order_action_server.py`
- `seu_restaurant_wave/scripts/find_waving_customer_action_server.py`


### 4.4 目标包构建

已执行并通过：

```bash
source /opt/ros/noetic/setup.bash
catkin_make --pkg seu_restaurant_msgs seu_restaurant_wave seu_restaurant_interaction seu_restaurant_smach seu_nav
```

说明：

- 构建过程中出现了一些来自工作区其他包的 CMake/PCL/VTK 警告。
- 本次新加的餐厅题相关包没有出现构建失败。


## 5. 目前已知状态

- 餐厅题主流程代码已经齐。
- `seu_nav` 的 action 包装已经齐。
- 挥手搜索、局部对正接单、SMACH 编排都已经落地。
- 抓取还是占位状态，没有实现真实抓取。
- 无地图导航的真正底层逻辑仍由外部已有导航能力负责，这一层没有被改写。


## 6. 如果下个线程继续做，建议优先检查

1. 在真实机器人上联调相机帧名是否和 `camera1_color_optical_frame` 一致。
2. 联调 `task_nav` 的 `TaskNavGoal` 字段和当前外部导航实现是否完全匹配。
3. 检查 `StartPoint` 在 `seu_nav/config/waypoints.yaml` 中是否存在且姿态正确。
4. 现场联调桌边 2m 过滤、转向速度、前后微调阈值是否需要调参。
5. 若以后要加抓取，只需要把 `SKIP_GRASP_PLACEHOLDER` 替换成真实 action 状态即可。


## 7. 本交接文档位置

本文件路径：

- `handoffs/restaurant_progress_2026-03-19.md`

