# seu_look_for_object

作者：东南大学自动化学院机器人工程系 周艺舵

## 1. 包功能概述

本包当前提供 4 个 ROS Service 接口：

- `find_object_service`
- `count_object_service`
- `relate_object_service`
- `analyze_object_service`

当前真实实现状态：

- `FIND`：已实现
- `COUNT`：已实现
- `RELATE`：占位，返回 `NOT_IMPLEMENTED`
- `ANALYZE`：占位，返回 `NOT_IMPLEMENTED`

当前运行脚本只有一个主节点：

- [scripts/object_query_server.py](/home/duo/GPSR_ws_zyd/GPSR_code/src/seu_look_for_object/scripts/object_query_server.py)

当前 `FIND` 的真实流程：

1. 根据 `search_scope` 从 `waypoints.yaml` 中提取该地点对应的全部精细锚点，例如 `bedroom_0`、`bedroom_1`
2. 逐个导航到精细锚点
3. 每个锚点连续检测 8 帧，命中 4 帧则认为该锚点找到了目标
4. 计算目标的全局坐标并发布到 RViz
5. 原地对正目标
6. 以视频流闭环监控的方式匀速靠近到目标前 `1.0m +- 0.15m`
7. 成功后返回

当前 `COUNT` 的真实流程：

1. 根据 `search_scope` 提取全部精细锚点
2. 逐个导航到精细锚点
3. 每个锚点检测 15 帧
4. 对 15 帧里的“当前帧目标个数”做投票
5. 取票数最多的数量作为该锚点真实计数
6. 把所有锚点的计数直接累加后返回

## 2. 启动命令

### 2.1 单独启动本包

```bash
rosrun seu_look_for_object object_query_server.py
```

### 2.2 带参数启动示例

```bash
rosrun seu_look_for_object object_query_server.py \
  _anchor_yaml_path:=/home/duo/GPSR_ws_zyd/GPSR_code/src/seu_nav/config/waypoints.yaml \
  _yolo_model_path:=/home/duo/GPSR_ws_zyd/GPSR_code/src/seu_look_for_object/scripts/yoloe26/yoloe-26x-seg.pt \
  _refer_image_path:=/home/duo/GPSR_ws_zyd/GPSR_code/src/seu_look_for_object/scripts/yoloe26/image_prompt.JPG
```

### 2.3 Service 调试命令

`FIND`

```bash
rosservice call /find_object_service "target_class: 'apple'
search_scope: 'bedroom'
timeout_sec: 60.0"
```

`COUNT`

```bash
rosservice call /count_object_service "target_class: 'apple'
search_scope: 'bedroom'
timeout_sec: 60.0"
```

`RELATE`

```bash
rosservice call /relate_object_service "target_class: 'apple'
reference_class: 'lays'
relation: 'behind'
search_scope: 'bedroom'
timeout_sec: 60.0"
```

`ANALYZE`

```bash
rosservice call /analyze_object_service "target_class: 'apple'
metric: 'biggest'
search_scope: 'bedroom'
timeout_sec: 60.0"
```

## 3. 重点可能需要修改的硬编码变量

下面只列当前最关键、最容易因为环境变化而需要修改的默认值。  
这些值大多在 [scripts/object_query_server.py](/home/duo/GPSR_ws_zyd/GPSR_code/src/seu_look_for_object/scripts/object_query_server.py) 里定义，并且绝大多数都可以用 ROS 参数覆盖。

### 3.1 路径与模型相关

| 位置 | 变量 | 当前值 | 说明 |
|---|---|---|---|
| `object_query_server.py` | `DEFAULT_WAYPOINT_PATH` | `/home/duo/GPSR_ws_zyd/GPSR_code/src/seu_nav/config/waypoints.yaml` | 默认精细锚点 YAML |
| `object_query_server.py` | `DEFAULT_MODEL_PATH` | `/home/duo/GPSR_ws_zyd/GPSR_code/src/seu_look_for_object/scripts/yoloe26/yoloe-26x-seg.pt` | 默认 YOLOE 模型 |
| `object_query_server.py` | `DEFAULT_REFER_IMAGE_PATH` | `/home/duo/GPSR_ws_zyd/GPSR_code/src/seu_look_for_object/scripts/yoloe26/image_prompt.JPG` | 默认 refer image |
| `object_query_server.py` | `DEFAULT_REFER_BBOX` | `[1054.0, 1695.0, 1460.0, 2734.0]` | refer image 中的提示框 |

### 3.2 坐标系、导航和话题名

| 位置 | 参数/变量 | 当前默认值 | 说明 |
|---|---|---|---|
| `object_query_server.py` | `~task_nav_action_name` | `task_nav` | 导航 action 名称 |
| `object_query_server.py` | `~cmd_vel_topic` | `/cmd_vel` | 底盘速度控制话题 |
| `object_query_server.py` | `~map_frame` | `map` | 全局坐标系 |
| `object_query_server.py` | `~base_frame` | `base_link` | 机器人底盘坐标系 |
| `object_query_server.py` | `~camera_frame` | `camera_color_optical_frame` | D435i 彩色相机坐标系 |
| `object_query_server.py` | `~candidate_pose_topic` | `~candidate_pose` | 候选点 Pose 发布 |
| `object_query_server.py` | `~candidate_marker_topic` | `~candidate_marker` | 候选点 Marker 发布 |

### 3.3 D435i 与检测相关

| 位置 | 参数/变量 | 当前默认值 | 说明 |
|---|---|---|---|
| `object_query_server.py` | `~realsense_width` | `1280` | 彩色/深度分辨率宽 |
| `object_query_server.py` | `~realsense_height` | `720` | 彩色/深度分辨率高 |
| `object_query_server.py` | `~realsense_fps` | `30` | 相机帧率 |
| `object_query_server.py` | `~detection_conf` | `0.15` | YOLO 置信度阈值 |
| `object_query_server.py` | `~find_frame_count` | `8` | FIND 每锚点检测帧数 |
| `object_query_server.py` | `~find_hit_count` | `4` | FIND 每锚点最少命中帧数 |
| `object_query_server.py` | `~count_frame_count` | `15` | COUNT 每锚点统计帧数 |
| `object_query_server.py` | `~preview_enabled` | `True` | 是否弹 OpenCV 视频窗口 |

### 3.4 对正与靠近控制相关

| 位置 | 参数/变量 | 当前默认值 | 说明 |
|---|---|---|---|
| `object_query_server.py` | `~align_center_tolerance` | `0.08` | 对正时允许的图像中心误差 |
| `object_query_server.py` | `~align_angular_kp` | `1.2` | 对正时纯比例旋转系数 |
| `object_query_server.py` | `~max_angular_speed` | `0.6` | 对正角速度上限，单位 `rad/s` |
| `object_query_server.py` | `~approach_target_distance` | `1.0` | 靠近目标距离，单位 `m` |
| `object_query_server.py` | `~approach_stop_tolerance` | `0.15` | 到达判断容差，单位 `m` |
| `object_query_server.py` | `~approach_linear_speed` | `0.15` | 靠近时匀速前进速度，单位 `m/s` |
| `object_query_server.py` | `~control_rate_hz` | `12.0` | 控制循环频率 |

## 4. 包的消息、服务、输入输出与格式

### 4.1 自定义消息

#### `msg/ObjectDetectionResult.msg`

```text
string label
float32 score
geometry_msgs/PoseStamped pose
bool pose_valid
float32[] bbox
float32 distance
```

含义：

- `label`：当前代码返回给上层的物体名
- `score`：YOLO 置信度
- `pose`：目标在 `map` 下的位置
- `pose_valid`：是否成功得到 3D 位姿
- `bbox`：图像中的检测框 `[x1, y1, x2, y2]`
- `distance`：D435i 测得的目标距离，单位 `m`

#### `msg/ObjectQueryResult.msg`

```text
bool success
string status_code
string message
string query_mode
string target_class
string reference_class
string resolved_label
int32 count
ObjectDetectionResult selected_object
ObjectDetectionResult[] candidate_objects
```

含义：

- `success`：是否成功
- `status_code`：状态码，如 `SUCCESS`、`NOT_FOUND`、`NOT_IMPLEMENTED`
- `message`：日志/错误说明
- `query_mode`：`FIND` / `COUNT` / `RELATE` / `ANALYZE`
- `target_class`：请求中的目标类别字符串
- `reference_class`：关系查询参考物
- `resolved_label`：最终物体名
- `count`：计数结果
- `selected_object`：最终选中的目标
- `candidate_objects`：中间候选列表

### 4.2 自定义服务

#### `srv/FindObject.srv`

请求：

```text
string target_class
string search_scope
float32 timeout_sec
```

返回：

```text
ObjectQueryResult result
```

#### `srv/CountObject.srv`

请求：

```text
string target_class
string search_scope
float32 timeout_sec
```

返回：

```text
ObjectQueryResult result
```

#### `srv/RelateObject.srv`

请求：

```text
string target_class
string reference_class
string relation
string search_scope
float32 timeout_sec
```

返回：

```text
ObjectQueryResult result
```

#### `srv/AnalyzeObject.srv`

请求：

```text
string target_class
string metric
string search_scope
float32 timeout_sec
```

返回：

```text
ObjectQueryResult result
```

### 4.3 运行时话题/动作依赖

本包当前主要依赖这些 ROS 通道：

- 发布：
  - `/cmd_vel`，类型 `geometry_msgs/Twist`
  - `~candidate_pose`，类型 `geometry_msgs/PoseStamped`
  - `~candidate_marker`，类型 `visualization_msgs/Marker`
- 使用：
  - `task_nav` action，类型 `tracer_nav/TaskNavAction`
  - tf：`map <- camera_color_optical_frame`
  - RealSense D435i 彩色图 + 深度图

## 5. 当前对正与靠近逻辑确认

### 5.1 旋转对正是否用了 PID

没有。

当前旋转对正函数是 [object_query_server.py](/home/duo/GPSR_ws_zyd/GPSR_code/src/seu_look_for_object/scripts/object_query_server.py) 里的 `_align_to_detection()`，它用的是：

- 图像中心误差 `center_error`
- 纯比例控制 `angular_z = -align_angular_kp * center_error`
- 再用 `max_angular_speed` 做限幅

也就是说：

- 不是 PID
- 是纯 P 控制

### 5.2 当前旋转速度常量是多少

严格来说，对正时没有固定单一角速度常量，因为它会跟 `center_error` 成比例变化。

当前两个关键常量是：

- `~align_angular_kp = 1.2`
- `~max_angular_speed = 0.6 rad/s`

如果你只问“当前最大旋转速度上限是多少”，答案是：

- `0.6 rad/s`

### 5.3 当前靠近逻辑

当前靠近函数是 `_approach_to_target_with_constant_speed()`。

它已经不是 PID，当前逻辑是：

- 持续看视频流
- 持续读取实时距离
- 如果距离大于 `1.0 + 0.15 m`，则以 `0.15 m/s` 匀速前进
- 如果距离小于 `1.0 - 0.15 m`，则以 `0.15 m/s` 匀速后退
- 如果距离落入 `1.0m +- 0.15m`，则立即停车并返回成功

当前前进速度常量是：

- `~approach_linear_speed = 0.15 m/s`

## 6. 当前识别逻辑的重要限制

### 6.1 当前是不是不管检测什么，实际上都在检测 image prompt 里的那个物体

是。

当前代码的视觉输入并没有根据 `target_class` 去切换不同的 refer image，也没有根据命令词选择不同的视觉 prompt。

现在真正用于视觉检测的只有：

- `~refer_image_path`
- `~refer_bbox`

默认就是：

- `/home/duo/GPSR_ws_zyd/GPSR_code/src/seu_look_for_object/scripts/yoloe26/image_prompt.JPG`
- `[1054.0, 1695.0, 1460.0, 2734.0]`

所以当前 YOLOE 实际上是在用这一张 refer image 做视觉提示检测。

### 6.2 那现在代码里是怎么判断“识别到的东西就是命令里让我找的东西”的

当前代码**并没有真正做这个语义判断**。

现在的做法是：

1. YOLOE 用固定的 `refer_image` 做检测
2. 只要检测到框，就认为“看到了当前目标”
3. 然后把请求里的 `target_class` 直接写回结果里的 `label` / `resolved_label`

也就是说，当前代码里：

- `target_class` 主要用于日志、返回结果和上层接口字段
- `target_class` **没有真正参与视觉模板切换**
- `target_class` **也没有和 refer image 做一致性检查**

这意味着当前版本的真实语义是：

- “检测 refer image 指向的那个物品，并把它按请求里的 `target_class` 名称返回”

所以如果后面要支持真正按命令找不同物品，必须补下面至少一项：

1. `target_class -> refer_image_path` 的映射表
2. `target_class -> refer_bbox` 的映射表
3. 或者彻底改成文本类别/开放词汇检测，而不是固定 refer image

## 7. 当前代码文件入口定位

最重要的几个入口：

- `vote_count_value()`：计数投票 helper
- `handle_find()`：`FIND` service 入口
- `handle_count()`：`COUNT` service 入口
- `_run_find_flow()`：FIND 主流程
- `_run_count_flow()`：COUNT 主流程
- `_align_to_detection()`：原地对正
- `_approach_to_target_with_constant_speed()`：匀速靠近

如果你后面继续改逻辑，基本都在 [scripts/object_query_server.py](/home/duo/GPSR_ws_zyd/GPSR_code/src/seu_look_for_object/scripts/object_query_server.py) 里完成，不需要再去跳很多文件。
