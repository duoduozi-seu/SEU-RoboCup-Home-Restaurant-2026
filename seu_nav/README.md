# seu_nav —— SEU 导航服务包 (2026版)

- 作者：东南大学自动化学院机器人工程系 周艺舵

## 概述

`seu_nav` 是 GPSR 项目的导航服务封装包。它对外提供一个 ROS Service (`nav_service`)，上层状态机只需传入**目标地点名称**（字符串），即可完成导航。

内部实现上，`seu_nav` 作为 **ROS Service Server** 接收请求，并作为 **ROS ActionLib Client** 连接 `catkin_robocup_2` 提供的 `task_nav` Action Server 发送带双坐标 (`nav_pose` 和 `pid_pose`) 的 `TaskNavGoal`。

---

## 2026年 核心架构更新 🚀

1. **底层交互框架升级 (Service → ActionLib)**：摒弃了以前直接控制 `move_base` 的方式，改为向最新的 `catkin_robocup_2` 提供 Action Goal。这使得导航具备了更清晰的状态查询和超时/打断机制。
2. **双位姿架构支持 (Double Pose)**：对齐最新的导航底盘要求，将目标解耦为 `nav_pose` (用于底盘粗定位行驶) 与 `pid_pose` (用于对接端点/手眼微调)，YAML 数据结构完全跟随上游 `catkin_robocup_2` 作者的示范格式。
3. **"到手即用" 的解耦配置**：取消了启动时的参数传递需求。内部代码直接硬编码解析与本包配套的 `config/waypoints.yaml`，并原封不动地存储原生 YAML `dict`。当底盘作者下发新版 yaml 时，支持直接替换文件即可生效测试。
4. **混合容错的名字路由匹配**：增强了 NLP 到导航地点的联想映射。兼顾了底盘作者要求的驼峰严格匹配（如 `StartPoint`）和状态机器人的自然语言宽松匹配（如 `start point`, `START_POINT`）。

---

## 1. 前置条件：启动 catkin_robocup_2 导航系统

`seu_nav` 依赖 `catkin_robocup_2` 提供的导航底层。在运行 `seu_nav` 之前，**必须先在 `catkin_robocup_2` 工作空间中启动导航**。

### 1.1 启动全套导航（底盘 + 定位 + move_base）

```bash
# 终端 1：source catkin_robocup_2 工作空间
source /home/duo/GPSR_ws_zyd/catkin_robocup_2/devel/setup.bash

# 启动导航（包含底盘驱动、激光定位、move_base）
roslaunch tracer_nav nav_all.launch
```

> **参数说明：**
> - `local_planner` (默认 `true`)：`true` 使用 TEB 局部规划器，`false` 使用 DWA
> - `map_pcd`：PCD 点云地图路径（用于 fast_lio 定位）
> - `map_yaml`：2D 栅格地图路径（用于 move_base 代价地图）

### 1.2 启动 task_nav Action Server

`nav_all.launch` **不会**自动启动 `task_nav_server`，需要手动启动：

```bash
# 终端 2（同一工作空间 source 后）
rosrun tracer_nav task_nav_server.py
```

> 或者也可以使用 `nav_with_waypoints.launch` 一键启动（该 launch 会同时启动 nav_all + task_nav_server + 交互测试节点）：
> ```bash
> roslaunch tracer_nav nav_with_waypoints.launch
> ```

---

## 2. 启动 seu_nav 导航服务 (启动格式)

```bash
# 终端 3：source GPSR_code 工作空间
source /home/duo/GPSR_ws_zyd/GPSR_code/devel/setup.bash

# 💡 2026 新特性：只需一行无参命令即可启动，航点文件已被硬编码指向配置目录。
rosrun seu_nav nav_service.py

# (可选) 指定导航超时时间（默认 120 秒）
rosrun seu_nav nav_service.py _timeout:=180
```

### ROS 参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `~timeout` | int | `120` | 单次导航超时时间（秒）。超时会自动取消 Action 目标。 |

---

## 3. 服务接口说明

### 服务名称

`nav_service`

### 服务类型

`seu_nav/seu_nav`

### 输入 (Request)

| 字段 | 类型 | 说明 |
|------|------|------|
| `des` | `string` | 目标地点名称，如 `"Kitchen"`、`"living room"`、`"start_point"` |

> **名称匹配规则 (重要)：**  
> 1. **期望格式 (精确匹配)**: 首选与 `waypoints.yaml` 中的键名完全一致（例如原样输入 `StartPoint`）。  
> 2. **兼容格式 (宽松匹配)**: 若未精确命中，系统会自动**忽略大小写**、并**将下划线 `_` 和空格 ` ` 视为等价**。  
> 举例：若 YAML 中的键名是 `StartPoint`，无论你在服务请求传入 `"start_point"`、`"start point"` 还是 `"START POINT"`，均能成功回退匹配到 `StartPoint`。

### 输出 (Response)

| 字段 | 类型 | 说明 |
|------|------|------|
| `state` | `bool` | 导航是否成功。`True` = Action Server 报告到达目标，`False` = 失败/超时/未知地点 |
| `errormsg` | `string` | 错误信息。成功时为空字符串，失败时包含具体原因 |

### 可能的错误信息

| errormsg | 含义 |
|----------|------|
| `未知地点: "xxx"，可用地点: [...]` | 请求的地点名称在航点 YAML 中不存在 |
| `task_nav Action Server 不可用` | task_nav_server 未启动或连接超时 |
| `导航超时 (120s)` | 导航未在超时时间内完成 |
| `导航失败 (ActionState=N)` | move_base 导航失败（如路径不可达） |

### 终端测试

```bash
# 导航到 kitchen
rosservice call /nav_service "{des: 'kitchen'}"

# 导航到 living room（空格形式）
rosservice call /nav_service "{des: 'living room'}"
```

---

## 4. 航点配置文件 (waypoints.yaml) 的期望组织格式

本包使用独立分离的航点文件 `config/waypoints.yaml`。为了与底盘作者 (`catkin_robocup_2`) 保持代码解析上的一致，此处强制期望要求每个地点包含 **`nav_pose`** 与 **`pid_pose`** 两套位姿体系。

> **提示：如果上游发送了新的 yaml，直接覆盖此 `waypoints.yaml` 即可开箱即用。**

格式示例如下：

```yaml
waypoints:
  StartPoint:
    nav_pose:
      position:
        x: 0.0
        y: 0.0
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    pid_pose:
      position:
        x: 0.0
        y: 0.0
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
```

### 如何获取航点坐标

1. 启动导航系统，在 RViz 中将机器人移动到目标位置
2. 终端执行以下命令获取当前位姿：
   ```bash
   rostopic echo -n 1 /localization
   ```
3. 将输出的 `position` 和 `orientation` 填入 YAML 文件

---

## 5. 输入输出信号与系统架构 (ActionLib & Services)

```text
┌─────────────────────────────────────────────────────┐
│  gpsr_smach.py (GPSR 状态机)                          │
│  发出信号：rosservice call /nav_service "{des: 'xxx'}"│
└────────────────────┬────────────────────────────────┘
                     │ ROS Service (seu_nav/seu_nav)
                     ▼
┌─────────────────────────────────────────────────────┐
│  seu_nav / nav_service.py                            │
│  - 作为 Service Server 等待请求                        │
│  - 匹配 YAML 文件中的目标地点与双级 Pose (nav & pid)    │
│  - 作为 ActionLib Client 发出 TaskNavGoal (阻塞超时)   │
└────────────────────┬────────────────────────────────┘
                     │ ROS ActionLib (tracer_nav/TaskNavAction)
                     ▼
┌─────────────────────────────────────────────────────┐
│  tracer_nav / task_nav_server.py  (catkin_robocup_2) │
│  - 等待 TaskNavGoal                                   │
│  - 转发至底层 move_base 或执行其他对准逻辑                │
└────────────────────┬────────────────────────────────┘
                     │ ActionLib (move_base)
                     ▼
┌─────────────────────────────────────────────────────┐
│  move_base (ROS Navigation Stack)                    │
│  - 全局规划 + 局部规划 + 底盘控制                       │
└─────────────────────────────────────────────────────┘
```

---

## 6. 文件结构

```
seu_nav/
├── CMakeLists.txt          # 构建配置
├── package.xml             # 包依赖声明
├── README.md               # 本文档
├── config/
│   └── waypoints.yaml      # 航点配置文件（需根据实际场地标定）
├── scripts/
│   └── nav_service.py      # 导航服务节点（核心代码）
└── srv/
    └── seu_nav.srv          # 服务消息定义 (string des → bool state + string errormsg)
```