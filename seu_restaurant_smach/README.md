# *SEU RoboCup@Home Restaurant 2026*

东南大学自动化学院 RoboCup@Home Restaurant 赛题代码

<!-- <div align="center">
  <span style="font-family: 'Times New Roman', 'SimSun', serif; font-size: 20px;">周艺舵</span><br/>
  <span style="font-family: 'Times New Roman', 'SimSun', serif; font-size: 16px;">东南大学自动化学院 (School of Automation, Southeast University)</span>
</div>

**Environment**

- Ubuntu 20.04
- ROS Noetic
- Python 3.10 -->

- 代码作者：东南大学自动化学院机器人工程系 周艺舵
- 开发基础环境：Ubuntu 20.04 + ROS Noetic


## 1. 状态机流程图
根据代码逻辑，状态机的流转如下：
```mermaid
graph TD
    A[FIND_WAVING_CUSTOMER] -->|success| B[BUILD_CUSTOMER_NAV_POSE]
    B -->|success| C[NAV_TO_CUSTOMER]
    C -->|success| D[ALIGN_AND_TAKE_ORDER]
    D -->|success| E[NAV_TO_BAR]
    E -->|success| F[SPEAK_ORDER_AT_BAR]
    F -->|success| G[SKIP_GRASP_PLACEHOLDER]
    G -->|success| H[NAV_BACK_TO_CUSTOMER]
    H -->|success| I[FINAL_DELIVER_SPEAK]
    I -->|success| J((task_success))
    
    A & B & C & D & E & F & H & I -->|failure| K((task_failure))
```

---
*SEU RoboCup Team 2026*