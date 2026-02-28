# seu_person_analyse 代码修改记录

> **日期**: 2025-12-25  
> **目的**: 使程序在新环境（非原始开发环境）中运行

---

## 1. 文件修改列表

### 1.1 `/home/duo/GPSR_ws_zyd/GPSR_code/src/common/camera.py`

| 行号 | 原始内容 | 修改后 | 修改原因 |
|------|----------|--------|----------|
| 1 | `#! /home/syw/anaconda3/envs/GPSR/bin/python` | `#!/usr/bin/env python3` | 原路径指向开发者电脑的 Python 解释器 |
| 7-10 | *(无)* | 添加动态路径解析代码 | 支持在任意位置运行脚本 |
| 97 | `self.model = YOLO("/home/syw/.../epoch10.pt", ...)` | `self.model = YOLO(os.path.join(_COMMON_DIR, "models/epoch10.pt"), ...)` | 模型路径硬编码 |
| 129 | `def __init__(self, model_path='/home/syw/.../yolov8n.pt'):` | 动态路径解析 | 同上 |
| 646 | `cv2.imwrite(f"/home/syw/.../images/{name}.png", image)` | `cv2.imwrite(os.path.join(_COMMON_DIR, f"images/{name}.png"), image)` | 同上 |
| 665 | `image_path = f"/home/syw/.../images/{image}_{i}.png"` | `image_path = os.path.join(_COMMON_DIR, ...)` | 同上 |

**第7-10行添加的代码:**
```python
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_COMMON_DIR = _SCRIPT_DIR
_SRC_DIR = os.path.dirname(_SCRIPT_DIR)
```

---

### 1.2 `/home/duo/GPSR_ws_zyd/GPSR_code/src/seu_person_analyse/scripts/person_analyse.py`

| 行号 | 原始内容 | 修改后 | 修改原因 |
|------|----------|--------|----------|
| 1 | `#! /home/syw/anaconda3/envs/GPSR/bin/python` | `#!/usr/bin/env python3` | 原路径硬编码 |
| 5 | *(无)* | `import os` | 新增导入 |
| 7 | `sys.path.append("/home/syw/.../GPSR/src")` | 动态路径解析 | 原路径硬编码 |

---

## 2. Conda 环境修改 (yolov11)

| 操作 | 说明 |
|------|------|
| Python 降级 | 3.10 → 3.8（兼容系统 PyKDL） |
| 安装 CUDA | `pytorch-cuda=12.1, cudatoolkit` |
| 安装 PyTorch | `pytorch 2.4.1 + torchvision 0.20.0` |
| 其他依赖 | tensorflow 2.13.1, mediapipe 0.10.9, deepface, ultralytics 等 |

---

## 3. 新增文件

| 文件 | 说明 |
|------|------|
| `scripts/run_service.sh` | 启动脚本，设置正确的环境变量 |
| `test/mock_topics.py` | 模拟外部 ROS 话题发布器 |
| `test/test.py` | 离线功能测试脚本 |

---

## 4. 运行方法

```bash
# 1. 确保 roscore 已运行
roscore

# 2. 运行服务（使用启动脚本）
./src/seu_person_analyse/scripts/run_service.sh

# 3. 调用服务
rosservice call /person_analyse_service "feature: 'all'"
```
