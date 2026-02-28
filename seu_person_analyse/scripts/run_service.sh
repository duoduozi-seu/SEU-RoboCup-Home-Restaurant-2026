#!/bin/bash
# seu_person_analyse 运行脚本
# ==========================
# 该脚本设置正确的环境变量并运行 person_analyse 服务

set -e

# 禁用用户本地 Python 包
export PYTHONNOUSERSITE=1

# 激活 conda 环境
source ~/anaconda3/etc/profile.d/conda.sh
conda activate yolov11

# 获取路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"
SRC_DIR="$(dirname "$PACKAGE_DIR")"
WS_DIR="$(dirname "$SRC_DIR")"

CONDA_PREFIX="$(python -c 'import sys; print(sys.prefix)')"
CONDA_SITE_PACKAGES="$CONDA_PREFIX/lib/python3.8/site-packages"
CONDA_LIB="$CONDA_PREFIX/lib"

# 关键：预加载 conda 的 libtiff 库来覆盖系统版本，解决 cv_bridge 符号冲突
export LD_PRELOAD="$CONDA_LIB/libtiff.so.6:$CONDA_LIB/libtiffxx.so.6"

# 设置 LD_LIBRARY_PATH - conda 库必须在最前面
export LD_LIBRARY_PATH="$CONDA_LIB:/opt/ros/noetic/lib:$WS_DIR/devel/lib"

# ROS 环境变量
export ROS_PACKAGE_PATH="/opt/ros/noetic/share:$SRC_DIR"
export ROS_MASTER_URI="http://localhost:11311"
export ROS_ROOT="/opt/ros/noetic/share/ros"
export ROS_DISTRO="noetic"

# PYTHONPATH - conda 优先
export PYTHONPATH="$CONDA_SITE_PACKAGES:$SRC_DIR:$WS_DIR/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages"

echo "=========================================="
echo "seu_person_analyse 服务启动脚本"
echo "=========================================="
echo "Conda: yolov11 | Python: $(python --version 2>&1)"
echo "LD_PRELOAD: $LD_PRELOAD"
echo "=========================================="

cd "$SRC_DIR"
exec python seu_person_analyse/scripts/person_analyse.py "$@"
