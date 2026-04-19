#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../../../.." && pwd)"
WORKSPACE_SETUP="${WORKSPACE_DIR}/devel/setup.bash"

source /opt/ros/noetic/setup.bash

if [ -f "${WORKSPACE_SETUP}" ]; then
    source "${WORKSPACE_SETUP}"
fi

gnome-terminal --title="demo" -- bash -lc "source /opt/ros/noetic/setup.bash; [ -f '${WORKSPACE_SETUP}' ] && source '${WORKSPACE_SETUP}'; roslaunch piper_with_gripper_moveit demo.launch; exec bash"

sleep 3

roslaunch seu_restaurant_smach init.launch launch_piper_demo:=false
