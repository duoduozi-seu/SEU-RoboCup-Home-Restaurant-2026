#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../../../.." && pwd)"
WORKSPACE_SETUP="${WORKSPACE_DIR}/devel/setup.bash"
DEMO_SHELL_PID_FILE="/tmp/seu_restaurant_demo_shell_${$}.pid"
DEMO_ROSLAUNCH_PID_FILE="/tmp/seu_restaurant_demo_roslaunch_${$}.pid"
MESHCAT_SHELL_PID_FILE="/tmp/seu_restaurant_meshcat_shell_${$}.pid"
MESHCAT_SERVER_PID_FILE="/tmp/seu_restaurant_meshcat_server_${$}.pid"

source /opt/ros/noetic/setup.bash

if [ -f "${WORKSPACE_SETUP}" ]; then
    source "${WORKSPACE_SETUP}"
fi

cleanup_demo_terminal() {
    local demo_shell_pid=""
    local demo_roslaunch_pid=""
    local meshcat_shell_pid=""
    local meshcat_server_pid=""

    if [ -f "${DEMO_SHELL_PID_FILE}" ]; then
        demo_shell_pid="$(cat "${DEMO_SHELL_PID_FILE}")"
    fi
    if [ -f "${DEMO_ROSLAUNCH_PID_FILE}" ]; then
        demo_roslaunch_pid="$(cat "${DEMO_ROSLAUNCH_PID_FILE}")"
    fi
    if [ -f "${MESHCAT_SHELL_PID_FILE}" ]; then
        meshcat_shell_pid="$(cat "${MESHCAT_SHELL_PID_FILE}")"
    fi
    if [ -f "${MESHCAT_SERVER_PID_FILE}" ]; then
        meshcat_server_pid="$(cat "${MESHCAT_SERVER_PID_FILE}")"
    fi

    if [ -n "${demo_shell_pid}" ] && kill -0 "${demo_shell_pid}" 2>/dev/null; then
        kill -TERM "${demo_shell_pid}" 2>/dev/null || true
        for _ in $(seq 1 20); do
            if ! kill -0 "${demo_shell_pid}" 2>/dev/null; then
                break
            fi
            sleep 0.1
        done
    fi

    if [ -n "${demo_roslaunch_pid}" ] && kill -0 "${demo_roslaunch_pid}" 2>/dev/null; then
        kill -INT "${demo_roslaunch_pid}" 2>/dev/null || true
        for _ in $(seq 1 20); do
            if ! kill -0 "${demo_roslaunch_pid}" 2>/dev/null; then
                break
            fi
            sleep 0.1
        done
        if kill -0 "${demo_roslaunch_pid}" 2>/dev/null; then
            kill -TERM "${demo_roslaunch_pid}" 2>/dev/null || true
        fi
    fi

    if [ -n "${meshcat_shell_pid}" ] && kill -0 "${meshcat_shell_pid}" 2>/dev/null; then
        kill -TERM "${meshcat_shell_pid}" 2>/dev/null || true
        for _ in $(seq 1 20); do
            if ! kill -0 "${meshcat_shell_pid}" 2>/dev/null; then
                break
            fi
            sleep 0.1
        done
    fi

    if [ -n "${meshcat_server_pid}" ] && kill -0 "${meshcat_server_pid}" 2>/dev/null; then
        kill -INT "${meshcat_server_pid}" 2>/dev/null || true
        for _ in $(seq 1 20); do
            if ! kill -0 "${meshcat_server_pid}" 2>/dev/null; then
                break
            fi
            sleep 0.1
        done
        if kill -0 "${meshcat_server_pid}" 2>/dev/null; then
            kill -TERM "${meshcat_server_pid}" 2>/dev/null || true
        fi
    fi

    rm -f "${DEMO_SHELL_PID_FILE}" "${DEMO_ROSLAUNCH_PID_FILE}" "${MESHCAT_SHELL_PID_FILE}" "${MESHCAT_SERVER_PID_FILE}"
}

trap cleanup_demo_terminal EXIT

gnome-terminal --title="demo" -- bash -lc '
set -e

source /opt/ros/noetic/setup.bash

if [ -f "'"${WORKSPACE_SETUP}"'" ]; then
    source "'"${WORKSPACE_SETUP}"'"
fi

DEMO_SHELL_PID_FILE="'"${DEMO_SHELL_PID_FILE}"'"
DEMO_ROSLAUNCH_PID_FILE="'"${DEMO_ROSLAUNCH_PID_FILE}"'"

cleanup_demo_roslaunch() {
    if [ -n "${DEMO_ROSLAUNCH_PID:-}" ] && kill -0 "${DEMO_ROSLAUNCH_PID}" 2>/dev/null; then
        kill -INT "${DEMO_ROSLAUNCH_PID}" 2>/dev/null || true
        wait "${DEMO_ROSLAUNCH_PID}" 2>/dev/null || true
    fi
    rm -f "${DEMO_SHELL_PID_FILE}" "${DEMO_ROSLAUNCH_PID_FILE}"
}

trap cleanup_demo_roslaunch EXIT INT TERM

echo "$$" > "${DEMO_SHELL_PID_FILE}"
roslaunch piper_with_gripper_moveit demo.launch &
DEMO_ROSLAUNCH_PID=$!
echo "${DEMO_ROSLAUNCH_PID}" > "${DEMO_ROSLAUNCH_PID_FILE}"
wait "${DEMO_ROSLAUNCH_PID}"
'

gnome-terminal --title="meshcat-server" -- bash -lc '
set -e

MESHCAT_SHELL_PID_FILE="'"${MESHCAT_SHELL_PID_FILE}"'"
MESHCAT_SERVER_PID_FILE="'"${MESHCAT_SERVER_PID_FILE}"'"

cleanup_meshcat_server() {
    if [ -n "${MESHCAT_SERVER_PID:-}" ] && kill -0 "${MESHCAT_SERVER_PID}" 2>/dev/null; then
        kill -INT "${MESHCAT_SERVER_PID}" 2>/dev/null || true
        wait "${MESHCAT_SERVER_PID}" 2>/dev/null || true
    fi
    rm -f "${MESHCAT_SHELL_PID_FILE}" "${MESHCAT_SERVER_PID_FILE}"
}

trap cleanup_meshcat_server EXIT INT TERM

echo "$$" > "${MESHCAT_SHELL_PID_FILE}"
meshcat-server &
MESHCAT_SERVER_PID=$!
echo "${MESHCAT_SERVER_PID}" > "${MESHCAT_SERVER_PID_FILE}"
wait "${MESHCAT_SERVER_PID}"
'

sleep 3

roslaunch seu_restaurant_smach init.launch launch_piper_demo:=false
