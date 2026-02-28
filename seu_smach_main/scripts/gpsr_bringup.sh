#!/bin/bash

gnome-terminal -- bash -c "roslaunch seu_smach_main init.launch; exec bash"

echo “init.launch  successfully started”

# 两个roslauch之间需要间隔一段时间，否则会相互抢占roscore,导致其中一个roslaunch失败,报runid错误

sleep 5s  

gnome-terminal -- bash -c "export PYTHONPATH="${PYTHONPATH}:/usr/lib/python3/dist-packages/";roslaunch seu_smach_main export.launch; exec bash"

echo “export.launch  successfully started”
sleep 5s  

gnome-terminal -- bash -c "export PYTHONPATH="${PYTHONPATH}:/usr/lib/python3/dist-packages/";roslaunch seu_smach_main other.launch; exec bash"

echo “other.launch  successfully started”
