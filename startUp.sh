#!/bin/bash

# while ! xset q; do
#     sleep 1
# done

source /opt/ros/humble/setup.bash
source /ssd/repos/seniorDesign/DVEPS-Lite/processor/install/setup.bash

nohup MicroXRCEAgent udp4 -p 8888 > /tmp/microxrceagent.log 2>&1 &
#MicroXRCEAgent udp4 -p 8888 &
AGENT_PID=$!

ros2 launch system_launch bring_up_system_launch.py
