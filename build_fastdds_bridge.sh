#!/bin/bash
source /opt/ros/humble/setup.bash
cd /workspace
colcon build --packages-select fastdds_ros2_bridge --symlink-install \
  --cmake-args '-DCMAKE_PREFIX_PATH=/workspace/src/env/fast_dds/local;/opt/ros/humble'