#!/bin/bash
# 文件名：start_pycharm_with_ros2.sh

# 完整的 ROS2 环境设置
source /opt/ros/humble/setup.bash

# 导出所有必要的环境变量
export LD_LIBRARY_PATH=/opt/ros/humble/lib:/opt/ros/humble/local/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:$PYTHONPATH

# 启动 PyCharm
pycharm-professional &
