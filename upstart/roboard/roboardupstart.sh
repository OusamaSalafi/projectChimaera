#!/bin/bash

source /opt/ros/fuerte/setup.bash

export ROS_PACKAGE_PATH=/home/uwesub/projectChimaera:$ROS_PACKAGE_PATH
export ROS_WORKSPACE=/home/uwesub/projectChimaera

roslaunch /home/uwesub/projectChimaera/Launch/roboard.launch
