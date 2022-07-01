#!/bin/bash
#export HUSKY_UST10_ENABLED=true
export HUSKY_LMS1XX_ENABLED=true
#export HUSKY_REALSENSE_ENABLED=true
#export HUSKY_URDF_EXTRAS="$(find husky_description)/urdf/cameras.urdf.xacro"
export HUSKY_URDF_EXTRAS="cameras.urdf.xacro"
export RPLIDAR=false
#export USB_CAM=true
export ROS_TCP_ENDPOINT_IPADDR="192.168.8.199"
export ROS_TCP_ENDPOINT_PORT="10000"
source ../cartographer_ws/devel_isolated/setup.bash
source devel/setup.bash
