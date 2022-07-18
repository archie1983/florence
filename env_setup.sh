#!/bin/bash
#export HUSKY_LMS1XX_ENABLED=true
#export HUSKY_REALSENSE_ENABLED=true
#export HUSKY_URDF_EXTRAS="$(find husky_description)/urdf/cameras.urdf.xacro"
#export RPLIDAR=false
#export USB_CAM=true
#export SIM_CAM=true
export HUSKY_URDF_EXTRAS="cameras.urdf.xacro"
export ROS_TCP_ENDPOINT_IPADDR="192.168.8.199"
export ROS_TCP_ENDPOINT_PORT="10000"
export HUSKY_UST10_ENABLED=true
export HUSKY_UST10_IP="192.168.131.20"
export HUSKY_UST10_SECONDARY_IP="192.168.131.21"
export HUSKY_UST10_ENABLED=true
export HUSKY_UST10_SECONDARY_ENABLED=true
export HUSKY_FRONT_BUMPER_EXTEND=false
export HUSKY_REAR_BUMPER_EXTEND=false
export HUSKY_UST10_XYZ="0.3812 0.0 -0.35635"
export HUSKY_UST10_SECONDARY_XYZ="-0.4712 0.0 -0.35635"
source /home/jet/robotics/cartographer_ws/devel_isolated/setup.bash
source /home/jet/robotics/florence_ws/devel/setup.bash
