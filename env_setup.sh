#!/bin/bash
#export HUSKY_UST10_ENABLED=true
export HUSKY_LMS1XX_ENABLED=true
#export HUSKY_REALSENSE_ENABLED=true
#export HUSKY_URDF_EXTRAS="$(find husky_description)/urdf/cameras.urdf.xacro"
export HUSKY_URDF_EXTRAS="cameras.urdf.xacro"
source ../cartographer_ws/devel_isolated/setup.bash
source devel/setup.bash
