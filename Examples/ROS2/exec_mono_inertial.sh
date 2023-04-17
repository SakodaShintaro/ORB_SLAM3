#!/bin/bash

set -eux

# change directory to the location of this script
DIR_NAME=$(readlink -f $(dirname $0))
cd $DIR_NAME

# install system
set +u
source ./install/setup.bash
set -u

# setting up environment variables
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:../../lib:../../../Pangolin/build/"

# run ORB_SLAM3_ROS2
ros2 run ORB_SLAM3_ROS2 MonoInertial \
    ../../Vocabulary/ORBvoc.txt \
    ./config_mono_inertial.yaml
