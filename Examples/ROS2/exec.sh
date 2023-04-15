#!/bin/bash

set -eux

# change directory to the location of this script
DIR_NAME=$(readlink -f $(dirname $0))
cd $DIR_NAME

# setting up environment variables
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:../../ORB_SLAM3/lib:../../../Pangolin/build/"

# run ORB_SLAM3_ROS2
ros2 run ORB_SLAM3_ROS2 Mono \
    ../../Vocabulary/ORBvoc.txt \
    ./config.yaml
