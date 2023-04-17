#!/bin/bash

set -eux

# set the path to the EuRoC dataset
pathDatasetEuroc=$1

# change directory to the location of this script
DIR_NAME=$(readlink -f $(dirname $0))
cd $DIR_NAME

#------------------------------------
# Monocular Examples
echo "Launching MH01 with Monocular sensor"
./Monocular/mono_euroc ../Vocabulary/ORBvoc.txt ./Monocular/EuRoC.yaml "$pathDatasetEuroc"/MH01 ./Monocular/EuRoC_TimeStamps/MH01.txt dataset-MH01_mono
