#!/bin/bash

set -eux

# set the path to the EuRoC dataset
pathDatasetEuroc=$1

# change directory to the location of this script
DIR_NAME=$(readlink -f $(dirname $0))
cd $DIR_NAME

#------------------------------------
# Monocular-Inertial Examples
echo "Launching MH01 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_euroc ../Vocabulary/ORBvoc.txt ./Monocular-Inertial/EuRoC.yaml "$pathDatasetEuroc"/MH01 ./Monocular-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_monoi
