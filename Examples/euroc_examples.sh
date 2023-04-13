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

#------------------------------------
# Stereo Examples
echo "Launching MH01 with Stereo sensor"
./Stereo/stereo_euroc ../Vocabulary/ORBvoc.txt ./Stereo/EuRoC.yaml "$pathDatasetEuroc"/MH01 ./Stereo/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereo

#------------------------------------
# Monocular-Inertial Examples
echo "Launching MH01 with Monocular-Inertial sensor"
./Monocular-Inertial/mono_inertial_euroc ../Vocabulary/ORBvoc.txt ./Monocular-Inertial/EuRoC.yaml "$pathDatasetEuroc"/MH01 ./Monocular-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_monoi

#------------------------------------
# Stereo-Inertial Examples
echo "Launching MH01 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_euroc ../Vocabulary/ORBvoc.txt ./Stereo-Inertial/EuRoC.yaml "$pathDatasetEuroc"/MH01 ./Stereo-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereoi
