#!/bin/bash

if (( $EUID > 0 )); then
	echo " - Please run as root"
	exit
fi

echo " - Install Camera Calibration"
apt install ros-humble-camera-calibration ros-humble-camera-calibration-parsers ros-humble-camera-info-manager ros-humble-launch-testing-ament-cmake
