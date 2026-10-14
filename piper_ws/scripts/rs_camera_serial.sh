#!/bin/bash
# List connected RealSense camera serial numbers.
# rs-enumerate-devices ships with ros-$ROS_DISTRO-librealsense2 (a dependency of realsense2_camera).

if ! command -v rs-enumerate-devices >/dev/null 2>&1; then
    echo "rs-enumerate-devices not found. Source /opt/ros/\$ROS_DISTRO/setup.bash or install librealsense2-utils."
    exit 1
fi

echo "camera serial number: "
rs-enumerate-devices -s

echo "-----------------"
echo ""
