#!/usr/bin/env bash
# Usage: ./start_cam.sh [dual|single]
ROBOT="${1:-dual}"

if [[ "$ROBOT" == "single" ]]; then
  # Side and wrist cameras (RealSense or USB), configured in piper_ws/src/piper/config/cameras_single.yaml
  ros2 launch piper cameras.launch.py
else
  # Set the serial numbers of the left wrist, right wrist and front cameras (see piper_ws/scripts/rs_camera_serial.sh)
  ros2 launch piper multi_camera.launch.py \
    serial_no_camera1:="''" \
    serial_no_camera2:="''" \
    serial_no_camera3:="''"
fi
