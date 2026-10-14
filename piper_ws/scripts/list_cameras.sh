#!/bin/bash
# List cameras for piper_ws/src/piper/config/cameras_*.yaml:
#   - RealSense serial numbers (serial_no)
#   - USB (UVC/V4L2) camera device paths (video_device)

echo "RealSense cameras (serial_no):"
if command -v rs-enumerate-devices >/dev/null 2>&1; then
    rs-enumerate-devices -s
else
    echo "  rs-enumerate-devices not found. Source /opt/ros/\$ROS_DISTRO/setup.bash first."
fi

echo ""
echo "USB cameras (video_device):"
found=0
for dev in /dev/v4l/by-id/*-video-index0; do
    [[ -e "$dev" ]] || continue
    # RealSense cameras also expose V4L2 nodes; they are handled by realsense2_camera instead
    [[ "$dev" == *RealSense* ]] && continue
    echo "  $dev -> $(readlink -f "$dev")"
    found=1
done
[[ $found -eq 1 ]] || echo "  none found"
echo ""
echo "Supported sizes: v4l2-ctl -d <video_device> --list-formats-ext (package v4l-utils)"
