# Starts the three RealSense cameras with the realsense2_camera (ROS 2) driver.
# Topics are published as /<camera_name>/color/image_raw and /<camera_name>/depth/image_rect_raw.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# (camera name, serial number launch argument)
CAMERAS = [
    ("camera_l", "serial_no_camera1"),
    ("camera_r", "serial_no_camera2"),
    ("camera_f", "serial_no_camera3"),
]
# The front camera is only started when its serial number is set, as in the ROS 1 launch file.
OPTIONAL_CAMERAS = {"camera_f"}


def _launch_cameras(context):
    rs_launch = os.path.join(get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py")
    actions = []
    for camera_name, serial_arg in CAMERAS:
        serial_no = LaunchConfiguration(serial_arg).perform(context).strip("'\"")
        if not serial_no and camera_name in OPTIONAL_CAMERAS:
            continue
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rs_launch),
                launch_arguments={
                    "camera_name": camera_name,
                    "camera_namespace": "",
                    # Quote the serial so it is kept as a string parameter
                    "serial_no": f"'{serial_no}'",
                    "initial_reset": LaunchConfiguration("initial_reset"),
                    "rgb_camera.color_profile": LaunchConfiguration("color_profile"),
                    "depth_module.depth_profile": LaunchConfiguration("depth_profile"),
                    "enable_color": "true",
                    "enable_depth": "true",
                    "pointcloud.enable": "false",
                    "align_depth.enable": "false",
                }.items(),
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            # Note: Replace with the actual serial numbers (see piper_ws/scripts/rs_camera_serial.sh)
            DeclareLaunchArgument("serial_no_camera1", default_value="''"),
            DeclareLaunchArgument("serial_no_camera2", default_value="''"),
            DeclareLaunchArgument("serial_no_camera3", default_value="''"),
            DeclareLaunchArgument("initial_reset", default_value="false"),
            DeclareLaunchArgument("color_profile", default_value="640,480,60"),
            DeclareLaunchArgument("depth_profile", default_value="640,480,30"),
            OpaqueFunction(function=_launch_cameras),
        ]
    )
