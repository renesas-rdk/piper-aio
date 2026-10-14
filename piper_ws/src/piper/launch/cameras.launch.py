# Starts the cameras listed in a YAML file (default: config/cameras_single.yaml).
# RealSense cameras use the realsense2_camera driver; USB (UVC/V4L2) cameras use v4l2_camera.
# Every camera publishes /<name>/color/image_raw (RealSense also /<name>/depth/image_rect_raw when enabled),
# so the rest of the stack does not depend on the camera type.
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _realsense_camera(camera):
    rs_launch = os.path.join(get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py")
    profile = f"{camera.get('width', 640)},{camera.get('height', 480)},{camera.get('fps', 30)}"
    enable_depth = str(bool(camera.get("enable_depth", False))).lower()
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch),
        launch_arguments={
            "camera_name": camera["name"],
            "camera_namespace": "",
            # Quote the serial so it is kept as a string parameter
            "serial_no": f"'{str(camera.get('serial_no') or '').strip(chr(39))}'",
            "initial_reset": str(bool(camera.get("initial_reset", False))).lower(),
            "rgb_camera.color_profile": profile,
            "depth_module.depth_profile": profile,
            "enable_color": "true",
            "enable_depth": enable_depth,
            "pointcloud.enable": "false",
            "align_depth.enable": "false",
        }.items(),
    )


def _usb_camera(camera):
    if not camera.get("video_device"):
        raise RuntimeError(f"USB camera '{camera['name']}' needs a video_device")
    return Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name=camera["name"],
        namespace=camera["name"],
        output="screen",
        parameters=[
            {
                "video_device": camera["video_device"],
                "image_size": [int(camera.get("width", 640)), int(camera.get("height", 480))],
                "pixel_format": camera.get("pixel_format", "YUYV"),
                "output_encoding": "rgb8",
                "camera_frame_id": f"{camera['name']}_color_optical_frame",
                # Only publish raw images; compressed transports cost CPU and are unused
                "image_raw.enable_pub_plugins": ["image_transport/raw"],
            }
        ],
        # Match the RealSense topic layout
        remappings=[("image_raw", "color/image_raw"), ("camera_info", "color/camera_info")],
    )


def _launch_cameras(context):
    config_path = LaunchConfiguration("config").perform(context)
    with open(config_path, "r", encoding="utf-8") as file:
        cameras = yaml.safe_load(file)["cameras"]

    actions = []
    for camera in cameras:
        camera_type = camera.get("type")
        if camera_type == "realsense":
            actions.append(_realsense_camera(camera))
        elif camera_type == "usb":
            actions.append(_usb_camera(camera))
        else:
            raise RuntimeError(f"Camera '{camera.get('name')}' has unknown type '{camera_type}' (realsense or usb)")
    return actions


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory("piper"), "config", "cameras_single.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config, description="Camera YAML file"),
            OpaqueFunction(function=_launch_cameras),
        ]
    )
