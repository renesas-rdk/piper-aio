from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _arm_node(name, can_port, suffix):
    """One leader-follower pair on one CAN bus; topics get `suffix` (must match robot_configs.yaml)."""
    return Node(
        package="piper",
        executable="piper_start_leader_follower_node.py",
        name=name,
        output="screen",
        parameters=[
            {
                "can_port": can_port,
                "mode": ParameterValue(LaunchConfiguration("mode"), value_type=int),
                "auto_enable": ParameterValue(LaunchConfiguration("auto_enable"), value_type=bool),
                "firmware_version": LaunchConfiguration("firmware_version"),
            }
        ],
        remappings=[
            ("/follower/arm_status", f"/follower/arm_status{suffix}"),
            ("/follower/joint_states", f"/follower/joint{suffix}"),
            ("/leader/joint_states", f"/leader/joint{suffix}"),
            ("/follower/end_pose", f"/follower/end_pose{suffix}"),
            ("/follower/end_pose_euler", f"/follower/end_pose_euler{suffix}"),
            ("/pos_cmd", f"/follower/pos_cmd{suffix}"),
        ],
    )


def _launch_arms(context):
    robot = LaunchConfiguration("robot").perform(context)
    if robot == "dual":
        return [
            _arm_node("piper_left", "can_left", "_left"),
            _arm_node("piper_right", "can_right", "_right"),
        ]
    if robot == "single":
        return [_arm_node("piper", LaunchConfiguration("can_port").perform(context), "")]
    raise RuntimeError(f"Unknown robot '{robot}', expected 'dual' or 'single'")


def generate_launch_description():
    return LaunchDescription(
        [
            # mode: 0 = read leader/follower and forward to ROS; 1 = control follower from leader topics
            DeclareLaunchArgument("mode", default_value="0"),
            DeclareLaunchArgument("auto_enable", default_value="true"),
            # dual: two leader-follower pairs on can_left/can_right; single: one pair on can_port
            DeclareLaunchArgument("robot", default_value="dual", choices=["dual", "single"]),
            DeclareLaunchArgument("can_port", default_value="can0", description="CAN interface for robot:=single"),
            # pyAgxArm firmware profile for the Piper main controller: default (<= S-V1.8-2),
            # v183 (S-V1.8-3 ~ S-V1.8-7), v188 (S-V1.8-8), v189 (>= S-V1.8-9).
            DeclareLaunchArgument("firmware_version", default_value="v189"),
            OpaqueFunction(function=_launch_arms),
        ]
    )
