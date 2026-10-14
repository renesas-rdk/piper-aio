# coding=utf-8
import argparse
import os
import sys
import time

import h5py
import numpy as np
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from piper_msgs.msg import PosCmd

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from robot_config import add_robot_argument, get_robot_config
from ros_operator import Rate


def load_hdf5(dataset_dir, dataset_name):
    dataset_path = os.path.join(dataset_dir, dataset_name + ".hdf5")
    if not os.path.isfile(dataset_path):
        raise FileNotFoundError(f"Dataset does not exist at {dataset_path}")

    with h5py.File(dataset_path, "r") as root:
        qpos = root["/observations/qpos"][()]
        action = root["/action"][()]
        eef_pose = root["/observations/eef_pose"][()]

    return qpos, action, eef_pose


def publish_joint_replay(joint_state_msg, timestamp, arm_actions, publishers):
    joint_state_msg.header.stamp = timestamp
    for arm_action, publisher in zip(arm_actions, publishers):
        # ROS 2 message fields only accept Python floats (not np.float32)
        joint_state_msg.position = [float(v) for v in arm_action]
        publisher.publish(joint_state_msg)


def publish_eef_replay(arm_poses, publishers):
    for arm_pose, publisher in zip(arm_poses, publishers):
        arm_pose = [float(v) for v in arm_pose]
        msg = PosCmd()
        msg.x = arm_pose[0]
        msg.y = arm_pose[1]
        msg.z = arm_pose[2]
        msg.roll = arm_pose[3]
        msg.pitch = arm_pose[4]
        msg.yaw = arm_pose[5]
        msg.gripper = arm_pose[6]
        msg.mode1 = 0
        msg.mode2 = 0
        publisher.publish(msg)


def wait_for_subscribers(publishers, timeout=3.0):
    """Give DDS discovery time to match the arm nodes so the first commands are not dropped."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and rclpy.ok():
        if all(publisher.get_subscription_count() > 0 for publisher in publishers):
            # Discovery is not always symmetric yet when the publisher first sees the subscriber
            time.sleep(0.5)
            return True
        time.sleep(0.05)
    print("Warning: no subscriber on some control topics; is the arm node running in mode 1?")
    return False


def main(args):
    rclpy.init()
    node = rclpy.create_node("replay_node")
    try:
        replay(args, node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def replay(args, node):
    robot = get_robot_config(args.robot)
    # In mode 1 the arm node follows the leader topic, so joint commands are published there
    joint_publishers = [node.create_publisher(JointState, arm.leader_topic, 10) for arm in robot.arms]
    eef_publishers = [node.create_publisher(PosCmd, arm.pos_cmd_topic, 10) for arm in robot.arms]

    dataset_dir = os.path.join(args.dataset_dir, args.task_name)
    dataset_name = f"episode_{args.episode_idx}"
    qposs, actions, eef_poses = load_hdf5(dataset_dir, dataset_name)
    if actions.shape[1] != robot.state_dim:
        raise ValueError(
            f"Episode has {actions.shape[1]}-dim actions but robot '{robot.name}' expects {robot.state_dim}; "
            "check --robot"
        )

    joint_state_msg = JointState()
    joint_state_msg.header = Header()
    joint_state_msg.name = ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

    if args.replay_mode == "joint":
        wait_for_subscribers(joint_publishers)
    else:
        wait_for_subscribers(eef_publishers)

    rate = Rate(args.frame_rate)

    for index in range(len(actions)):
        if not rclpy.ok():
            break

        if args.replay_mode == "joint":
            arm_actions = robot.split_arms(actions[index])
            publish_joint_replay(joint_state_msg, node.get_clock().now().to_msg(), arm_actions, joint_publishers)
            print(
                "joint replay",
                *[f"{name}_action: {np.round(a, 4)}" for name, a in zip(robot.arm_names, arm_actions)],
            )
        else:
            arm_poses = robot.split_arms(eef_poses[index])
            publish_eef_replay(arm_poses, eef_publishers)
            print(
                "eef replay",
                *[f"{name}_eef: {np.round(p, 4)}" for name, p in zip(robot.arm_names, arm_poses)],
            )

        rate.sleep()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dataset_dir",
        action="store",
        type=str,
        help="Dataset dir.",
        default="",
        required=False,
    )
    parser.add_argument(
        "--task_name",
        action="store",
        type=str,
        help="Task name.",
        default="test",
        required=False,
    )
    parser.add_argument(
        "--episode_idx",
        action="store",
        type=int,
        help="Episode index.",
        default=0,
        required=False,
    )
    parser.add_argument(
        "--replay_mode",
        action="store",
        type=str,
        choices=["joint", "eef"],
        default="joint",
        help="Replay using joint commands or end-effector pose commands.",
        required=False,
    )
    add_robot_argument(parser)
    parser.add_argument(
        "--frame_rate",
        action="store",
        type=int,
        help="frame_rate",
        default=30,
        required=False,
    )

    main(parser.parse_args())
