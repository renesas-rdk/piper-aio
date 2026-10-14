#!/usr/bin/env python3
# Leader-follower Piper node.
# mode=0 publishes both leader and follower arm state to ROS.
# mode=1 controls the follower arm from leader topics.
import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from piper_msgs.msg import PiperStatusMsg, PosCmd
from pyAgxArm import ArmModel, AgxArmFactory, PiperFW, create_agx_arm_config
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.exceptions import ROSInterruptException
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

# CAN-command speed rates (0-100%), matching the old piper_sdk ModeCtrl calls.
LEADER_PUBLISH_SPEED_PCT = 50  # end-effector pose control (pos_callback)
LEADER_JOINT_SPEED_PCT = 100  # joint control (joint_callback)

_FIRMWARE_VERSIONS = {
    "default": PiperFW.DEFAULT,
    "v183": PiperFW.V183,
    "v188": PiperFW.V188,
    "v189": PiperFW.V189,
}


def quaternion_from_euler(roll, pitch, yaw):
    """Return (x, y, z, w) for static-axis roll/pitch/yaw, matching tf.transformations' default 'sxyz'."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class C_PiperRosNode(Node):
    """Piper ROS node."""

    def __init__(self) -> None:
        super().__init__("piper_start_all_node")
        logger = self.get_logger()

        self.can_port = self.declare_parameter("can_port", "").value
        if not self.can_port:
            logger.info("can_port parameter not found, please use --ros-args -p can_port:=can0.")
            sys.exit(0)
        logger.info(f"can_port is {self.can_port}")
        self.mode = self.declare_parameter("mode", -1).value
        if self.mode not in (0, 1):
            logger.info("mode parameter not found, please use --ros-args -p mode:=0.")
            sys.exit(0)
        logger.info(f"mode is {self.mode}")

        self.auto_enable = bool(self.declare_parameter("auto_enable", False).value) and self.mode == 1
        logger.info(f"auto_enable is {self.auto_enable}")
        self.gripper_exist = True

        firmware_version = self.declare_parameter("firmware_version", "v189").value
        if firmware_version not in _FIRMWARE_VERSIONS:
            logger.info(
                f"Unknown firmware_version '{firmware_version}', expected one of "
                f"{list(_FIRMWARE_VERSIONS)}. Falling back to 'v189'."
            )
            firmware_version = "v189"
        logger.info(f"firmware_version is {firmware_version}")

        # publish
        self.joint_std_pub_follower = self.create_publisher(JointState, "/follower/joint_states", 1)
        if self.mode == 0:
            self.joint_std_pub_leader = self.create_publisher(JointState, "/leader/joint_states", 1)
        self.arm_status_pub = self.create_publisher(PiperStatusMsg, "/follower/arm_status", 1)
        self.end_pose_pub = self.create_publisher(PoseStamped, "/follower/end_pose", 1)
        self.end_pose_euler_pub = self.create_publisher(PosCmd, "/follower/end_pose_euler", 1)
        self.__enable_flag = False
        self.joint_state_follower = JointState()
        self.joint_state_follower.name = [
            "joint0",
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]
        self.joint_state_follower.position = [0.0] * 7
        self.joint_state_follower.velocity = [0.0] * 7
        self.joint_state_follower.effort = [0.0] * 7
        self.joint_state_leader = JointState()
        self.joint_state_leader.name = [
            "joint0",
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]
        self.joint_state_leader.position = [0.0] * 7
        self.joint_state_leader.velocity = [0.0] * 7
        self.joint_state_leader.effort = [0.0] * 7

        cfg = create_agx_arm_config(
            robot=ArmModel.PIPER,
            firmeware_version=_FIRMWARE_VERSIONS[firmware_version],
            interface="socketcan",
            channel=self.can_port,
        )
        self.piper = AgxArmFactory.create_arm(cfg)
        self.piper.connect()
        self.gripper = self.piper.init_effector(self.piper.OPTIONS.EFFECTOR.AGX_GRIPPER)
        # service
        str_can_port = str(self.can_port)
        # Leader arm home service.
        self.leader_go_zero_service = self.create_service(
            Trigger,
            "/" + str_can_port + "/go_zero_leader",
            self.handle_leader_go_zero_service,
        )
        # Leader and follower arm home service.
        self.leader_follower_go_zero_service = self.create_service(
            Trigger,
            "/" + str_can_port + "/go_zero_leader_follower",
            self.handle_leader_follower_go_zero_service,
        )
        # Restore leader-follower mode service.
        self.restore_leader_follower_mode_service = self.create_service(
            Trigger,
            "/" + str_can_port + "/restore_leader_follower_mode",
            self.handle_restore_leader_follower_mode_service,
        )
        if self.mode == 1:
            # Each subscription gets its own callback group so pose, joint and enable commands
            # are handled concurrently by the multi-threaded executor (like the ROS 1 threads).
            self.create_subscription(
                PosCmd, "/pos_cmd", self.pos_callback, 1, callback_group=MutuallyExclusiveCallbackGroup()
            )
            self.create_subscription(
                JointState,
                "/leader/joint_states",
                self.joint_callback,
                1,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
            self.create_subscription(
                Bool, "/enable_flag", self.enable_callback, 1, callback_group=MutuallyExclusiveCallbackGroup()
            )

    def GetEnableFlag(self):
        return self.__enable_flag

    def Pubilsh(self):
        """Publish arm messages."""
        rate = self.create_rate(200)  # 200 Hz
        enable_flag = False
        timeout = 5
        start_time = time.time()
        elapsed_time_flag = False
        while rclpy.ok():
            if self.auto_enable and self.mode == 1:
                while not (enable_flag):
                    elapsed_time = time.time() - start_time
                    print("--------------------")
                    enable_flag = all(self.piper.get_joints_enable_status_list())
                    print("Enable Status:", enable_flag)
                    if enable_flag:
                        self.__enable_flag = True
                    self.piper.enable()
                    self.gripper.reset_gripper()
                    self.gripper.move_gripper_m(0.0, force=1.0)
                    print("--------------------")
                    if elapsed_time > timeout:
                        print("timeout....")
                        elapsed_time_flag = True
                        enable_flag = True
                        break
                    time.sleep(1)
                    pass
            if elapsed_time_flag:
                print("auto enable timeout, exiting program")
                rclpy.try_shutdown()
                return
            self.PublishFollowerArmJointAndGripper()
            self.PublishFollowerArmState()
            self.PublishFollowerArmEndPose()
            if self.mode == 0:
                self.PublishLeaderArmJointAndGripper()

            rate.sleep()

    def PublishFollowerArmState(self):
        status = self.piper.get_arm_status()
        if status is None:
            return
        arm_status = PiperStatusMsg()
        arm_status.ctrl_mode = status.msg.ctrl_mode
        arm_status.arm_status = status.msg.arm_status
        arm_status.mode_feedback = status.msg.mode_feedback
        arm_status.teach_status = status.msg.teach_status
        arm_status.motion_status = status.msg.motion_status
        arm_status.trajectory_num = status.msg.trajectory_num
        # pyAgxArm has no single packed err_code; the per-joint flags below
        # carry the same information, so this field is left at its default.
        arm_status.joint_1_angle_limit = status.msg.err_status.joint_1_angle_limit
        arm_status.joint_2_angle_limit = status.msg.err_status.joint_2_angle_limit
        arm_status.joint_3_angle_limit = status.msg.err_status.joint_3_angle_limit
        arm_status.joint_4_angle_limit = status.msg.err_status.joint_4_angle_limit
        arm_status.joint_5_angle_limit = status.msg.err_status.joint_5_angle_limit
        arm_status.joint_6_angle_limit = status.msg.err_status.joint_6_angle_limit
        arm_status.communication_status_joint_1 = status.msg.err_status.communication_status_joint_1
        arm_status.communication_status_joint_2 = status.msg.err_status.communication_status_joint_2
        arm_status.communication_status_joint_3 = status.msg.err_status.communication_status_joint_3
        arm_status.communication_status_joint_4 = status.msg.err_status.communication_status_joint_4
        arm_status.communication_status_joint_5 = status.msg.err_status.communication_status_joint_5
        arm_status.communication_status_joint_6 = status.msg.err_status.communication_status_joint_6
        self.arm_status_pub.publish(arm_status)

    def PublishFollowerArmEndPose(self):
        flange_pose = self.piper.get_flange_pose()
        gripper_status = self.gripper.get_gripper_status()
        if flange_pose is None or gripper_status is None:
            return

        x, y, z, roll, pitch, yaw = flange_pose.msg

        endpos = PoseStamped()
        endpos.pose.position.x = x
        endpos.pose.position.y = y
        endpos.pose.position.z = z
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        endpos.pose.orientation.x = quaternion[0]
        endpos.pose.orientation.y = quaternion[1]
        endpos.pose.orientation.z = quaternion[2]
        endpos.pose.orientation.w = quaternion[3]
        endpos.header.stamp = self.get_clock().now().to_msg()
        self.end_pose_pub.publish(endpos)

        end_pose_euler = PosCmd()
        end_pose_euler.x = x
        end_pose_euler.y = y
        end_pose_euler.z = z
        end_pose_euler.roll = roll
        end_pose_euler.pitch = pitch
        end_pose_euler.yaw = yaw
        end_pose_euler.gripper = gripper_status.msg.value
        end_pose_euler.mode1 = 0
        end_pose_euler.mode2 = 0
        self.end_pose_euler_pub.publish(end_pose_euler)

    def PublishFollowerArmJointAndGripper(self):
        joint_angles = self.piper.get_joint_angles()
        gripper_status = self.gripper.get_gripper_status()
        if joint_angles is None or gripper_status is None:
            return

        self.joint_state_follower.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_follower.position = [*joint_angles.msg, gripper_status.msg.value]

        velocities = []
        for joint_index in range(1, 7):
            motor_state = self.piper.get_motor_states(joint_index)
            velocities.append(motor_state.msg.velocity if motor_state is not None else 0.0)
        self.joint_state_follower.velocity = [*velocities, 0.0]

        self.joint_state_follower.effort[6] = gripper_status.msg.force
        self.joint_std_pub_follower.publish(self.joint_state_follower)

    def PublishLeaderArmJointAndGripper(self):
        leader_angles = self.piper.get_leader_joint_angles()
        gripper_ctrl = self.gripper.get_gripper_ctrl_states()
        if leader_angles is None or gripper_ctrl is None:
            return

        self.joint_state_leader.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_leader.position = [*leader_angles.msg, gripper_ctrl.msg.value]
        self.joint_std_pub_leader.publish(self.joint_state_leader)

    def pos_callback(self, pos_data):
        """Handle end-effector pose commands."""
        self.get_logger().info("Received PosCmd:")
        self.get_logger().info(f"x: {pos_data.x}")
        self.get_logger().info(f"y: {pos_data.y}")
        self.get_logger().info(f"z: {pos_data.z}")
        self.get_logger().info(f"roll: {pos_data.roll}")
        self.get_logger().info(f"pitch: {pos_data.pitch}")
        self.get_logger().info(f"yaw: {pos_data.yaw}")
        self.get_logger().info(f"gripper: {pos_data.gripper}")
        self.get_logger().info(f"mode1: {pos_data.mode1}")
        self.get_logger().info(f"mode2: {pos_data.mode2}")
        if self.GetEnableFlag():
            self.piper.set_speed_percent(LEADER_PUBLISH_SPEED_PCT)
            self.piper.move_p([pos_data.x, pos_data.y, pos_data.z, pos_data.roll, pos_data.pitch, pos_data.yaw])
            if self.gripper_exist:
                gripper = max(0.0, min(0.08, pos_data.gripper))
                self.gripper.move_gripper_m(abs(gripper), force=1.0)

    def joint_callback(self, joint_data):
        """Handle joint commands."""
        self.get_logger().info("Received Joint States:")
        self.get_logger().info(f"joint_0: {joint_data.position[0]}")
        self.get_logger().info(f"joint_1: {joint_data.position[1]}")
        self.get_logger().info(f"joint_2: {joint_data.position[2]}")
        self.get_logger().info(f"joint_3: {joint_data.position[3]}")
        self.get_logger().info(f"joint_4: {joint_data.position[4]}")
        self.get_logger().info(f"joint_5: {joint_data.position[5]}")
        self.get_logger().info(f"joint_6: {joint_data.position[6]}")
        if self.GetEnableFlag():
            self.piper.set_speed_percent(LEADER_JOINT_SPEED_PCT)
            self.piper.move_j(list(joint_data.position[:6]))
            gripper = max(0.0, min(0.08, joint_data.position[6]))
            self.gripper.move_gripper_m(abs(gripper), force=1.0)

    def enable_callback(self, enable_flag: Bool):
        """Handle enable commands."""
        self.get_logger().info("Received enable flag:")
        self.get_logger().info(f"enable_flag: {enable_flag.data}")
        if enable_flag.data:
            self.__enable_flag = True
            self.piper.enable()
            self.gripper.reset_gripper()
            self.gripper.move_gripper_m(0.0, force=1.0)
        else:
            self.__enable_flag = False
            self.piper.disable()
            self.gripper.disable_gripper()

    def handle_leader_go_zero_service(self, req, response):
        self.get_logger().info(f"-----------------------RESET---------------------------")
        self.get_logger().info(f"{self.can_port} send piper leader go zero service")
        self.get_logger().info(f"-----------------------RESET---------------------------")
        self.piper.move_leader_to_home()
        response.success = True
        response.message = str({self.can_port}) + "send piper leader go zero service success"
        self.get_logger().info(f"Returning resetResponse: {response.success}, {response.message}")
        return response

    def handle_leader_follower_go_zero_service(self, req, response):
        self.get_logger().info(f"-----------------------RESET---------------------------")
        self.get_logger().info(f"{self.can_port} send piper leader follower go zero service")
        self.get_logger().info(f"-----------------------RESET---------------------------")
        self.piper.move_leader_follower_to_home()
        response.success = True
        response.message = str({self.can_port}) + "send piper leader follower go zero service success"
        self.get_logger().info(f"Returning resetResponse: {response.success}, {response.message}")
        return response

    def handle_restore_leader_follower_mode_service(self, req, response):
        self.get_logger().info(f"-----------------------RESET---------------------------")
        self.get_logger().info(f"{self.can_port} send piper restore leader follower mode service")
        self.get_logger().info(f"-----------------------RESET---------------------------")
        self.piper.restore_leader_drag_mode()
        response.success = True
        response.message = str({self.can_port}) + "send piper restore leader follower mode service success"
        self.get_logger().info(f"Returning resetResponse: {response.success}, {response.message}")
        return response


def main():
    rclpy.init()
    piper_leader_follower = C_PiperRosNode()
    executor = MultiThreadedExecutor()
    executor.add_node(piper_leader_follower)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        piper_leader_follower.Pubilsh()
    except (KeyboardInterrupt, ExternalShutdownException, ROSInterruptException):
        pass
    finally:
        executor.shutdown()
        piper_leader_follower.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
