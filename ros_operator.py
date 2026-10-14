from collections import deque
import atexit
import threading
import time

import cv2
import dm_env
import numpy as np
import rclpy
from piper_msgs.msg import PosCmd
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Header

from robot_config import get_robot_config

_logger = rclpy.logging.get_logger("piper_aio")

# Best-effort subscribers connect to both reliable and best-effort publishers (e.g. realsense2_camera).
SUBSCRIBER_QOS = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)

_IMAGE_ENCODINGS = {
    "rgb8": (np.uint8, 3),
    "bgr8": (np.uint8, 3),
    "rgba8": (np.uint8, 4),
    "bgra8": (np.uint8, 4),
    "mono8": (np.uint8, 1),
    "8UC1": (np.uint8, 1),
    "8UC3": (np.uint8, 3),
    "mono16": (np.uint16, 1),
    "16UC1": (np.uint16, 1),
    "32FC1": (np.float32, 1),
}


def is_shutdown():
    return not rclpy.ok()


def signal_shutdown(reason=""):
    if reason:
        _logger.info(f"Shutdown requested: {reason}")
    rclpy.try_shutdown()


def logwarn(msg):
    _logger.warning(msg)


class Rate:
    """Wall-clock rate limiter that sleeps like ROS 1's Rate.sleep()."""

    def __init__(self, hz):
        self.period = 1.0 / hz
        self.last_time = time.monotonic()

    def sleep(self):
        now = time.monotonic()
        # Reset if time jumped backwards or we fell more than one period behind
        if now < self.last_time or now - self.last_time > self.period:
            self.last_time = now
        remaining = self.last_time + self.period - now
        if remaining > 0:
            time.sleep(remaining)
        self.last_time += self.period


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def imgmsg_to_numpy(msg):
    """Convert a sensor_msgs/Image to a numpy array without changing the encoding (cv_bridge "passthrough")."""
    if msg.encoding not in _IMAGE_ENCODINGS:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")
    dtype, channels = _IMAGE_ENCODINGS[msg.encoding]
    dtype = np.dtype(dtype).newbyteorder(">" if msg.is_bigendian else "<")
    row_items = msg.step // dtype.itemsize
    image = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, row_items)
    image = image[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
    if channels == 1:
        image = image[:, :, 0]
    return np.ascontiguousarray(image.astype(dtype.newbyteorder("=")))


def _to_float_list(values):
    return [float(v) for v in values]
class RosOperator:
    queue_size_limit = 2000
    # Padding (top, bottom, left, right) applied to depth images during collection
    depth_border_padding = (40, 40, 0, 0)

    def __init__(self, args, mode):
        self.args = args
        if mode not in {"collection", "inference"}:
            raise ValueError(f"Unsupported RosOperator mode: {mode}")
        self.mode = mode
        self.robot = get_robot_config(args.robot)
        if args.use_depth_image:
            missing_depth = [camera.name for camera in self.robot.cameras if not camera.depth_topic]
            if missing_depth:
                raise ValueError(f"--use_depth_image requires a depth_topic for cameras: {missing_depth}")
        self.node = None
        self.executor = None
        self.spin_thread = None

        self.joint_publishers = {}
        self.pose_publishers = {}
        self.follower_arm_publish_thread = None
        self.follower_arm_publish_lock = threading.Lock()
        self.follower_arm_publish_lock.acquire()

        self._init_queues()
        self.init_ros()

    def _init_queues(self):
        camera_names = self.robot.camera_names
        arm_names = self.robot.arm_names
        self.image_queues = {name: deque() for name in camera_names}
        self.depth_queues = {name: deque() for name in camera_names} if self.args.use_depth_image else {}
        self.follower_queues = {name: deque() for name in arm_names}
        self.follower_pose_queues = {name: deque() for name in arm_names}
        self.leader_queues = {name: deque() for name in arm_names} if self.mode == "collection" else {}

    def init(self):
        self.reset()

    def reset(self):
        # Swap in fresh deques; callbacks look queues up by name so they write into the new ones
        self._init_queues()

    def _append_to_queue(self, queue, msg):
        if len(queue) >= self.queue_size_limit:
            queue.popleft()
        queue.append(msg)

    def _make_queue_callback(self, group, name, stamp_now=False):
        def _callback(msg):
            queue = getattr(self, group)[name]
            data = (self.now_sec(), msg) if stamp_now else msg
            self._append_to_queue(queue, data)

        return _callback

    def _register_subscribers(self, subscriber_specs):
        for topic, msg_type, callback in subscriber_specs:
            self.node.create_subscription(msg_type, topic, callback, SUBSCRIBER_QOS)

    def now_sec(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _get_msg_stamp(self, msg):
        if isinstance(msg, tuple):
            return msg[0]
        return stamp_to_sec(msg.header.stamp)

    def _format_sync_failure(self, stream_map, frame_time=None):
        missing_streams = []
        stale_streams = []
        stream_states = []
        for name, queue in stream_map.items():
            if len(queue) == 0:
                missing_streams.append(name)
                stream_states.append(f"{name}:len=0")
                continue
            latest_stamp = self._get_msg_stamp(queue[-1])
            if frame_time is not None and latest_stamp < frame_time:
                stale_streams.append(f"{name}={latest_stamp:.6f}")
            if frame_time is None:
                stream_states.append(f"{name}:len={len(queue)},latest={latest_stamp:.6f}")
            else:
                stream_states.append(
                    f"{name}:len={len(queue)},latest={latest_stamp:.6f},delta={latest_stamp - frame_time:+.4f}s"
                )

        details = []
        if frame_time is not None:
            details.append(f"frame_time={frame_time:.6f}")
        if missing_streams:
            details.append("missing=" + ",".join(missing_streams))
        if stale_streams:
            details.append("stale=" + ",".join(stale_streams))
        if stream_states:
            details.append("streams=[" + "; ".join(stream_states) + "]")
        return " | ".join(details) if details else "unknown sync state"

    def _resolve_frame_time(self, camera_queues):
        return min(self._get_msg_stamp(queue[-1]) for queue in camera_queues)

    def _validate_stream_sync(self, stream_map, frame_time):
        for queue in stream_map.values():
            if len(queue) == 0 or self._get_msg_stamp(queue[-1]) < frame_time:
                return False
        return True

    def _pop_synced_message(self, queue, frame_time):
        while self._get_msg_stamp(queue[0]) < frame_time:
            queue.popleft()
        return queue.popleft()

    def _pop_synced_image(self, queue, frame_time, border_padding=None):
        image = imgmsg_to_numpy(self._pop_synced_message(queue, frame_time))
        if border_padding is not None:
            top, bottom, left, right = border_padding
            image = cv2.copyMakeBorder(image, top, bottom, left, right, cv2.BORDER_CONSTANT, value=0)
        return image

    def _peek_latest_message(self, queue):
        return queue[-1]

    def _peek_latest_image(self, queue, border_padding=None):
        image = imgmsg_to_numpy(queue[-1])
        if border_padding is not None:
            top, bottom, left, right = border_padding
            image = cv2.copyMakeBorder(image, top, bottom, left, right, cv2.BORDER_CONSTANT, value=0)
        return image

    def _format_zero_value_failure(self, named_values):
        zero_fields = [name for name, values in named_values if np.allclose(values, 0.0)]
        if not zero_fields:
            return None
        return "all_zero=" + ",".join(zero_fields)

    def _build_single_arm_pose(self, follower_arm_pose, follower_arm_joint_state):
        return np.array(
            [
                follower_arm_pose.x,
                follower_arm_pose.y,
                follower_arm_pose.z,
                follower_arm_pose.roll,
                follower_arm_pose.pitch,
                follower_arm_pose.yaw,
                follower_arm_joint_state.position[-1],
            ]
        )

    def build_qpos(self, frame):
        return np.concatenate([np.array(frame["follower"][name].position) for name in self.robot.arm_names], axis=0)

    def build_follower_arm_pose(self, frame):
        return np.concatenate(
            [
                self._build_single_arm_pose(frame["follower_pose"][name], frame["follower"][name])
                for name in self.robot.arm_names
            ],
            axis=0,
        )

    def _camera_queues(self):
        return list(self.image_queues.values()) + list(self.depth_queues.values())

    def _stream_map(self):
        stream_map = {f"img_{name}": queue for name, queue in self.image_queues.items()}
        stream_map.update({f"leader_{name}": queue for name, queue in self.leader_queues.items()})
        stream_map.update({f"follower_{name}": queue for name, queue in self.follower_queues.items()})
        stream_map.update({f"follower_pose_{name}": queue for name, queue in self.follower_pose_queues.items()})
        stream_map.update({f"img_{name}_depth": queue for name, queue in self.depth_queues.items()})
        return stream_map

    def _get_synced_frame(self, depth_border_padding=None):
        """Pop the newest set of messages aligned to the oldest latest camera frame.

        Returns (True, frame) where frame is a dict with keys
        images/depth (camera name -> array) and follower/follower_pose/leader (arm name -> msg).
        """
        stream_map = self._stream_map()
        if any(len(queue) == 0 for queue in stream_map.values()):
            return False, self._format_sync_failure(stream_map)

        frame_time = self._resolve_frame_time(self._camera_queues())
        if not self._validate_stream_sync(stream_map, frame_time):
            return False, self._format_sync_failure(stream_map, frame_time)

        frame = {
            "images": {name: self._pop_synced_image(queue, frame_time) for name, queue in self.image_queues.items()},
            "leader": {name: self._pop_synced_message(queue, frame_time) for name, queue in self.leader_queues.items()},
            "follower": {
                name: self._pop_synced_message(queue, frame_time) for name, queue in self.follower_queues.items()
            },
            "follower_pose": {
                name: self._pop_synced_message(queue, frame_time)[1]
                for name, queue in self.follower_pose_queues.items()
            },
            "depth": {
                name: self._pop_synced_image(queue, frame_time, depth_border_padding)
                for name, queue in self.depth_queues.items()
            },
        }
        return True, frame

    def get_collection_frame(self):
        return self._get_synced_frame(depth_border_padding=self.depth_border_padding)

    def get_inference_frame(self):
        return self._get_synced_frame()

    def get_latest_inference_frame(self):
        stream_map = self._stream_map()
        if any(len(queue) == 0 for queue in stream_map.values()):
            return False, self._format_sync_failure(stream_map)

        frame = {
            "images": {name: self._peek_latest_image(queue) for name, queue in self.image_queues.items()},
            "leader": {name: self._peek_latest_message(queue) for name, queue in self.leader_queues.items()},
            "follower": {name: self._peek_latest_message(queue) for name, queue in self.follower_queues.items()},
            "follower_pose": {
                name: self._peek_latest_message(queue)[1] for name, queue in self.follower_pose_queues.items()
            },
            "depth": {name: self._peek_latest_image(queue) for name, queue in self.depth_queues.items()},
        }
        return True, frame

    def get_frame(self):
        if self.mode == "collection":
            return self.get_collection_frame()
        return self.get_inference_frame()

    def init_ros(self):
        node_name = "record_episodes" if self.mode == "collection" else "joint_state_publisher"
        if not rclpy.ok():
            # Keep Python's default SIGINT handling (KeyboardInterrupt / custom handlers in the scripts)
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node(node_name)

        subscriber_specs = []
        for camera in self.robot.cameras:
            subscriber_specs.append(
                (camera.color_topic, Image, self._make_queue_callback("image_queues", camera.name))
            )
            if self.args.use_depth_image:
                subscriber_specs.append(
                    (camera.depth_topic, Image, self._make_queue_callback("depth_queues", camera.name))
                )
        for arm in self.robot.arms:
            subscriber_specs.append(
                (arm.follower_topic, JointState, self._make_queue_callback("follower_queues", arm.name))
            )
            # PosCmd has no header, so pose messages are stamped on arrival
            subscriber_specs.append(
                (arm.follower_pose_topic, PosCmd, self._make_queue_callback("follower_pose_queues", arm.name, True))
            )
            if self.mode == "collection":
                subscriber_specs.append(
                    (arm.leader_topic, JointState, self._make_queue_callback("leader_queues", arm.name))
                )

        self._register_subscribers(subscriber_specs)

        if self.mode == "inference":
            # In mode 1 the arm node follows the leader topic, so joint commands are published there
            for arm in self.robot.arms:
                self.joint_publishers[arm.name] = self.node.create_publisher(JointState, arm.leader_topic, 10)
                self.pose_publishers[arm.name] = self.node.create_publisher(PosCmd, arm.pos_cmd_topic, 10)

        # Spin in the background so subscriber callbacks keep filling the queues
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self._spin, daemon=True)
        self.spin_thread.start()
        atexit.register(self.shutdown)

    def _spin(self):
        try:
            self.executor.spin()
        except Exception:
            # The context was shut down while spinning
            pass

    def shutdown(self):
        # Stop spinning before destroying the node so no callback runs on a destroyed entity
        if self.executor is not None:
            self.executor.shutdown(timeout_sec=1.0)
            self.executor = None
        if self.spin_thread is not None:
            self.spin_thread.join(timeout=1.0)
            self.spin_thread = None
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        rclpy.try_shutdown()

    def process(self, keyboard_handler):
        """
        Collect data continuously until space is pressed.
        Returns: (timesteps, actions, user_choice)
            - user_choice: 's' to save, 'q' to discard
        """
        timesteps = []
        actions = []
        count = 0

        rate = Rate(self.args.frame_rate)
        last_warning = None

        print("\n\033[36m=== Recording started ===\033[0m")
        print("\033[33mPress SPACE to stop recording, then 's' to save/continue or 'q' to discard/restart\033[0m\n")

        while not is_shutdown():
            key = keyboard_handler.get_key()
            if key == " ":
                print(
                    "\n\033[33m\nRecording paused. Press 's' to SAVE (continue) or 'q' to DISCARD (restart): \033[0m",
                    end="",
                    flush=True,
                )
                choice = keyboard_handler.wait_for_key(["s", "q", "S", "Q", "\x1b[C", "\x1b[D"])
                if choice == "\x1b[C":
                    choice = "s"
                elif choice == "\x1b[D":
                    choice = "q"
                else:
                    choice = choice.lower()
                print(choice)
                print("len(timesteps): ", len(timesteps))
                print("len(actions)  : ", len(actions))
                return timesteps, actions, choice

            success, result = self.get_collection_frame()
            if not success:
                warning = f"syn fail: {result}"
                if warning != last_warning:
                    print(warning)
                    last_warning = warning
                rate.sleep()
                continue

            count += 1
            frame = result
            arm_names = self.robot.arm_names

            observation = {"images": dict(frame["images"])}
            if self.args.use_depth_image:
                observation["images_depth"] = dict(frame["depth"])

            follower = [frame["follower"][name] for name in arm_names]
            leader = [frame["leader"][name] for name in arm_names]
            qpos = np.concatenate([np.array(msg.position) for msg in follower], axis=0)
            qvel = np.concatenate([np.array(msg.velocity) for msg in follower], axis=0)
            effort = np.concatenate([np.array(msg.effort) for msg in follower], axis=0)
            action = np.concatenate([np.array(msg.position) for msg in leader], axis=0)
            eef_pose = self.build_follower_arm_pose(frame)

            zero_value_error = self._format_zero_value_failure(
                [(f"follower_{name}.position", np.array(msg.position)) for name, msg in zip(arm_names, follower)]
                + [(f"leader_{name}.position", np.array(msg.position)) for name, msg in zip(arm_names, leader)]
            )
            if zero_value_error:
                warning = f"arm data invalid: {zero_value_error}"
                if warning != last_warning:
                    print(warning)
                    last_warning = warning
                rate.sleep()
                continue

            last_warning = None
            observation["qpos"] = qpos
            observation["qvel"] = qvel
            observation["effort"] = effort
            observation["eef_pose"] = eef_pose

            if count == 1:
                timesteps.append(
                    dm_env.TimeStep(
                        step_type=dm_env.StepType.FIRST,
                        reward=None,
                        discount=None,
                        observation=observation,
                    )
                )
                continue

            timesteps.append(
                dm_env.TimeStep(
                    step_type=dm_env.StepType.MID,
                    reward=None,
                    discount=None,
                    observation=observation,
                )
            )
            actions.append(action)
            print("Frame data: ", count)
            if is_shutdown():
                exit(-1)
            rate.sleep()

        print("len(timesteps): ", len(timesteps))
        print("len(actions)  : ", len(actions))
        return timesteps, actions, "q"

    def follower_arm_publish(self, arm_joint_positions):
        """Publish one 7-value joint command (6 joints + gripper) per arm, in robot arm order."""
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.node.get_clock().now().to_msg()
        joint_state_msg.name = ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        for name, positions in zip(self.robot.arm_names, arm_joint_positions, strict=True):
            joint_state_msg.position = _to_float_list(positions)
            self.joint_publishers[name].publish(joint_state_msg)

    def follower_arm_pose_publish(self, arm_poses):
        """Publish one 7-value pose command (xyz + rpy + gripper) per arm, in robot arm order."""
        for name, pose in zip(self.robot.arm_names, arm_poses, strict=True):
            pose = _to_float_list(pose)
            pose_msg = PosCmd()
            pose_msg.x, pose_msg.y, pose_msg.z = pose[:3]
            pose_msg.roll, pose_msg.pitch, pose_msg.yaw = pose[3:6]
            pose_msg.gripper = pose[6]
            self.pose_publishers[name].publish(pose_msg)

    def follower_arm_publish_continuous(self, arm_joint_targets):
        """Move every arm to its 7-value joint target in steps of at most args.arm_steps_length."""
        rate = Rate(self.args.publish_rate)
        arm_names = self.robot.arm_names
        targets = [list(target) for target in arm_joint_targets]
        if len(targets) != len(arm_names):
            raise ValueError(f"Expected {len(arm_names)} arm targets, got {len(targets)}")

        current = {}
        while not is_shutdown():
            for name in arm_names:
                if len(self.follower_queues[name]) != 0:
                    current[name] = list(self.follower_queues[name][-1].position)
            if len(current) == len(arm_names):
                break
            rate.sleep()
        if len(current) != len(arm_names):
            # Shut down before any follower state arrived
            return
        current = [current[name] for name in arm_names]

        directions = [
            [1 if target[i] - arm[i] > 0 else -1 for i in range(len(target))] for target, arm in zip(targets, current)
        ]
        moving = True
        step = 0
        while moving and not is_shutdown():
            if self.follower_arm_publish_lock.acquire(False):
                return
            moving = False
            for target, arm, direction in zip(targets, current, directions):
                for i in range(len(target)):
                    if abs(target[i] - arm[i]) < self.args.arm_steps_length[i]:
                        arm[i] = target[i]
                    else:
                        arm[i] += direction[i] * self.args.arm_steps_length[i]
                        moving = True
            self.follower_arm_publish(current)
            step += 1
            print("follower_arm_publish_continuous:", step)
            rate.sleep()


def get_ros_observation(args, ros_operator):
    """Block until a synchronized frame is available and return it (see RosOperator._get_synced_frame)."""
    rate = Rate(args.publish_rate)
    print_flag = True

    while not is_shutdown():
        success, frame_or_error = ros_operator.get_inference_frame()
        if not success:
            if print_flag:
                print(f"syn fail when get_ros_observation: {frame_or_error}")
                print_flag = False
            rate.sleep()
            continue
        return frame_or_error


def get_latest_ros_observation(args, ros_operator):
    """Block until every stream has data and return the latest messages as a frame dict."""
    rate = Rate(args.publish_rate)
    print_flag = True

    while not is_shutdown():
        success, frame_or_error = ros_operator.get_latest_inference_frame()
        if not success:
            if print_flag:
                print(f"latest obs unavailable: {frame_or_error}")
                print_flag = False
            rate.sleep()
            continue
        return frame_or_error
