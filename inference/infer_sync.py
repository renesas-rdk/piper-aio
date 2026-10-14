import argparse
import os
import signal
import sys
import termios
import threading
import time
import tty
from collections import deque

import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from clients import OpenpiClient
from utils import (
    InferenceDataRecorder,
    check_keyboard_input,
    get_config,
    get_inference_observation,
    get_policy_images,
    get_rollout_observation,
    handle_interactive_mode,
    process_action,
)

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from robot_config import add_robot_argument
from ros_operator import Rate, RosOperator, is_shutdown, signal_shutdown

observation_window = None
observation_window_lock = threading.Lock()

shutdown_event = threading.Event()


def _on_sigint(signum, frame):
    try:
        shutdown_event.set()
    except Exception:
        pass
    try:
        signal_shutdown("SIGINT")
    except Exception:
        pass


def reset_observation_window():
    """Reset observation window for a new episode."""
    global observation_window
    with observation_window_lock:
        observation_window = None


def update_observation_window(args, config, ros_operator):
    global observation_window
    with observation_window_lock:
        if observation_window is None:
            observation_window = deque(maxlen=2)

            # Append the first dummy image
            observation_window.append(
                {
                    "qpos": None,
                    "images": {name: None for name in config["camera_names"]},
                    "eef_pose": None,
                }
            )

    observation = get_inference_observation(args, config, ros_operator)
    if observation is None:
        return False

    with observation_window_lock:
        observation_window.append(observation)

    return True


def inference_fn_sync(args, config, policy, ros_operator):
    if not update_observation_window(args, config, ros_operator):
        return None

    start_time = time.perf_counter()

    with observation_window_lock:
        # images keyed by camera name, in robot camera order
        images = get_policy_images(observation_window[-1], config)

        if args.ctrl_type == "joint":
            # state: Abs Joint (7 per arm)
            state = observation_window[-1]["qpos"]
        elif args.ctrl_type == "eef":
            # state: Abs EEF (7 per arm)
            state = observation_window[-1]["eef_pose"]
        else:
            raise ValueError(f"Unknown ctrl_type: {args.ctrl_type}")

    payload = {
        "images": images,
        "instruction": config["language_instruction"],
        "state": state,
    }

    actions = policy.predict_action(payload)
    print(f"Model inference time: {(time.perf_counter() - start_time)*1000:.3f} ms")

    return actions


# Main loop for the manipulation task
def model_inference(args, config, ros_operator):
    if args.model == "openpi":
        policy = OpenpiClient(
            host=args.host,
            port=args.port,
            camera_names=config["camera_names"],
            state_dim=config["state_dim"],
        )
    else:
        raise ValueError(f"Unknown model: {args.model}")

    max_publish_step = config["episode_len"]
    chunk_size = config["chunk_size"]

    # Initial joint positions, one 7-value list per arm
    init_qpos = config["init_qpos"]

    ros_operator.follower_arm_publish_continuous(init_qpos)

    print("Warmup the server...")
    policy.warmup()
    print("Server warmed up")

    input("Press enter to continue")
    task_time = time.time()
    ros_operator.follower_arm_publish_continuous(init_qpos)
    recorder = InferenceDataRecorder(args, config, shutdown_event=shutdown_event)

    try:
        # Inference loop
        while not is_shutdown():
            # The current time step
            t = 0
            rate = Rate(args.publish_rate)

            reset_observation_window()
            action_buffer = np.zeros([chunk_size, config["state_dim"]])
            episode_closed = False

            while t < max_publish_step and not is_shutdown() and not shutdown_event.is_set():
                # Check for keyboard input (space to enter interactive mode)
                key = check_keyboard_input()
                if key == " ":
                    result = handle_interactive_mode(task_time)
                    if result == "reset":
                        recorder.save_episode()
                        episode_closed = True
                        # Reset to starting position
                        ros_operator.follower_arm_publish_continuous(init_qpos)
                        input("Press enter to continue")
                        task_time = time.time()
                        break  # Break inner loop to restart
                    elif result == "quit":
                        recorder.save_episode()
                        return  # Exit the function entirely
                    # 'continue' just resumes the loop

                # When coming to the end of the action chunk
                if t % chunk_size == 0:
                    # Start inference
                    action_buffer = inference_fn_sync(args, config, policy, ros_operator)
                    if action_buffer is None:
                        break
                    assert action_buffer is not None, "Sync inference returned None"
                    assert (
                        action_buffer.shape[0] >= chunk_size
                    ), f"Action chunk length {action_buffer.shape[0]} is smaller than {chunk_size}"

                act = action_buffer[t % chunk_size]
                observation_to_save = get_rollout_observation(args, config, ros_operator) if recorder.enabled else None
                if recorder.enabled and observation_to_save is None:
                    break

                arm_actions = process_action(config, act)
                action_to_save = np.concatenate(arm_actions, axis=0)
                if args.ctrl_type == "joint":
                    ros_operator.follower_arm_publish(arm_actions)
                elif args.ctrl_type == "eef":
                    ros_operator.follower_arm_pose_publish(arm_actions)

                recorder.add_step(observation_to_save, action_to_save)
                t += 1
                print("Published Step", t)
                rate.sleep()

            if not episode_closed:
                recorder.save_episode()
            if shutdown_event.is_set():
                return
    finally:
        ros_operator.follower_arm_publish_continuous(init_qpos)


def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--max_publish_step",
        action="store",
        type=int,
        help="Maximum number of action publishing steps",
        default=10000,
        required=False,
    )
    add_robot_argument(parser)
    parser.add_argument(
        "--publish_rate",
        action="store",
        type=int,
        help="The rate at which to publish the actions",
        default=30,
        required=False,
    )
    parser.add_argument(
        "--chunk_size",
        action="store",
        type=int,
        help="Action chunk size",
        default=50,
        required=False,
    )
    parser.add_argument(
        "--arm_steps_length",
        action="store",
        type=float,
        nargs=7,
        help="The maximum change allowed for each joint per timestep (7 values)",
        default=[0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.2],
        required=False,
    )
    parser.add_argument(
        "--use_depth_image",
        action="store_true",
        help="Whether to use depth images",
        default=False,
        required=False,
    )
    parser.add_argument(
        "--save_rollout",
        action="store_true",
        help="Save rollout observations/actions to HDF5 episodes",
        default=False,
        required=False,
    )
    parser.add_argument(
        "--save_dir",
        action="store",
        type=str,
        help="Directory used when --save_rollout is set.",
        default="",
        required=False,
    )
    parser.add_argument(
        "--ctrl_type",
        type=str,
        choices=["joint", "eef"],
        help="Control type for the robot arm",
        default="joint",
    )
    parser.add_argument(
        "--host",
        action="store",
        type=str,
        help="Websocket server host",
        default="127.0.0.1",
        required=False,
    )
    parser.add_argument(
        "--port",
        action="store",
        type=int,
        help="Websocket server port",
        default=8000,
        required=False,
    )
    parser.add_argument(
        "--task",
        action="store",
        type=str,
        help="Task name",
        required=True,
    )
    parser.add_argument(
        "--model",
        type=str,
        choices=["openpi"],
        help="Model to use",
        default="openpi",
        required=False,
    )

    args = parser.parse_args()
    return args


def main():
    args = get_arguments()
    ros_operator = RosOperator(args, mode="inference")
    config = get_config(args)

    signal.signal(signal.SIGINT, _on_sigint)

    # Set terminal to raw mode for non-blocking keyboard input
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        model_inference(args, config, ros_operator)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


if __name__ == "__main__":
    main()
