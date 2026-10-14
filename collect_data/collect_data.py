# -- coding: UTF-8
import argparse
import os
import select
import subprocess
import sys
import termios
import time
import tty

import h5py
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from robot_config import add_robot_argument, get_robot_config
from ros_operator import RosOperator, is_shutdown

# Must match EPS in convert_data/convert_to_lerobot.py: that script rejects an
# episode at conversion time ("Found no qpos that exceeds the threshold.") if
# qpos never differs from its first/last frame by more than this in any
# dimension, i.e. the whole episode is flat. Checking it here catches a dead
# episode (e.g. a stale follower joint-state topic) right after recording
# instead of during conversion.
MOTION_EPS = 5e-3


class KeyboardHandler:
    """Non-blocking keyboard input handler for terminal."""

    def __init__(self):
        self.old_settings = None

    def setup(self):
        """Set terminal to raw mode for non-blocking input."""
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def cleanup(self):
        """Restore terminal settings."""
        if self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def get_key(self):
        """Get a key if one is pressed, otherwise return None (non-blocking)."""
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def wait_for_key(self, valid_keys):
        """Wait for a specific key from valid_keys list (blocking).

        Arrow keys arrive as a 3-byte escape sequence (ESC [ C / ESC [ D for
        right/left); pass those as the literal strings "\\x1b[C" / "\\x1b[D"
        in valid_keys to match them.
        """
        while True:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key == "\x1b" and select.select([sys.stdin], [], [], 0.1)[0]:
                    key += sys.stdin.read(2)
                if key in valid_keys:
                    return key


def check_episode_motion(timesteps, actions, eps=MOTION_EPS):
    """Check whether recorded observations/action show real motion.

    Returns (qpos_is_flat, flat_fields) where flat_fields lists every field
    whose per-dimension range across the episode never exceeds eps - the
    same condition convert_data/convert_to_lerobot.py's cut_head/cut_tail
    checks use to reject an episode. qpos is the field that actually gates
    conversion; the others are reported for diagnosis since a stale upstream
    topic tends to freeze all of them together.
    """
    num_frames = len(actions)
    fields = {
        "qpos": np.stack([ts.observation["qpos"] for ts in timesteps[:num_frames]]),
        "qvel": np.stack([ts.observation["qvel"] for ts in timesteps[:num_frames]]),
        "effort": np.stack([ts.observation["effort"] for ts in timesteps[:num_frames]]),
        "eef_pose": np.stack([ts.observation["eef_pose"] for ts in timesteps[:num_frames]]),
        "action": np.stack(actions),
    }

    flat_fields = []
    for name, values in fields.items():
        value_range = float((values.max(axis=0) - values.min(axis=0)).max())
        if value_range <= eps:
            flat_fields.append((name, value_range))

    qpos_is_flat = any(name == "qpos" for name, _ in flat_fields)
    return qpos_is_flat, flat_fields


class LeRobotConverter:
    """Converts saved hdf5 episodes into a LeRobot dataset in a separate process.

    Runs convert_data/convert_to_lerobot.py --stdin with the lerobot venv's python (this
    venv has no torch/lerobot), so video encoding neither blocks recording nor competes
    for the GIL. Appends to the dataset if it already exists.
    """

    def __init__(self, python, tgt_path, repo_id, robot, cut_head=False, cut_tail=False):
        python = os.path.expanduser(python)
        if not os.path.exists(python):
            raise FileNotFoundError(f"--lerobot_python not found: {python}")
        script = os.path.join(PROJECT_ROOT, "convert_data", "convert_to_lerobot.py")
        cmd = [python, script, "--stdin", "--tgt_path", tgt_path, "--save_repoid", repo_id, "--robot", robot]
        cmd.append("--cut_head" if cut_head else "--no-cut_head")
        cmd.append("--cut_tail" if cut_tail else "--no-cut_tail")
        # New session: Ctrl+C in the terminal must not interrupt an in-flight episode encode
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, text=True, start_new_session=True)
        self.target_dir = os.path.join(tgt_path, repo_id)

    def submit(self, hdf5_path):
        try:
            self.proc.stdin.write(os.path.abspath(hdf5_path) + "\n")
            self.proc.stdin.flush()
        except (BrokenPipeError, ValueError):
            print("\033[31m[LeRobot] Converter process is not running; episode not converted "
                  f"(hdf5 kept): {hdf5_path}\033[0m")

    def close(self):
        """Close stdin and wait for the worker to convert the rest and finalize the dataset."""
        print("\033[33m[LeRobot] Finishing queued conversions (don't kill)...\033[0m")
        try:
            self.proc.stdin.close()
        except (BrokenPipeError, ValueError):
            pass
        self.proc.wait()
        if self.proc.returncode != 0:
            print(f"\033[31m[LeRobot] Converter exited with code {self.proc.returncode}\033[0m")


def save_data(args, timesteps, actions, dataset_path):
    data_size = len(actions)
    camera_names = get_robot_config(args.robot).camera_names
    first_observation = timesteps[0].observation
    data_dict = {
        "/observations/qpos": [],
        "/observations/qvel": [],
        "/observations/effort": [],
        "/observations/eef_pose": [],
        "/action": [],
        "/collect": [],
    }

    for cam_name in camera_names:
        data_dict[f"/observations/images/{cam_name}"] = []
        if args.use_depth_image:
            data_dict[f"/observations/images_depth/{cam_name}"] = []

    while actions:
        action = actions.pop(0)
        ts = timesteps.pop(0)

        data_dict["/observations/qpos"].append(ts.observation["qpos"])
        data_dict["/observations/qvel"].append(ts.observation["qvel"])
        data_dict["/observations/effort"].append(ts.observation["effort"])
        data_dict["/observations/eef_pose"].append(ts.observation["eef_pose"])

        data_dict["/action"].append(action)
        data_dict["/collect"].append("teleop")

        for cam_name in camera_names:
            data_dict[f"/observations/images/{cam_name}"].append(ts.observation["images"][cam_name])
            if args.use_depth_image:
                data_dict[f"/observations/images_depth/{cam_name}"].append(ts.observation["images_depth"][cam_name])

    t0 = time.time()
    with h5py.File(dataset_path + ".hdf5", "w", rdcc_nbytes=1024**2 * 2) as root:
        obs = root.create_group("observations")
        image = obs.create_group("images")
        for cam_name in camera_names:
            image_shape = first_observation["images"][cam_name].shape
            _ = image.create_dataset(
                cam_name,
                (data_size, *image_shape),
                dtype="uint8",
                chunks=(1, *image_shape),
            )
        if args.use_depth_image:
            image_depth = obs.create_group("images_depth")
            for cam_name in camera_names:
                depth_shape = first_observation["images_depth"][cam_name].shape
                _ = image_depth.create_dataset(
                    cam_name,
                    (data_size, *depth_shape),
                    dtype="uint16",
                    chunks=(1, *depth_shape),
                )

        # 7 values (6 joints + gripper) per arm
        state_dim = len(first_observation["qpos"])
        _ = obs.create_dataset("qpos", (data_size, state_dim), dtype="float32")
        _ = obs.create_dataset("qvel", (data_size, state_dim), dtype="float32")
        _ = obs.create_dataset("effort", (data_size, state_dim), dtype="float32")
        _ = obs.create_dataset("eef_pose", (data_size, state_dim), dtype="float32")
        _ = root.create_dataset("action", (data_size, state_dim), dtype="float32")
        _ = root.create_dataset("collect", (data_size,), dtype=h5py.string_dtype(encoding="utf-8"))

        # data_dict write into h5py.File
        for name, array in data_dict.items():
            root[name][...] = array
    print(f"\033[32m\nSaving: {time.time() - t0:.1f} secs. %s \033[0m\n" % dataset_path)


def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dataset_dir",
        action="store",
        type=str,
        help="Dataset_dir.",
        default="",
        required=False,
    )
    parser.add_argument(
        "--task_name",
        action="store",
        type=str,
        help="Task name.",
        default="test",
        required=True,
    )
    parser.add_argument(
        "--episode_idx",
        action="store",
        type=int,
        help="Starting episode index (auto-increments after each save).",
        default=0,
        required=False,
    )
    add_robot_argument(parser)

    # collect depth image
    parser.add_argument(
        "--use_depth_image",
        action="store_true",
        help="use_depth_image",
        required=False,
    )

    parser.add_argument(
        "--frame_rate",
        action="store",
        type=int,
        help="frame_rate",
        default=30,
        required=False,
    )

    parser.add_argument(
        "--lerobot_dir",
        type=str,
        default=None,
        help="If set, also convert each saved episode into a LeRobot dataset at <lerobot_dir>/<task_name> "
        "(appends if it already exists). The hdf5 file is always kept.",
    )
    parser.add_argument(
        "--lerobot_python",
        type=str,
        default="~/venvs/lerobot/bin/python",
        help="with --lerobot_dir: python of the lerobot venv used for conversion",
    )
    parser.add_argument(
        "--cut_head",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="with --lerobot_dir: trim initial still frames",
    )
    parser.add_argument(
        "--cut_tail",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="with --lerobot_dir: trim final still frames",
    )

    args = parser.parse_args()
    return args


def get_next_episode_idx(dataset_dir):
    """Find the next available episode index in the dataset directory."""
    if not os.path.exists(dataset_dir):
        return 0
    existing_episodes = [f for f in os.listdir(dataset_dir) if f.startswith("episode_") and f.endswith(".hdf5")]
    if not existing_episodes:
        return 0
    indices = []
    for ep in existing_episodes:
        try:
            idx = int(ep.replace("episode_", "").replace(".hdf5", ""))
            indices.append(idx)
        except ValueError:
            continue
    return max(indices) + 1 if indices else 0


def main():
    args = get_arguments()
    ros_operator = RosOperator(args, mode="collection")
    dataset_dir = os.path.join(args.dataset_dir, args.task_name)

    # Create dataset directory if it doesn't exist
    if not os.path.exists(dataset_dir):
        os.makedirs(dataset_dir)

    converter = None
    if args.lerobot_dir:
        converter = LeRobotConverter(
            args.lerobot_python,
            args.lerobot_dir,
            args.task_name,
            args.robot,
            args.cut_head,
            args.cut_tail,
        )

    # Initialize keyboard handler
    keyboard_handler = KeyboardHandler()
    keyboard_handler.setup()

    # Start with provided episode_idx or find next available
    if args.episode_idx == 0:
        # Auto-detect next available episode index
        current_episode_idx = get_next_episode_idx(dataset_dir)
    else:
        current_episode_idx = args.episode_idx

    try:
        print("\n\033[32m========================================\033[0m")
        print("\033[32m   Continuous Data Collection Started   \033[0m")
        print("\033[32m========================================\033[0m")
        print("\033[33mControls:\033[0m")
        print("  - Press ENTER to start recording")
        print("  - Press SPACE to stop current recording")
        print("  - Press -> (right arrow) or ['s'] to SAVE/continue")
        print("  - Press <- (left arrow) or ['q'] to DISCARD/restart")
        print("    (/ also still work)")
        print("  - Press Ctrl+C to exit the program")
        print("\033[32m========================================\033[0m\n")

        while not is_shutdown():
            print(f"\n\033[36m>>> Episode {current_episode_idx} ready <<<\033[0m")
            print("\033[33mPress ENTER to start recording...\033[0m", end="", flush=True)

            # Wait for Enter key to start
            keyboard_handler.wait_for_key(["\n", "\r"])
            print()

            # Clear the data queues before starting new episode
            ros_operator.reset()

            # Collect data until space is pressed
            timesteps, actions, choice = ros_operator.process(keyboard_handler)

            if choice == "s":
                # Save the data
                if len(actions) == 0:
                    print("\033[31m\nNo data to save (0 frames recorded).\033[0m")
                else:
                    proceed = True
                    qpos_is_flat, flat_fields = check_episode_motion(timesteps, actions)
                    if qpos_is_flat:
                        print("\033[31m\n*** WARNING: no joint motion detected in this episode! ***\033[0m")
                        for name, value_range in flat_fields:
                            print(f"\033[31m    /{name}: max range {value_range:.5f} <= {MOTION_EPS} (flat)\033[0m")
                        print(
                            "\033[31mThis episode would be rejected during HDF5->lerobot conversion "
                            '("Found no qpos that exceeds the threshold."). The follower arm/joint-state '
                            "topic likely never updated during recording - check the robot connection.\033[0m"
                        )
                        print(
                            "\033[33mPress -> or 's' to save anyway, <- or 'q' to discard and retry: \033[0m",
                            end="",
                            flush=True,
                        )
                        confirm = keyboard_handler.wait_for_key(["s", "\x1b[C", "q", "\x1b[D"])
                        print()
                        proceed = confirm in ("s", "\x1b[C")

                    if proceed:
                        dataset_path = os.path.join(dataset_dir, "episode_" + str(current_episode_idx))
                        save_data(args, timesteps, actions, dataset_path)
                        print(f"\033[32mEpisode {current_episode_idx} saved successfully!\033[0m")
                        if converter is not None:
                            converter.submit(dataset_path + ".hdf5")
                        current_episode_idx += 1
                    else:
                        print(f"\033[31m\nEpisode discarded (no motion detected). {len(actions)} frames thrown away.\033[0m")
            else:
                # Discard the data
                print(f"\033[31m\nEpisode discarded. {len(actions)} frames thrown away.\033[0m")

            time.sleep(0.5)  # Brief pause before next episode

    except KeyboardInterrupt:
        print("\n\033[33m\nExiting data collection...\033[0m")
    finally:
        keyboard_handler.cleanup()
        if converter is not None:
            converter.close()
        print("\033[32mData collection ended.\033[0m")


if __name__ == "__main__":
    main()

# python collect_data_ori.py --dataset_dir ~/data --task_name aloha_mobile_dummy --episode_idx 0
# Controls: Press SPACE to stop, then -> to save or <- to discard ('s'/'q' also still work)
