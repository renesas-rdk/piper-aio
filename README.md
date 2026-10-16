<p align="center">
  <h1 align="center">Piper All In One</h1>
</p>

Piper All In One is an imitation-learning stack for [AgileX Piper](https://global.agilex.ai/products/piper) robotic arms and [Cobot Magic](https://global.agilex.ai/products/cobot-magic) systems. The workflow maintained in this repository is: **teleoperated data collection (`start_collect.sh`) → conversion to a LeRobot dataset → upload to the Hugging Face Hub**.

> ℹ️ The replay, visualization, and policy-inference scripts inherited from the original repository are still present but have **not been re-verified** with the current ROS 2 / LeRobot v3.0 setup, so they are not documented here.

> ‼️ Safety first: this repository controls real robot arms. Keep the emergency stop reachable, start from a clear workspace, verify CAN/camera mappings before motion, and keep teleoperation motions slow and supervised.

## 📰 News

- **Oct 14 2026**: Documented the verified workflow: `start_collect.sh` → LeRobot conversion → Hugging Face Hub upload. Replay, visualization, and inference are unverified and no longer documented.
- **Sep 24 2026**: Added a single-arm setup (one leader-follower pair, a side camera and a wrist camera) with support for both RealSense and USB (UVC) cameras.
- **Sep 24 2026**: Ported to ROS 2 Jazzy (Ubuntu 24.04) and LeRobot v0.6.1 (dataset `v3.0`).
- **Apr 29 2026**: Repository released.

## 📋 Contents

- [📰 News](#-news)
- [📋 Contents](#-contents)
- [🗂️ Repository Layout](#️-repository-layout)
- [⚙️ Setup](#️-setup)
  - [Prerequisites](#prerequisites)
    - [Hardware](#hardware)
    - [Software](#software)
  - [Installation](#installation)
    - [Clone the Repository](#clone-the-repository)
    - [Install System Packages](#install-system-packages)
    - [Configure CAN Interfaces](#configure-can-interfaces)
    - [Create Python Environments](#create-python-environments)
    - [Build the Piper Workspace](#build-the-piper-workspace)
    - [Configure Cameras](#configure-cameras)
      - [Dual-Arm Cameras](#dual-arm-cameras)
      - [Single-Arm Cameras](#single-arm-cameras)
- [🚀 Usage](#-usage)
  - [Collect Data](#collect-data)
  - [Convert Data to LeRobot](#convert-data-to-lerobot)
  - [Upload the Dataset to the Hugging Face Hub](#upload-the-dataset-to-the-hugging-face-hub)
- [🧩 Robot Configs](#-robot-configs)
- [📊 Dataset Format](#-dataset-format)
- [🙏 Acknowledgements](#-acknowledgements)

## 🗂️ Repository Layout

```text
.
├── piper_ws/                          # ROS 2 workspace for Piper messages, arm node, and arm/camera launch files
├── start_collect.sh                   # One-shot launcher: cameras + arms + viewers + data collection
├── collect_data/                      # Data collection script (collect_data.py)
├── convert_data/                      # HDF5 -> LeRobot conversion and Hub upload scripts
├── lerobot/                           # LeRobot submodule used for dataset conversion
├── robot_configs.yaml                 # Robot layouts (arms, cameras, topics): dual and single
├── robot_config.py                    # Loader for robot_configs.yaml
├── start_cam.sh                       # Camera launcher (used by start_collect.sh)
└── start_mode0.sh                     # Teleoperation arm launcher (used by start_collect.sh)
```

## ⚙️ Setup

### Prerequisites

#### Hardware

Two robot layouts are supported. Select one with the first argument of `start_collect.sh` (`single` or `dual`) or with `--robot` on the Python scripts (see [Robot Configs](#-robot-configs)).

Dual-arm (`dual`, default):

- **Four AgileX Piper or PiperX arms**: two leader arms for teleoperation, and two follower arms for execution.
- **Two USB-to-CAN modules**: one for each leader-follower pair.
- **Three RealSense cameras**: one front camera and two wrist cameras, with each wrist camera mounted on a follower arm.

Single-arm (`single`):

- **Two AgileX Piper or PiperX arms**: one leader arm for teleoperation and one follower arm for execution.
- **One USB-to-CAN module** for the leader-follower pair.
- **Two cameras**: one side camera and one wrist camera mounted on the follower arm. Each can be a RealSense camera or a standard USB (UVC) camera.

Both layouts need **a control machine** with enough USB bandwidth for all cameras and CAN devices.

#### Software

- **Ubuntu 24.04**: required by ROS 2 Jazzy.
- **ROS 2 Jazzy**: follow the [installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) and install `ros-jazzy-desktop`.
- **Python 3.12**: the system Python of Ubuntu 24.04. It is used by ROS 2 Jazzy and required by LeRobot v0.6.
- **Terminator**: recommended when running multiple ROS terminals.

### Installation

#### Clone the Repository

```bash
git clone --recurse-submodules https://github.com/renesas-rdk/piper-aio.git
cd piper-aio
```

If you already cloned without submodules, initialize them with:

```bash
git submodule update --init --recursive
```

#### Install System Packages

```bash
sudo apt update
sudo apt install -y \
  python3-venv \
  python3-colcon-common-extensions \
  can-utils \
  ethtool \
  net-tools

sudo apt install -y \
  ros-jazzy-realsense2-camera \
  ros-jazzy-v4l2-camera \
  ros-jazzy-rqt-image-view \
  v4l-utils
```

`ros-jazzy-realsense2-camera` installs the ROS 2 RealSense wrapper together with `librealsense2`, so the RealSense camera workspace is no longer built from source. `ros-jazzy-v4l2-camera` drives standard USB (UVC) cameras.

#### Configure CAN Interfaces

Connect the CAN modules, then list their USB bus information:

```bash
bash piper_ws/scripts/find_all_can_port.sh
```

Edit [`piper_ws/scripts/can_config.sh`](piper_ws/scripts/can_config.sh):

- Set `EXPECTED_CAN_COUNT` to the number of connected CAN modules.
- Update the `USB_PORTS` mapping so each `bus-info` value maps to the intended CAN interface name and bitrate.
- The default two-module mapping uses `can_left:1000000` and `can_right:1000000`.
- For the single-arm layout, set `EXPECTED_CAN_COUNT=1`. The single module is then named `can0` at 1 Mbps, which is what `./start_mode0.sh single` use by default (override with `can_port:=<name>`).

After each machine startup, run the CAN configuration script once to rename and bring up the CAN interfaces:

```bash
bash piper_ws/scripts/can_config.sh
```

#### Create Python Environments

Create the main environment used by ROS control and data collection. ROS 2 Python packages (`rclpy`, message types) live in the system Python under `/opt/ros/jazzy`, so the environment must be a Python 3.12 virtual environment with access to system site packages. Conda environments with other Python versions cannot import `rclpy`.

```bash
python3 -m venv --system-site-packages ~/venvs/piperaio
source ~/venvs/piperaio/bin/activate

pip install python-can
pip install git+https://github.com/agilexrobotics/pyAgxArm.git
pip install h5py opencv-python matplotlib dm-env pyyaml
```

`pyyaml` is also used to read [`robot_configs.yaml`](robot_configs.yaml). The arm node uses [`pyAgxArm`](https://github.com/agilexrobotics/pyAgxArm) to talk to the Piper CAN interface; it is not on PyPI, so it installs directly from its GitHub repo (`piper_sdk` stays installed too, in case you need to roll back).

Create a separate environment for [LeRobot](https://github.com/huggingface/lerobot), used for dataset conversion and Hub upload. `start_collect.sh` also calls it for automatic conversion, expecting it at `~/venvs/lerobot` by default (v0.6.1, dataset `v3.0`):

```bash
python3 -m venv ~/venvs/lerobot
source ~/venvs/lerobot/bin/activate

cd lerobot
pip install -e ".[dataset]"
pip install h5py
cd ..
```

LeRobot encodes videos through PyAV, whose wheels bundle FFmpeg with AV1 support, so a separate FFmpeg install is not needed.

Note: Keep the `lerobot` and `datasets` package versions consistent between conversion and training. Datasets converted with this release use the LeRobot `v3.0` format, which older LeRobot versions (dataset `v2.1`) cannot read. Existing local `v2.1` datasets can be upgraded in place with `python -m lerobot.scripts.convert_dataset_v21_to_v30 --repo-id=<repo_id> --root=<dataset_dir> --push-to-hub=false`.

#### Build the Piper Workspace

```bash
source /opt/ros/jazzy/setup.bash

cd piper_ws
colcon build --symlink-install
cd ..
```

Source ROS 2 and the Piper workspace in your shell startup file:

```bash
# Bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source $(pwd)/piper_ws/install/setup.bash" >> ~/.bashrc

# Zsh
echo "source /opt/ros/jazzy/setup.zsh" >> ~/.zshrc
echo "source $(pwd)/piper_ws/install/setup.zsh" >> ~/.zshrc
```

The arm node runs with the `python3` found on `PATH`, so activate the `piperaio` environment before launching the arms. Otherwise `pyAgxArm` will not be found.

#### Configure Cameras

##### Dual-Arm Cameras

List connected camera serial numbers:

```bash
bash piper_ws/scripts/rs_camera_serial.sh
```

Set `serial_no_camera1` (left wrist), `serial_no_camera2` (right wrist), and `serial_no_camera3` (front) in [`start_cam.sh`](start_cam.sh) according to their physical positions. The cameras are started by [`piper_ws/src/piper/launch/multi_camera.launch.py`](piper_ws/src/piper/launch/multi_camera.launch.py), which runs the `realsense2_camera` ROS 2 driver and publishes `/camera_{f,l,r}/color/image_raw` and `/camera_{f,l,r}/depth/image_rect_raw`.

##### Single-Arm Cameras

The side and wrist cameras are configured in [`piper_ws/src/piper/config/cameras_single.yaml`](piper_ws/src/piper/config/cameras_single.yaml) and started by [`piper_ws/src/piper/launch/cameras.launch.py`](piper_ws/src/piper/launch/cameras.launch.py). Each camera is either:

- `type: realsense`: identified by `serial_no`. Optionally publishes depth with `enable_depth: true`.
- `type: usb`: any UVC/V4L2 camera (webcam), identified by `video_device`.

List connected RealSense serial numbers and USB camera device paths:

```bash
bash piper_ws/scripts/list_cameras.sh
```

Example with a USB side camera and a RealSense wrist camera:

```yaml
cameras:
  - name: camera_side
    type: usb
    video_device: /dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_XXXXXXXX-video-index0
    width: 640
    height: 480

  - name: camera_wrist
    type: realsense
    serial_no: "814412072260"
    width: 640
    height: 480
    fps: 30
```

Notes:

- Every camera publishes `/<name>/color/image_raw` regardless of its type, so collection does not depend on the camera type. Keep the names `camera_side` and `camera_wrist`, or update the `single` topics in [`robot_configs.yaml`](robot_configs.yaml) to match.
- Prefer the stable `/dev/v4l/by-id/...-video-index0` paths over `/dev/videoN`, which can change between boots. RealSense cameras also appear as V4L2 devices; configure them as `type: realsense`.
- For USB cameras, `width`/`height` must be a size the camera supports (`v4l2-ctl -d <video_device> --list-formats-ext`). The frame rate is the camera default for that size, typically 30 fps. Some webcams lower their frame rate in dim light when auto exposure is on.
- The workspace is built with `--symlink-install`, so edits to the YAML file apply without rebuilding. You can also pass another file with `ros2 launch piper cameras.launch.py config:=/path/to/cameras.yaml`.

## 🚀 Usage

Source ROS 2 and the Piper workspace, and activate the `piperaio` environment (`source ~/venvs/piperaio/bin/activate`) before collecting data. Power on all arms first.

### Collect Data

`start_collect.sh` starts the cameras, the arms in teleoperation mode (mode 0), a live view per camera, and `collect_data.py` with a single command:

```bash
./start_collect.sh single --dataset_dir ~/data --task_name pick_cube    # single-arm layout
./start_collect.sh dual --dataset_dir ~/data --task_name fold_towel     # dual-arm layout
./start_collect.sh --no-view --dataset_dir ~/data --task_name pick_cube # no camera viewers (e.g. over SSH without X11)
```

Arguments:

- First argument: robot layout, `single` (script default) or `dual`.
- `--no-view`: skip the `rqt_image_view` camera viewers. Viewers need a Qt binding (`sudo apt install python3-pyqt5` or `pip install PyQt5`).
- `--no-lerobot`: disable the automatic LeRobot conversion (see below).
- `--cut_head` / `--cut_tail`: trim still frames at the start/end of each converted episode.
- All other arguments are forwarded to [`collect_data/collect_data.py`](collect_data/collect_data.py): `--dataset_dir`, `--task_name` (required), `--episode_idx`, `--use_depth_image`, `--frame_rate` (default 30), `--lerobot_dir`, `--lerobot_python`.

Keyboard controls:

- `ENTER`: start recording an episode.
- `SPACE`: stop the current recording.
- `s`: save the episode and continue to the next one.
- `q`: discard the episode and restart it.
- `Ctrl+C`: exit; every process started by the script (cameras, arms, viewers) is stopped.

Episodes are saved to `{dataset_dir}/{task_name}/episode_{episode_idx}.hdf5`. When `--episode_idx` is omitted or `0`, the next available index in the save directory is used.

**Automatic LeRobot conversion.** Unless `--no-lerobot` is given, every saved episode is also converted in the background (using `~/venvs/lerobot/bin/python`, override with `--lerobot_python`) and appended to a LeRobot dataset at `{dataset_dir}/lerobot_convert/{task_name}`. The HDF5 file is always kept. Set a different output root with `--lerobot_dir`. For example, with `--dataset_dir ~/data --task_name pick_cube`:

```text
<dataset_dir>
├── lerobot_convert          # LeRobot datasets (automatic conversion)
│   └── <task_name>          # LeRobot v3.0 dataset: data/, videos/, meta/
└── <task_name>              # raw HDF5 episodes
    ├── episode_0.hdf5
    └── episode_1.hdf5
```

Pass this `lerobot_convert/<task_name>` directory as `--root` when [uploading to the Hub](#upload-the-dataset-to-the-hugging-face-hub).

`collect_data.py` warns when an episode would be rejected by the conversion.

Logs of the camera/arm launches and viewers are kept under `/tmp/piper_collect.<robot>.*` for troubleshooting.

Note: the system may occasionally drop frames and report `syn fail`, so avoid collecting data too quickly, especially during complex motions. Also check USB bandwidth and verify all camera and arm topics are publishing at the expected rate.

### Convert Data to LeRobot

If you used the automatic conversion above, the dataset already exists at `{dataset_dir}/lerobot_convert/{task_name}` and you can go to the next step. To (re)convert existing HDF5 episodes in batch, e.g. after rejecting bad episodes or to select an episode range:

Before conversion, edit [`convert_data/convert_to_lerobot.py`](convert_data/convert_to_lerobot.py):

- Set `INSTRUCTION` near the top of the file to the language instruction used for training.
- Add any rejected HDF5 episode paths to `exclude_files` in `main()`.

Then convert one or more task directories to a LeRobot v3.0 dataset using the feature schema in [`convert_data/features.py`](convert_data/features.py). The cameras and state size follow the `--robot` layout, and each camera's image size is read from the recorded episodes:

```bash
source ~/venvs/lerobot/bin/activate

python convert_data/convert_to_lerobot.py \
  --src_path ~/data \
  --tgt_path ~/lerobot \
  --repo_ids pick_cube \
  --save_repoid pick_cube \
  --cut_head \
  --cut_tail \
  --robot single \
  --start_episode 0 \
  --end_episode 50
```

Options:

- `--src_path`: root directory containing collected task folders, which corresponds to `{dataset_dir}`.
- `--tgt_path`: output root for the converted LeRobot dataset. The dataset is written to `{tgt_path}/{save_repoid}`.
- `--repo_ids`: one or more task names to convert, which correspond to `{task_name}`.
- `--save_repoid`: output dataset name. Defaults to the first `--repo_ids` value.
- `--cut_head` / `--no-cut_head`: trim or keep still frames at the start.
- `--cut_tail` / `--no-cut_tail`: trim or keep still frames at the end.
- `--robot`: robot layout, `dual` or `single`. Use the same layout as during collection.
- `--start_episode` / `--end_episode`: inclusive episode-index range to convert (matched against the `episode_{idx}.hdf5` filename). Omit either bound to leave it open. Applies per `--repo_ids` entry.

### Upload the Dataset to the Hugging Face Hub

Log in once with a Hugging Face token that has write access (`hf auth login`), then push the local LeRobot dataset:

```bash
source ~/venvs/lerobot/bin/activate

python convert_data/upload_to_hub.py \
  --root ~/data/lerobot_convert/pick_cube \
  --repo_id <hf-username>/pick_cube
```

`--root` is the local LeRobot dataset directory (`{dataset_dir}/lerobot_convert/{task_name}` from automatic conversion, or `{tgt_path}/{save_repoid}` from the manual conversion); `--repo_id` is the target Hub dataset repo (`<hf-username>/<dataset-name>`). The repo is created on first push.

Options:

- `--private` / `--no-private`: push as a private or public repo (default: private).
- `--license`: dataset card license (default `apache-2.0`).
- `--push_videos` / `--no-push_videos`: include the `videos/` directory (default: on).
- `--branch`: Hub branch/revision to push to.
- `--tags`: tags added to the generated dataset card.
- `--upload_large_folder`: use `upload_large_folder` instead of `upload_folder`, for very large datasets.

## 🧩 Robot Configs

[`robot_configs.yaml`](robot_configs.yaml) defines the robot layouts selected with the first argument of `start_collect.sh` / `--robot`:

- `arms`: one entry per leader-follower pair, with its `name` and ROS topics. The order of the arms defines the state/action layout: 7 values per arm (6 joints + gripper).
- `cameras`: one entry per camera, with its dataset `name`, `color_topic`, and optional `depth_topic` (RealSense only, used with `--use_depth_image`). The order of the cameras defines their order in the dataset.

| Layout | Arms | Cameras | State/action size |
| --- | --- | --- | --- |
| `dual` | `left`, `right` | `cam_high`, `cam_left_wrist`, `cam_right_wrist` | 14 |
| `single` | `arm` | `cam_side`, `cam_wrist` | 7 |

The arm topics must match the topics published by [`start_leader_follower.launch.py`](piper_ws/src/piper/launch/start_leader_follower.launch.py), and the camera topics must match the camera launch files.

## 📊 Dataset Format

Collected episodes are HDF5 files with the following keys (camera names follow the robot layout: `cam_high`, `cam_left_wrist`, `cam_right_wrist` for `dual`; `cam_side`, `cam_wrist` for `single`):

```text
/observations/images/<camera_name>
/observations/qpos
/observations/qvel
/observations/effort
/observations/eef_pose
/action
/collect
```

Shape conventions:

- RGB images: `(T, H, W, 3)`, `uint8`, with each camera's configured size (`480 x 640` by default).
- Joint state `qpos` (6-DoF + gripper), velocity `qvel`, effort `effort`, end-effector pose `eef_pose` (xyz + rpy + gripper), and action arrays: `(T, 7 x number of arms)`. For `dual` this is `(T, 14)`, left arm 7 dimensions followed by right arm 7 dimensions; for `single` it is `(T, 7)`.
- `/collect`: per-frame string label, for example `teleop` or `rollout`.

## 🙏 Acknowledgements

We thank the following repositories for their references and prior work:

- [piper_ros](https://github.com/agilexrobotics/piper_ros)
- [realsense-ros](https://github.com/realsenseai/realsense-ros)
- [LeRobot](https://github.com/huggingface/lerobot)
- [openpi](https://github.com/Physical-Intelligence/openpi)
- [AgiBot-World](https://github.com/OpenDriveLab/AgiBot-World)
