<p align="center">
  <h1 align="center">Piper All In One</h1>
</p>

Piper All In One is an imitation-learning stack for [AgileX Piper](https://global.agilex.ai/products/piper) robotic arms and [Cobot Magic](https://global.agilex.ai/products/cobot-magic) systems. It covers the full robot-learning loop: hardware bring-up, teleoperated data collection, data replay, LeRobot dataset conversion, and policy inference.

> ‼️ Safety first: this repository controls real robot arms. Keep the emergency stop reachable, start from a clear workspace, verify CAN/camera mappings before motion, and test every new policy with slow, supervised motions before normal operation.

## 📰 News

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
  - [Choose the Robot Layout](#choose-the-robot-layout)
  - [Start Cameras](#start-cameras)
  - [Start Arms](#start-arms)
  - [Collect Data](#collect-data)
  - [Visualize Data](#visualize-data)
  - [Replay Data](#replay-data)
  - [Convert Data to LeRobot](#convert-data-to-lerobot)
  - [Upload the Dataset to the Hugging Face Hub](#upload-the-dataset-to-the-hugging-face-hub)
  - [Run Policy Inference](#run-policy-inference)
    - [Synchronous Inference](#synchronous-inference)
    - [Asynchronous Inference](#asynchronous-inference)
- [🧩 Configuration Reference](#-configuration-reference)
  - [Robot Configs](#robot-configs)
  - [Task Configs](#task-configs)
  - [Policy Clients](#policy-clients)
- [📊 Dataset Format](#-dataset-format)
- [🙏 Acknowledgements](#-acknowledgements)

## 🗂️ Repository Layout

```text
.
├── piper_ws/                          # ROS 2 workspace for Piper messages, arm node, and arm/camera launch files
├── collect_data/                      # Data collection, replay, and visualization scripts
├── convert_data/                      # HDF5 filtering and LeRobot conversion scripts
├── inference/                         # Policy clients and sync/async inference runners
├── lerobot/                           # LeRobot submodule used for dataset conversion
├── robot_configs.yaml                 # Robot layouts (arms, cameras, topics): dual and single
├── robot_config.py                    # Loader for robot_configs.yaml
```

## ⚙️ Setup

### Prerequisites

#### Hardware

Two robot layouts are supported. Select one with `--robot dual` (default) or `--robot single` (see [Choose the Robot Layout](#choose-the-robot-layout)).

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
git clone --recurse-submodules https://github.com/innovator-zero/piper-aio.git
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
- For the single-arm layout, set `EXPECTED_CAN_COUNT=1`. The single module is then named `can0` at 1 Mbps, which is what `./start_mode0.sh single` and `./start_mode1.sh single` use by default (override with `can_port:=<name>`).

After each machine startup, run the CAN configuration script once to rename and bring up the CAN interfaces:

```bash
bash piper_ws/scripts/can_config.sh
```

#### Create Python Environments

Create the main environment used by ROS control, collection, replay, and inference. ROS 2 Python packages (`rclpy`, message types) live in the system Python under `/opt/ros/jazzy`, so the environment must be a Python 3.12 virtual environment with access to system site packages. Conda environments with other Python versions cannot import `rclpy`.

```bash
python3 -m venv --system-site-packages ~/venvs/piperaio
source ~/venvs/piperaio/bin/activate

pip install python-can piper_sdk
pip install git+https://github.com/agilexrobotics/pyAgxArm.git
pip install h5py opencv-python matplotlib dm-env pyyaml
```

`pyyaml` is also used to read [`robot_configs.yaml`](robot_configs.yaml). The arm node uses [`pyAgxArm`](https://github.com/agilexrobotics/pyAgxArm) to talk to the Piper CAN interface; it is not on PyPI, so it installs directly from its GitHub repo (`piper_sdk` stays installed too, in case you need to roll back).

Install the [openpi client](https://github.com/Physical-Intelligence/openpi/tree/main/packages/openpi-client) in the same environment if you run policy inference.

If you plan to convert collected data to the [LeRobot](https://github.com/huggingface/lerobot) dataset format, create a separate environment for LeRobot (v0.6.1, dataset `v3.0`):

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

- Every camera publishes `/<name>/color/image_raw` regardless of its type, so collection and inference do not depend on the camera type. Keep the names `camera_side` and `camera_wrist`, or update the `single` topics in [`robot_configs.yaml`](robot_configs.yaml) to match.
- Prefer the stable `/dev/v4l/by-id/...-video-index0` paths over `/dev/videoN`, which can change between boots. RealSense cameras also appear as V4L2 devices; configure them as `type: realsense`.
- For USB cameras, `width`/`height` must be a size the camera supports (`v4l2-ctl -d <video_device> --list-formats-ext`). The frame rate is the camera default for that size, typically 30 fps. Some webcams lower their frame rate in dim light when auto exposure is on.
- The workspace is built with `--symlink-install`, so edits to the YAML file apply without rebuilding. You can also pass another file with `ros2 launch piper cameras.launch.py config:=/path/to/cameras.yaml`.

## 🚀 Usage

Run each long-running command in a separate terminal. Source ROS 2 and the Piper workspace, and activate the `piperaio` environment (`source ~/venvs/piperaio/bin/activate`) where needed. Stop launch files with `Ctrl+C`; killing `ros2 launch` with `SIGTERM` (for example with `kill` or `timeout`) can leave the arm nodes running.

### Choose the Robot Layout

Every step below works for both layouts. The dual-arm layout is the default; for the single-arm layout:

- pass `single` to the start scripts: `./start_cam.sh single`, `./start_mode0.sh single`, `./start_mode1.sh single`;
- pass `--robot single` to `collect_data.py`, `replay_data.py`, `convert_to_lerobot.py`, `infer_sync.py`, and `infer_async.py`.

`visualize_episodes.py` and `dataset_filter.py` work on both layouts without extra options.

Single-arm quick reference:

```bash
./start_cam.sh single
./start_mode0.sh single    # teleoperation, for data collection
python collect_data/collect_data.py --robot single --dataset_dir ~/data --task_name pick_cube

./start_mode1.sh single    # follower control, for replay and inference
python collect_data/replay_data.py --robot single --dataset_dir ~/data --task_name pick_cube --episode_idx 0
python inference/infer_sync.py --robot single --task pick_cube --host 127.0.0.1 --port 8000
```

### Start Cameras

```bash
./start_cam.sh           # dual-arm: three RealSense cameras
./start_cam.sh single    # single-arm: side and wrist cameras from cameras_single.yaml
```

Use `ros2 run rqt_image_view rqt_image_view` to inspect the camera streams.

### Start Arms

Use mode 0 for leader-follower teleoperation during data collection:

```bash
./start_mode0.sh           # dual-arm: can_left and can_right
./start_mode0.sh single    # single-arm: can0 (or ./start_mode0.sh single can_port:=<name>)
```

Use mode 1 when controlling the follower arms during replay or inference, and wait until every follower arm reports `Enable Status: True`:

```bash
./start_mode1.sh           # or: ./start_mode1.sh single
```

Both scripts pass extra arguments through to the launch file, so override the [`pyAgxArm`](https://github.com/agilexrobotics/pyAgxArm) firmware profile (default `v189`, i.e. Piper main-controller firmware >= S-V1.8-9) if your arms are on older firmware: `./start_mode0.sh single firmware_version:=default` (also accepts `v183`, `v188`).

### Collect Data

#### One-shot: `start_collect.sh`

`start_collect.sh` starts cameras, arms (mode 0), a live view per camera, and `collect_data.py` with a single command — the manual 5-terminal flow below (cameras + arms + one viewer per camera + the collection script) collapsed into one, closer to `lerobot-record`'s workflow. Power on all arms first, then (with ROS 2 + the workspace sourced and the `piperaio` venv activated):

```bash
./start_collect.sh --dataset_dir ~/data --task_name test
./start_collect.sh single --dataset_dir ~/data --task_name pick_cube   # single-arm layout
./start_collect.sh --no-view --dataset_dir ~/data --task_name test     # skip the camera viewers (e.g. over SSH with no X11)
```

The first argument is the robot layout (`dual`, the default, or `single`); everything else is forwarded to `collect_data.py` (see its keyboard controls below). Ctrl+C, or `collect_data.py` exiting on its own, stops every process the script started. Logs for the camera/arm launches and viewers are kept under `/tmp/piper_collect.<robot>.*` for troubleshooting if something doesn't come up.

#### Manual (one terminal per step)

1. Power on all arms.
2. Start cameras with `./start_cam.sh` (or `./start_cam.sh single`).
3. Start arms in mode 0 with `./start_mode0.sh` (or `./start_mode0.sh single`).
4. Start the collection script (add `--robot single` for the single-arm layout):

```bash
source ~/venvs/piperaio/bin/activate

python collect_data/collect_data.py --dataset_dir ~/data --task_name test
```

Keyboard controls:

- `ENTER`: start recording an episode.
- `SPACE`: stop the current recording.
- `→` (right arrow): save the episode and continue to the next one (`s` also still works).
- `←` (left arrow): discard the episode and restart it (`q` also still works).
- `Ctrl+C`: exit the collection script.

Episodes are saved to `{dataset_dir}/{task_name}/episode_{episode_idx}.hdf5`.

When `--episode_idx` is omitted or set to `0`, the script automatically uses the next available episode index in the save directory.

Note: the system may occasionally drop frames and report `syn fail`, so avoid collecting data too quickly, especially during complex motions. Also check USB bandwidth and verify all camera and arm topics are publishing at the expected rate.

### Visualize Data

```bash
source ~/venvs/piperaio/bin/activate

python collect_data/visualize_episodes.py --dataset_dir ~/data --task_name test --episode_idx 0
```

The visualization script plays and saves camera videos, joint plots, and end-effector pose plots for the selected episode.

### Replay Data

1. Power on only the follower arms.
2. Start cameras with `./start_cam.sh` (or `./start_cam.sh single`).
3. Start arms in mode 1 with `./start_mode1.sh` (or `./start_mode1.sh single`).
4. Replay an episode (add `--robot single` for the single-arm layout):

```bash
source ~/venvs/piperaio/bin/activate

python collect_data/replay_data.py --dataset_dir ~/data --task_name test --episode_idx 0 --replay_mode joint
```

`replay_mode` can be either `joint` or `eef`. The script reads the recorded data and sends control commands to the control topics.

### Convert Data to LeRobot

First inspect collected episodes for outliers:

```bash
source ~/venvs/lerobot/bin/activate

python convert_data/dataset_filter.py --root_dir ~/data/test --std_threshold 5.0
```

`--root_dir` should point to `{dataset_dir}/{task_name}`. The script analyzes `observations/qpos` and `action`, then reports files with values outside the standard-deviation threshold configured by `--std_threshold`.

Before conversion, edit [`convert_data/convert_to_lerobot.py`](convert_data/convert_to_lerobot.py):

- Set `INSTRUCTION` near the top of the file to the language instruction used for training.
- Add any rejected HDF5 episode paths to `exclude_files` in `main()`.

Convert one or more task directories to a LeRobot v3.0 dataset using the feature schema in [`convert_data/features.py`](convert_data/features.py). The cameras and state size follow the `--robot` layout, and each camera's image size is read from the recorded episodes:

```bash
source ~/venvs/lerobot/bin/activate

python convert_data/convert_to_lerobot.py \
  --src_path ~/data \
  --tgt_path ~/lerobot \
  --repo_ids test \
  --save_repoid test \
  --cut_head \
  --cut_tail \
  --robot single \
  --start_episode 0 \
  --end_episode 50
```

Useful options:

- `--src_path`: root directory containing collected task folders, which corresponds to `{dataset_dir}`.
- `--tgt_path`: output root for the converted LeRobot dataset. The dataset is written to `{tgt_path}/{save_repoid}`.
- `--repo_ids`: one or more task names to convert, which correspond to `{task_name}`.
- `--save_repoid`: output dataset name. Defaults to the first `--repo_ids` value.
- `--cut_head` / `--no-cut_head`: trim or keep still frames at the start.
- `--cut_tail` / `--no-cut_tail`: trim or keep still frames at the end.
- `--robot`: robot layout, `dual` (default) or `single`.
- `--start_episode` / `--end_episode`: inclusive episode-index range to convert (matched against the `episode_{idx}.hdf5` filename), e.g. `--start_episode 0 --end_episode 50` converts only episodes 0-50 out of a larger recorded set. Omit either bound to leave it open. Applies per `--repo_ids` entry.

### Upload the Dataset to the Hugging Face Hub

Log in once with a Hugging Face token that has write access (`huggingface-cli login`), then push the local LeRobot dataset produced above:

```bash
source ~/venvs/lerobot/bin/activate

python convert_data/upload_to_hub.py \
  --root ~/lerobot/test \
  --repo_id <hf-username>/test
```

`--root` is the local dataset directory from the conversion step (`{tgt_path}/{save_repoid}`); `--repo_id` is the target Hub dataset repo (`<hf-username>/<dataset-name>`). The repo is created on first push.

Useful options:

- `--private` / `--no-private`: push as a private or public repo (default: private).
- `--license`: dataset card license (default `apache-2.0`).
- `--push_videos` / `--no-push_videos`: include the `videos/` directory (default: on).
- `--branch`: Hub branch/revision to push to.
- `--tags`: tags added to the generated dataset card.
- `--upload_large_folder`: use `upload_large_folder` instead of `upload_folder`, for very large datasets.

### Run Policy Inference

Policy inference uses task settings from [`inference/task_configs.yaml`](inference/task_configs.yaml) and client code from [`inference/clients.py`](inference/clients.py). Refer to [Configuration Reference](#-configuration-reference) on how to add new tasks and clients.

We provide two inference runners:

- Synchronous inference in [`inference/infer_sync.py`](inference/infer_sync.py): standard practice for action-chunking policy. The robot executes one action chunk and requests the next chunk only after the current chunk has been consumed. During policy inference, the robot controller pauses and resumes only when the new actions arrive.
- Asynchronous inference in [`inference/infer_async.py`](inference/infer_async.py): initiates inference for the next chunk before the current chunk is fully executed. Once the next inference request is triggered, the robot continues executing the remaining actions in the ongoing chunk. Before the final action is completed, the newly predicted chunk is expected to be available, enabling seamless execution without halting. Useful for real-time methods such as [Training-time RTC](https://arxiv.org/abs/2512.05964) and [FASTER](https://github.com/innovator-zero/FASTER).

> Refer to our [FASTER](https://arxiv.org/abs/2603.19199) paper for a clear understanding of the sync and async inference pipelines.

Start the robot in follower-control mode:

```bash
./start_cam.sh      # or: ./start_cam.sh single
./start_mode1.sh    # or: ./start_mode1.sh single
```

For the single-arm layout, also pass `--robot single` and a single-arm task (such as `pick_cube`) to the inference runners.

Start your policy server on the same machine or on a LAN-connected workstation. For remote inference, prefer wired LAN to reduce latency and packet loss.

#### Synchronous Inference

```bash
source ~/venvs/piperaio/bin/activate

python inference/infer_sync.py \
  --task towel \
  --model openpi \
  --host 127.0.0.1 \
  --port 8000 \
  --ctrl_type joint
```

Useful options:

- `--task`: task key from `inference/task_configs.yaml`.
- `--model`: policy client type. Currently `openpi` is supported.
- `--host`: policy server host.
- `--port`: policy server port.
- `--ctrl_type`: `joint` or `eef`.
- `--chunk_size`: number of actions expected per inference call, default `50`. If it is set to be smaller than the action chunk size generated from policy, then only the first `chunk_size` actions will be executed.
- `--save_rollout`: save rollout observations and executed actions to HDF5.
- `--save_dir`: directory for rollout HDF5 files when `--save_rollout` is set.

#### Asynchronous Inference

```bash
source ~/venvs/piperaio/bin/activate

python inference/infer_async.py \
  --task towel \
  --model openpi \
  --host 127.0.0.1 \
  --port 8000 \
  --ctrl_type joint \
  --mode rtc \
  --delay 4 \
  --exec_horizon 25
```

Additional async options:

- `--mode`: async inference mode. Use `rtc` for RTC-style inference (using action prefix), or `naive` for the naive asynchronous mode (such as [SmolVLA](https://arxiv.org/abs/2506.01844)).
- `--delay`: inference delay $d:= \lfloor \Delta t_\text{infer}/\Delta t_{\text{ctrl}}\rfloor$, determined by the inference latency $\Delta t_\text{infer}$ and control period $\Delta t_{\text{ctrl}}$. We recommend setting it one step larger to account for variation in inference time and transmission latency.
- `--exec_horizon`: execution horizon $s$ for the action chunk. The client executes only the first $s$ valid actions (excluding delayed ones), then triggers a new inference request. This value should be larger than or equal to $d$.
- `--streaming`: supports the *Streaming Client-Server Interface* proposed in [FASTER](https://github.com/innovator-zero/FASTER).

During inference, press `SPACE` to enter interactive mode:

- `→` (right arrow): continue.
- `←` (left arrow): reset to the initial arm positions and restart.
- `q`: save the current rollout if enabled, then quit.

## 🧩 Configuration Reference

### Robot Configs

[`robot_configs.yaml`](robot_configs.yaml) defines the robot layouts selected with `--robot`:

- `arms`: one entry per leader-follower pair, with its `name` and ROS topics. The order of the arms defines the state/action layout: 7 values per arm (6 joints + gripper).
- `cameras`: one entry per camera, with its dataset `name`, `color_topic`, and optional `depth_topic` (RealSense only, used with `--use_depth_image`). The order of the cameras defines the order of the images sent to the policy.

| Layout | Arms | Cameras | State/action size |
| --- | --- | --- | --- |
| `dual` | `left`, `right` | `cam_high`, `cam_left_wrist`, `cam_right_wrist` | 14 |
| `single` | `arm` | `cam_side`, `cam_wrist` | 7 |

The arm topics must match the topics published by [`start_leader_follower.launch.py`](piper_ws/src/piper/launch/start_leader_follower.launch.py), and the camera topics must match the camera launch files.

### Task Configs

Each top-level task in [`inference/task_configs.yaml`](inference/task_configs.yaml) must define:

```yaml
towel:
  language_instruction: "Fold the towel."
  left0: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1] # or *default_left0
  right0: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1] # or *default_right0
  action_postprocess:
    left_gripper:
      - when: below
        threshold: 0.02
        set: 0.0
    right_gripper:
      - when: below
        threshold: 0.02
        set: 0.0
```

Single-arm tasks use the arm name `arm` from [`robot_configs.yaml`](robot_configs.yaml):

```yaml
pick_cube:
  robot: single
  language_instruction: "Pick up the cube."
  arm0: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
  action_postprocess:
    arm_gripper:
      - when: below
        threshold: 0.02
        set: 0.0
```

Required fields:

- `language_instruction`: instruction sent to the policy. Keep this consistent with the instruction used during dataset conversion and training.
- `<arm>0` for every arm of the robot layout: initial joint positions of the follower arm (7 values), i.e. `left0` and `right0` for `dual`, `arm0` for `single`.

Optional fields:

- `robot`: the layout this task is for. The inference scripts refuse to run the task with a different `--robot`.
- `action_postprocess`: gripper post-processing rules for model outputs, under `<arm>_gripper` (e.g. `left_gripper`, `right_gripper`, or `arm_gripper`).
  - `when`: `below` or `above`.
  - `threshold`: condition threshold.
  - `set`: replacement value when the condition is true.
  - `add`: offset added when the condition is true.

### Policy Clients

The default `OpenpiClient` in [`inference/clients.py`](inference/clients.py) uses the [openpi WebSocket client](https://github.com/Physical-Intelligence/openpi/tree/main/packages/openpi-client). To add a new client:

1. Implement a client class in `inference/clients.py`.
2. Match the constructor and public methods used by the inference scripts:

```python
Client(host, port, camera_names, state_dim)
predict_action(payload)
warmup()
```

`payload` contains `images` (camera name to `(H, W, 3)` RGB image, in robot camera order), `state` (`(state_dim,)` joint or end-effector state), `instruction`, and for asynchronous RTC inference `action_prefix` and `delay`. `predict_action` returns an action chunk of shape `(chunk, state_dim)`.

3. Import and select the new client in [`inference/infer_sync.py`](inference/infer_sync.py) or [`inference/infer_async.py`](inference/infer_async.py). Also add the new client name to the `--model` choices.
4. Install any dependencies in the `piperaio` environment.

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
