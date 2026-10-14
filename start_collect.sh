#!/usr/bin/env bash
# One-shot data-collection launcher: starts cameras, arms (teleoperation
# mode 0), and (by default) a live view per camera, then runs
# collect_data.py in the foreground. Replaces the usual 5-terminal flow
# (start_cam.sh + start_mode0.sh + one rqt_image_view per camera +
# collect_data.py) with a single command, similar to `lerobot-record`.
#
# Usage:
#   ./start_collect.sh [dual|single] [--no-view] [--no-lerobot] [--cut_head] [--cut_tail]
#                      [collect_data.py args...]
#
# Every saved episode is also converted to a LeRobot dataset (in the background,
# via the lerobot venv) at <dataset_dir>/lerobot_convert/<task_name>; the HDF5 is
# always kept. --cut_head/--cut_tail trim the still frames at the start/end of
# each converted episode. --no-lerobot disables the conversion. Passing your own
# --lerobot_dir overrides the default location.
#
# Examples:
#   ./start_collect.sh single --dataset_dir ~/data --task_name pick_cube
#   ./start_collect.sh single --cut_head --cut_tail --dataset_dir ~/data --task_name pick_cube
#   ./start_collect.sh single --no-lerobot --dataset_dir ~/data --task_name pick_cube
#   ./start_collect.sh --no-view --dataset_dir ~/data --task_name fold_towel
#
# Prerequisites (same as the manual flow): ROS 2 + the piper workspace
# sourced, the piperaio venv activated, arms powered on. The camera
# view(s) need a Qt binding for rqt_image_view (python3-pyqt5 via apt, or
# `pip install PyQt5` in the active venv) -- without it collect_data.py
# still runs fine, just without a live preview; use --no-view to skip it.
#
# Ctrl+C (or collect_data.py exiting on its own) stops everything this
# script started. `ros2 launch` needs SIGINT specifically to shut its node
# tree down cleanly (see README) -- SIGTERM/plain `kill` can leave arm nodes
# running, so cleanup below sends SIGINT to the two launch processes.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STARTUP_WAIT_SEC=5

ROBOT="single"
SHOW_VIEW=1
LEROBOT=1
HAS_LEROBOT_DIR=0
DATASET_DIR=""
COLLECT_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        dual|single)
            ROBOT="$1"
            ;;
        --no-view)
            SHOW_VIEW=0
            ;;
        --no-lerobot)
            LEROBOT=0
            ;;
        --dataset_dir)
            DATASET_DIR="${2:-}"
            COLLECT_ARGS+=("$1")
            ;;
        --dataset_dir=*)
            DATASET_DIR="${1#*=}"
            COLLECT_ARGS+=("$1")
            ;;
        --lerobot_dir|--lerobot_dir=*)
            HAS_LEROBOT_DIR=1
            COLLECT_ARGS+=("$1")
            ;;
        *)
            COLLECT_ARGS+=("$1")
            ;;
    esac
    shift
done

# Default LeRobot output: <dataset_dir>/lerobot_convert (collect_data.py adds <task_name>)
if [[ "$LEROBOT" -eq 1 && "$HAS_LEROBOT_DIR" -eq 0 ]]; then
    COLLECT_ARGS+=(--lerobot_dir "${DATASET_DIR%/}/lerobot_convert")
fi

LOG_DIR="$(mktemp -d "/tmp/piper_collect.${ROBOT}.XXXXXX")"
echo "Logs: $LOG_DIR"

LAUNCH_PIDS=()   # ros2 launch processes: need SIGINT for a clean node shutdown
VIEW_PIDS=()     # rqt_image_view processes: plain kill is fine

cleanup() {
    trap - EXIT INT TERM
    echo -e "\nStopping camera/arm/view processes..."
    for pid in "${VIEW_PIDS[@]:-}"; do
        [[ -n "$pid" ]] && kill "$pid" 2>/dev/null
    done
    for pid in "${LAUNCH_PIDS[@]:-}"; do
        [[ -n "$pid" ]] && kill -INT "$pid" 2>/dev/null
    done
    # Give ros2 launch a few seconds to shut its nodes down gracefully
    # before falling back to a hard kill.
    for _ in 1 2 3 4 5; do
        alive=0
        for pid in "${LAUNCH_PIDS[@]:-}"; do
            [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null && alive=1
        done
        [[ "$alive" -eq 0 ]] && break
        sleep 1
    done
    for pid in "${LAUNCH_PIDS[@]:-}"; do
        [[ -n "$pid" ]] || continue
        # SIGKILL can't be caught/forwarded, so if ros2 launch itself needs a
        # hard kill here, also reap any children it hadn't shut down yet
        # (otherwise they'd be orphaned and keep running).
        pkill -9 -P "$pid" 2>/dev/null
        kill -9 "$pid" 2>/dev/null
    done
    wait 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "[1/4] Starting cameras ($ROBOT)..."
if [[ "$ROBOT" == "single" ]]; then
    ros2 launch piper cameras.launch.py > "$LOG_DIR/cameras.log" 2>&1 &
else
    ros2 launch piper multi_camera.launch.py \
        serial_no_camera1:="''" \
        serial_no_camera2:="''" \
        serial_no_camera3:="''" \
        > "$LOG_DIR/cameras.log" 2>&1 &
fi
LAUNCH_PIDS+=("$!")

echo "[2/4] Starting arms in teleoperation mode ($ROBOT)..."
ros2 launch piper start_leader_follower.launch.py \
    robot:="$ROBOT" mode:=0 auto_enable:=false \
    > "$LOG_DIR/arms.log" 2>&1 &
LAUNCH_PIDS+=("$!")

echo "Waiting ${STARTUP_WAIT_SEC}s for nodes to come up..."
sleep "$STARTUP_WAIT_SEC"

for pid in "${LAUNCH_PIDS[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "One of the camera/arm launches exited early -- check $LOG_DIR before retrying." >&2
        exit 1
    fi
done

if [[ "$SHOW_VIEW" -eq 1 ]]; then
    echo "[3/4] Starting camera view(s)..."
    if [[ "$ROBOT" == "single" ]]; then
        CAMERA_TOPICS=("/camera_side/color/image_raw" "/camera_wrist/color/image_raw")
    else
        CAMERA_TOPICS=("/camera_f/color/image_raw" "/camera_l/color/image_raw" "/camera_r/color/image_raw")
    fi
    for i in "${!CAMERA_TOPICS[@]}"; do
        ros2 run rqt_image_view rqt_image_view "${CAMERA_TOPICS[$i]}" > "$LOG_DIR/view_$i.log" 2>&1 &
        VIEW_PIDS+=("$!")
    done
    sleep 1
    for i in "${!VIEW_PIDS[@]}"; do
        if ! kill -0 "${VIEW_PIDS[$i]}" 2>/dev/null; then
            echo "Warning: camera view for ${CAMERA_TOPICS[$i]} exited immediately -- see $LOG_DIR/view_$i.log" \
                "(often a missing Qt binding: pip install PyQt5, or sudo apt install python3-pyqt5)." >&2
        fi
    done
else
    echo "[3/4] Skipping camera view (--no-view)"
fi

echo "[4/4] Starting collect_data.py..."
echo
python3 "$SCRIPT_DIR/collect_data/collect_data.py" --robot "$ROBOT" "${COLLECT_ARGS[@]}"
