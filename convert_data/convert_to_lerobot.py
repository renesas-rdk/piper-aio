import os
import re
import sys
import shutil
import tempfile
from datetime import datetime
import h5py
import numpy as np
from pathlib import Path
from tqdm import tqdm
from contextlib import contextmanager
from typing import List, Optional

from features import build_features
from lerobot.datasets.lerobot_dataset import LeRobotDataset

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from robot_config import DEFAULT_ROBOT, ROBOT_CONFIGS, get_robot_config

INSTRUCTION = "Example"
DEFAULT_COLLECT = "teleop"

EPISODE_INDEX_PATTERN = re.compile(r"episode_(\d+)\.hdf5$")


def extract_episode_index(episode_path: str | Path) -> int:
    """Episode index parsed from the `episode_{idx}.hdf5` filename."""
    match = EPISODE_INDEX_PATTERN.search(Path(episode_path).name)
    if match is None:
        raise ValueError(f"Could not parse episode index from filename: {episode_path}")
    return int(match.group(1))


def _decode_collect_value(value) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8")
    return str(value)


@contextmanager
def suppress_stderr_on_success():
    """Hide noisy native stderr output from video encoding, but replay it if an error occurs."""
    stderr_fd = sys.stderr.fileno()
    saved_stderr_fd = os.dup(stderr_fd)
    tmp = tempfile.TemporaryFile(mode="w+b")

    try:
        os.dup2(tmp.fileno(), stderr_fd)
        try:
            yield
        except Exception:
            os.dup2(saved_stderr_fd, stderr_fd)
            tmp.seek(0)
            stderr_output = tmp.read().decode("utf-8", errors="replace").strip()
            if stderr_output:
                print("Captured ffmpeg/libav stderr:", file=sys.stderr)
                print(stderr_output, file=sys.stderr)
            raise
    finally:
        os.dup2(saved_stderr_fd, stderr_fd)
        os.close(saved_stderr_fd)
        tmp.close()


def read_camera_shapes(episode_path: str | Path, camera_names: List[str]) -> dict:
    """Per-camera (height, width, channel) image shapes of an episode."""
    with h5py.File(episode_path) as f:
        return {name: tuple(f[f"observations/images/{name}"].shape[1:]) for name in camera_names}


def load_hdf5_dataset(
    episode_path: str | Path,
    camera_names: List[str],
    state_dim: int,
    cut_head: bool = False,
    cut_tail: bool = False,
) -> tuple[list, dict]:
    """Load hdf5 dataset and return per-frame observations with aligned actions."""

    with h5py.File(episode_path) as f:
        missing_cameras = [name for name in camera_names if f"observations/images/{name}" not in f]
        if missing_cameras:
            raise ValueError(f"Missing cameras {missing_cameras}; check --robot.")
        state_images = {name: np.array(f[f"observations/images/{name}"]) for name in camera_names}
        state_qpos = np.array(f["observations/qpos"], dtype=np.float32)
        state_eef_pose = np.array(f["observations/eef_pose"], dtype=np.float32)
        actions = np.array(f["action"], dtype=np.float32)
        if "collect" in f:
            collect = [_decode_collect_value(value) for value in np.array(f["collect"])]
        else:
            collect = [DEFAULT_COLLECT] * actions.shape[0]

    lengths = {images.shape[0] for images in state_images.values()}
    lengths.update({state_qpos.shape[0], state_eef_pose.shape[0], actions.shape[0], len(collect)})
    if len(lengths) != 1:
        raise ValueError("Mismatch in dataset lengths.")

    if state_qpos.shape[1] != state_dim:
        raise ValueError(f"Expected qpos to have {state_dim} dimensions, but got {state_qpos.shape[1]}.")

    if state_eef_pose.shape[1] != state_dim:
        raise ValueError(f"Expected eef_pose to have {state_dim} dimensions, but got {state_eef_pose.shape[1]}.")

    if actions.shape[1] != state_dim:
        raise ValueError(f"Expected actions to have {state_dim} dimensions, but got {actions.shape[1]}.")

    # [Optional] We skip the first few still steps
    EPS = 5e-3
    # Get the idx of the first qpos whose delta exceeds the threshold
    if cut_head:
        qpos_delta = np.abs(state_qpos - state_qpos[0:1])
        indices = np.where(np.any(qpos_delta > EPS, axis=1))[0]
        if len(indices) > 0:
            first_idx = indices[0]
        else:
            raise ValueError("Found no qpos that exceeds the threshold.")
    else:
        first_idx = 1

    if cut_tail:
        qpos_delta = np.abs(state_qpos - state_qpos[-1:])
        indices = np.where(np.any(qpos_delta > EPS, axis=1))[0]
        if len(indices) > 0:
            last_idx = indices[-1]
        else:
            raise ValueError("Found no qpos that exceeds the threshold.")
    else:
        last_idx = state_qpos.shape[0] - 1

    st = first_idx - 1
    end = min(last_idx, state_qpos.shape[0] - 1)

    if end <= st:
        raise ValueError("Need at least 2 frames after trimming to build next-frame eef actions.")

    frames = [
        {
            "observation.state": state_qpos[i].reshape(-1),
            "observation.eef_pose": state_eef_pose[i].reshape(-1),
            **{f"observation.images.{name}": images[i] for name, images in state_images.items()},
            "action": actions[i].reshape(-1),
            "eef_action": state_eef_pose[i + 1].reshape(-1),
            "collect": collect[i],
        }
        for i in range(st, end)
    ]

    cut_info = {
        "total_frames": int(state_qpos.shape[0]),
        "start_index": int(st),
        "end_index_inclusive": int(end),
        "kept_frames": int(len(frames)),
        "cut_head_frames": int(st),
        "cut_tail_frames": int(state_qpos.shape[0] - 1 - end),
        "cut_head_enabled": cut_head,
        "cut_tail_enabled": cut_tail,
    }

    return frames, cut_info


def open_lerobot_dataset(target_dir: str, repo_id: str, camera_shapes: dict, state_dim: int, fps: int = 30):
    """Resume the LeRobot dataset at target_dir if it exists, otherwise create it."""
    kwargs = dict(
        image_writer_processes=10,
        image_writer_threads=5,
        video_backend="torchcodec",
    )
    if os.path.exists(os.path.join(target_dir, "meta", "info.json")):
        return LeRobotDataset.resume(repo_id=repo_id, root=target_dir, **kwargs)
    return LeRobotDataset.create(
        repo_id=repo_id,
        root=target_dir,
        fps=fps,
        robot_type="agilex",
        features=build_features(camera_shapes, state_dim),
        **kwargs,
    )


def convert_episode(
    dataset,
    hdf5_file: str | Path,
    robot_config,
    cut_head: bool = False,
    cut_tail: bool = False,
    task: str = INSTRUCTION,
) -> dict:
    """Append one hdf5 episode to the dataset and save it. Returns the trimming info."""
    try:
        frames, cut_info = load_hdf5_dataset(
            hdf5_file,
            camera_names=robot_config.camera_names,
            state_dim=robot_config.state_dim,
            cut_head=cut_head,
            cut_tail=cut_tail,
        )
        for frame in frames:
            frame["task"] = task
            dataset.add_frame(frame)
        with suppress_stderr_on_success():
            dataset.save_episode()
    except Exception:
        # Drop any partially buffered frames so they don't leak into the next episode
        dataset.clear_episode_buffer()
        raise
    return cut_info


def main(
    src_path: str,
    tgt_path: str,
    repo_ids: List[str],
    save_repoid: str,
    cut_head: bool = False,
    cut_tail: bool = False,
    robot: str = DEFAULT_ROBOT,
    start_episode: Optional[int] = None,
    end_episode: Optional[int] = None,
):
    robot_config = get_robot_config(robot)
    target_dir = f"{tgt_path}/{save_repoid}"
    error_log_path = Path(f"error_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")
    if os.path.exists(target_dir):
        print(f"[Init] Target folder exists, deleting: {target_dir}")
        shutil.rmtree(target_dir)
        print(f"[Init] Deleted: {target_dir}")

    exclude_files = [
        # list of hdf5 files to exclude
    ]

    repo_files = {}
    for repo_id in repo_ids:
        hdf5_source_path = os.path.join(src_path, repo_id)
        if not os.path.exists(hdf5_source_path):
            print(f"[Skip] Source folder not found: {hdf5_source_path}")
            continue
        hdf5_files = sorted(
            (f.as_posix() for f in Path(hdf5_source_path).glob("*.hdf5")),
            key=extract_episode_index,
        )
        hdf5_files = [f for f in hdf5_files if f not in exclude_files]
        if start_episode is not None or end_episode is not None:
            lo = start_episode if start_episode is not None else 0
            hi = end_episode if end_episode is not None else float("inf")
            hdf5_files = [f for f in hdf5_files if lo <= extract_episode_index(f) <= hi]
            print(f"[Init] {repo_id}: filtering to episode range [{lo}, {hi}] -> {len(hdf5_files)} files")
        repo_files[repo_id] = hdf5_files

    all_files = [f for files in repo_files.values() for f in files]
    if not all_files:
        print("[Done] No episode files found")
        return

    # Video feature shapes follow the recorded image sizes of each camera
    camera_shapes = read_camera_shapes(all_files[0], robot_config.camera_names)
    print(f"[Init] Robot: {robot_config.name}, state_dim={robot_config.state_dim}, cameras={camera_shapes}")
    print(f"[Init] Creating LeRobot dataset at: {target_dir}")
    print(f"[Init] Error log file: {error_log_path}")

    dataset = open_lerobot_dataset(target_dir, save_repoid, camera_shapes, robot_config.state_dim)

    for repo_id, hdf5_files in repo_files.items():
        repo_total_frames = 0
        repo_kept_frames = 0
        repo_head_cut_frames = 0
        repo_tail_cut_frames = 0
        repo_success_episodes = 0

        print(f"[Repo] {repo_id}: {len(hdf5_files)} episode files to process")

        for hdf5_file in tqdm(hdf5_files, total=len(hdf5_files), desc="Processing episodes"):
            try:
                cut_info = convert_episode(dataset, hdf5_file, robot_config, cut_head, cut_tail)

                print(
                    "[Episode] "
                    f"{Path(hdf5_file).name}: "
                    f"head_cut={cut_info['cut_head_frames']} "
                    f"tail_cut={cut_info['cut_tail_frames']} "
                    f"kept={cut_info['kept_frames']}/{cut_info['total_frames']} "
                    f"(range: {cut_info['start_index']}:{cut_info['end_index_inclusive']} inclusive, "
                    f"cut_head={cut_info['cut_head_enabled']}, "
                    f"cut_tail={cut_info['cut_tail_enabled']})"
                )
                repo_total_frames += cut_info["total_frames"]
                repo_kept_frames += cut_info["kept_frames"]
                repo_head_cut_frames += cut_info["cut_head_frames"]
                repo_tail_cut_frames += cut_info["cut_tail_frames"]
                repo_success_episodes += 1
            except Exception as e:
                print(f"[Error] Failed to process episode: {hdf5_file}")
                print(f"[Error] {e}")
                with open(error_log_path, "a") as f:
                    f.write(f"Error processing episode {hdf5_file}: {e}\n")

        if repo_success_episodes > 0:
            avg_kept_frames = repo_kept_frames / repo_success_episodes
            print(
                "[Repo Summary] "
                f"{repo_id}: "
                f"episodes={repo_success_episodes}/{len(hdf5_files)} "
                f"head_cut_total={repo_head_cut_frames} "
                f"tail_cut_total={repo_tail_cut_frames} "
                f"kept_total={repo_kept_frames}/{repo_total_frames} "
                f"avg_kept={avg_kept_frames:.1f}"
            )
        else:
            print(f"[Repo Summary] {repo_id}: no episode converted successfully")

    # LeRobot v3.0 datasets must be finalized to flush parquet/metadata writers
    dataset.finalize()
    print("[Done] Conversion finished")


def serve_stdin(target_dir: str, repo_id: str, robot: str, cut_head: bool, cut_tail: bool):
    """Convert hdf5 episode paths read from stdin (one per line) into the dataset at target_dir.

    Appends to the dataset if it exists. Finalizes on EOF, so closing stdin (or the parent
    dying) still leaves a valid dataset. Used by collect_data.py --lerobot_dir.
    """
    robot_config = get_robot_config(robot)
    dataset = None
    failed = []
    for line in sys.stdin:
        path = line.strip()
        if not path:
            continue
        try:
            if dataset is None:
                shapes = read_camera_shapes(path, robot_config.camera_names)
                dataset = open_lerobot_dataset(target_dir, repo_id, shapes, robot_config.state_dim)
            info = convert_episode(dataset, path, robot_config, cut_head, cut_tail)
            print(f"[LeRobot] {Path(path).name} converted ({info['kept_frames']}/{info['total_frames']} frames)", flush=True)
        except Exception as e:
            failed.append(path)
            print(f"[LeRobot] Failed to convert {path}: {e}", flush=True)
    if dataset is not None:
        dataset.finalize()
        print(f"[LeRobot] Dataset finalized: {target_dir}", flush=True)
    if failed:
        print(f"[LeRobot] {len(failed)} episode(s) failed; re-run convert_to_lerobot.py for: {failed}", flush=True)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--src_path",
        type=str,
        required=False,
        default="",
        help="src path for the original dataset",
    )
    parser.add_argument(
        "--tgt_path",
        type=str,
        required=False,
        default="",
        help="tgt path to save the converted dataset",
    )
    parser.add_argument(
        "--stdin",
        action="store_true",
        help="worker mode: read hdf5 paths from stdin and append them to <tgt_path>/<save_repoid> (no repo_ids needed)",
    )
    parser.add_argument("--repo_ids", nargs="+", default=None, help="repo ids")
    parser.add_argument(
        "--save_repoid", type=str, default=None, help="save repoid, default as the first repo_id in the list"
    )
    parser.add_argument(
        "--cut_head",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="cut head of the dataset",
    )
    parser.add_argument(
        "--cut_tail",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="cut tail of the dataset",
    )
    parser.add_argument(
        "--robot",
        type=str,
        choices=list(ROBOT_CONFIGS),
        default=DEFAULT_ROBOT,
        help="robot layout (arms and cameras) from robot_configs.yaml",
    )
    parser.add_argument(
        "--start_episode",
        type=int,
        default=None,
        help="first episode index to convert, inclusive (matches episode_{idx}.hdf5); default: no lower bound",
    )
    parser.add_argument(
        "--end_episode",
        type=int,
        default=None,
        help="last episode index to convert, inclusive (matches episode_{idx}.hdf5); default: no upper bound",
    )
    args = parser.parse_args()
    if args.stdin:
        if not args.save_repoid:
            parser.error("--stdin requires --save_repoid")
        serve_stdin(
            f"{args.tgt_path}/{args.save_repoid}", args.save_repoid, args.robot, args.cut_head, args.cut_tail
        )
        sys.exit(0)
    if not args.repo_ids:
        parser.error("--repo_ids is required")
    repo_ids = args.repo_ids
    save_repoid = args.save_repoid
    if save_repoid is None:
        save_repoid = repo_ids[0]

    main(
        src_path=args.src_path,
        tgt_path=args.tgt_path,
        repo_ids=repo_ids,
        save_repoid=save_repoid,
        cut_head=args.cut_head,
        cut_tail=args.cut_tail,
        robot=args.robot,
        start_episode=args.start_episode,
        end_episode=args.end_episode,
    )
