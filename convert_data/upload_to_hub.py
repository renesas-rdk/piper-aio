import argparse
from pathlib import Path

from lerobot.datasets.lerobot_dataset import LeRobotDataset


def main(
    root: str,
    repo_id: str,
    private: bool = True,
    license: str = "apache-2.0",
    push_videos: bool = True,
    branch: str | None = None,
    tags: list[str] | None = None,
    upload_large_folder: bool = False,
):
    root_path = Path(root)
    if not root_path.exists():
        raise FileNotFoundError(f"Local dataset not found: {root_path}")

    print(f"[Init] Loading local LeRobot dataset from: {root_path}")
    dataset = LeRobotDataset(repo_id, root=root_path)
    print(
        f"[Init] Loaded '{repo_id}': "
        f"{dataset.meta.total_episodes} episodes, {dataset.meta.total_frames} frames"
    )

    print(
        f"[Push] Uploading to https://huggingface.co/datasets/{repo_id} "
        f"(private={private}, push_videos={push_videos})"
    )
    dataset.push_to_hub(
        branch=branch,
        tags=tags,
        license=license,
        push_videos=push_videos,
        private=private,
        upload_large_folder=upload_large_folder,
    )
    print("[Done] Upload finished")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--root",
        type=str,
        required=True,
        help="local LeRobot dataset dir to upload, e.g. {tgt_path}/{save_repoid} from convert_to_lerobot.py",
    )
    parser.add_argument(
        "--repo_id",
        type=str,
        required=True,
        help="hub dataset repo id to push to, e.g. <hf-username>/<dataset-name>",
    )
    parser.add_argument(
        "--private",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="create/push as a private hub repo (default: private)",
    )
    parser.add_argument(
        "--license",
        type=str,
        default="apache-2.0",
        help="license identifier for the generated dataset card",
    )
    parser.add_argument(
        "--push_videos",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="also upload the videos/ directory",
    )
    parser.add_argument(
        "--branch",
        type=str,
        default=None,
        help="hub branch/revision to push to, created from the current revision if missing",
    )
    parser.add_argument(
        "--tags",
        nargs="+",
        default=None,
        help="tags to add to the generated dataset card",
    )
    parser.add_argument(
        "--upload_large_folder",
        action="store_true",
        help="use HfApi.upload_large_folder instead of upload_folder, for very large datasets",
    )
    args = parser.parse_args()

    main(
        root=args.root,
        repo_id=args.repo_id,
        private=args.private,
        license=args.license,
        push_videos=args.push_videos,
        branch=args.branch,
        tags=args.tags,
        upload_large_folder=args.upload_large_folder,
    )
