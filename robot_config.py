from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

ROBOT_CONFIG_PATH = Path(__file__).with_name("robot_configs.yaml")
DEFAULT_ROBOT = "dual"
ARM_DIM = 7  # 6 joints + gripper


@dataclass(frozen=True)
class ArmConfig:
    name: str
    leader_topic: str
    follower_topic: str
    follower_pose_topic: str
    pos_cmd_topic: str


@dataclass(frozen=True)
class CameraConfig:
    name: str
    color_topic: str
    depth_topic: str | None = None


@dataclass(frozen=True)
class RobotConfig:
    name: str
    arms: tuple
    cameras: tuple

    @property
    def arm_names(self):
        return [arm.name for arm in self.arms]

    @property
    def camera_names(self):
        return [camera.name for camera in self.cameras]

    @property
    def state_dim(self):
        return ARM_DIM * len(self.arms)

    def split_arms(self, vector):
        """Split a (state_dim,) vector into one (7,) vector per arm."""
        vector = np.asarray(vector)
        if vector.shape[-1] != self.state_dim:
            raise ValueError(f"Expected {self.state_dim} values for robot '{self.name}', got {vector.shape[-1]}")
        return [vector[..., i * ARM_DIM : (i + 1) * ARM_DIM] for i in range(len(self.arms))]


def _load_robot_configs():
    with ROBOT_CONFIG_PATH.open("r", encoding="utf-8") as file:
        raw_configs = yaml.safe_load(file)
    configs = {}
    for name, raw in raw_configs.items():
        configs[name] = RobotConfig(
            name=name,
            arms=tuple(ArmConfig(**arm) for arm in raw["arms"]),
            cameras=tuple(CameraConfig(**camera) for camera in raw["cameras"]),
        )
    return configs


ROBOT_CONFIGS = _load_robot_configs()


def get_robot_config(name):
    if name not in ROBOT_CONFIGS:
        raise ValueError(f"Unknown robot '{name}', expected one of {list(ROBOT_CONFIGS)} ({ROBOT_CONFIG_PATH})")
    return ROBOT_CONFIGS[name]


def add_robot_argument(parser):
    parser.add_argument(
        "--robot",
        type=str,
        choices=list(ROBOT_CONFIGS),
        default=DEFAULT_ROBOT,
        help=f"Robot layout (arms and cameras) defined in {ROBOT_CONFIG_PATH.name}",
    )
