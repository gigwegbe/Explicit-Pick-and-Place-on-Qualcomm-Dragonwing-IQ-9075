from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path


JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
ARM_JOINT_NAMES = JOINT_NAMES[:-1]


@dataclass(frozen=True)
class AppConfig:
    root: Path
    scene_path: Path
    joint_step_rad: float
    cartesian_step_m: float
    ik_damping: float
    replay_segment_seconds: float
    home: dict[str, float]
    cube_xyz_m: tuple[float, float, float]
    box_xyz_m: tuple[float, float, float]
    box_inner_half_xy_m: tuple[float, float]


def load_config(path: str | Path) -> AppConfig:
    path = Path(path).resolve()
    raw = json.loads(path.read_text())
    root = path.parent
    import math

    return AppConfig(
        root=root,
        scene_path=(root / raw["scene_path"]).resolve(),
        joint_step_rad=math.radians(float(raw["control"]["joint_step_deg"])),
        cartesian_step_m=float(raw["control"]["cartesian_step_m"]),
        ik_damping=float(raw["control"]["ik_damping"]),
        replay_segment_seconds=float(raw["control"]["replay_segment_seconds"]),
        home={k: float(v) for k, v in raw["home"].items()},
        cube_xyz_m=tuple(float(v) for v in raw["task"]["cube_xyz_m"]),
        box_xyz_m=tuple(float(v) for v in raw["task"]["box_xyz_m"]),
        box_inner_half_xy_m=tuple(float(v) for v in raw["task"]["box_inner_half_xy_m"]),
    )
