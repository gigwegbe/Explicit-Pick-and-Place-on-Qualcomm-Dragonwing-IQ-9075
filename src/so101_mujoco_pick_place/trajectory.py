from __future__ import annotations

from dataclasses import dataclass, asdict
import json
from pathlib import Path
from typing import Iterable

import numpy as np


@dataclass
class Waypoint:
    name: str
    ctrl: list[float]
    ee_xyz_m: list[float]


class WaypointRecorder:
    def __init__(self) -> None:
        self.waypoints: list[Waypoint] = []

    def add(self, ctrl: Iterable[float], ee_xyz_m: Iterable[float]) -> Waypoint:
        wp = Waypoint(
            name=f"waypoint_{len(self.waypoints) + 1:02d}",
            ctrl=[float(v) for v in ctrl],
            ee_xyz_m=[float(v) for v in ee_xyz_m],
        )
        self.waypoints.append(wp)
        return wp

    def undo(self) -> Waypoint | None:
        return self.waypoints.pop() if self.waypoints else None

    def clear(self) -> None:
        self.waypoints.clear()

    def save(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {"format": "so101-mujoco-waypoints-v1", "waypoints": [asdict(w) for w in self.waypoints]}
        path.write_text(json.dumps(payload, indent=2))
        return path

    @staticmethod
    def load(path: str | Path) -> list[Waypoint]:
        data = json.loads(Path(path).read_text())
        return [Waypoint(**item) for item in data["waypoints"]]


def smoothstep(alpha: float) -> float:
    alpha = float(np.clip(alpha, 0.0, 1.0))
    return alpha * alpha * (3.0 - 2.0 * alpha)


def interpolate_ctrl(start: np.ndarray, target: np.ndarray, alpha: float) -> np.ndarray:
    s = smoothstep(alpha)
    return start + s * (target - start)
