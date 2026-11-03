from pathlib import Path
import numpy as np

from so101_mujoco_pick_place.trajectory import WaypointRecorder, interpolate_ctrl, smoothstep


def test_smoothstep_ends():
    assert smoothstep(0.0) == 0.0
    assert smoothstep(1.0) == 1.0


def test_interpolate_midpoint():
    out = interpolate_ctrl(np.zeros(2), np.ones(2), 0.5)
    np.testing.assert_allclose(out, [0.5, 0.5])


def test_waypoint_roundtrip(tmp_path: Path):
    r = WaypointRecorder()
    r.add([1, 2, 3], [0.1, 0.2, 0.3])
    r.add([4, 5, 6], [0.4, 0.5, 0.6])
    assert r.undo().name == "waypoint_02"
    p = r.save(tmp_path / "t.json")
    loaded = WaypointRecorder.load(p)
    assert len(loaded) == 1
    assert loaded[0].name == "waypoint_01"
