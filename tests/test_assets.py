from pathlib import Path
import struct


def binary_stl_bounds(path: Path):
    raw = path.read_bytes()
    count = struct.unpack_from('<I', raw, 80)[0]
    mins = [float('inf')] * 3
    maxs = [float('-inf')] * 3
    off = 84
    for _ in range(count):
        off += 12  # normal
        for _ in range(3):
            xyz = struct.unpack_from('<fff', raw, off)
            off += 12
            for i, v in enumerate(xyz):
                mins[i] = min(mins[i], v)
                maxs[i] = max(maxs[i], v)
        off += 2
    return mins, maxs


def test_user_cube_dimensions():
    root = Path(__file__).resolve().parents[1]
    lo, hi = binary_stl_bounds(root / 'model/task_assets/cube.stl')
    ext = [round(hi[i] - lo[i], 4) for i in range(3)]
    assert ext == [30.0, 30.0, 30.0]


def test_user_box_dimensions():
    root = Path(__file__).resolve().parents[1]
    lo, hi = binary_stl_bounds(root / 'model/task_assets/box.stl')
    ext = [round(hi[i] - lo[i], 3) for i in range(3)]
    assert ext == [100.0, 100.0, 20.0]
