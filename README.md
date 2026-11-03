# SO-101 MuJoCo Pick & Place

A deliberately small classical-robotics project: control an SO-101 in MuJoCo,
pick up the supplied 30 mm cube, place it in the supplied 100 x 100 x 20 mm box,
save important poses as waypoints, then replay them.

## What is included

- Your `cube.stl` and `box.stl` in `model/task_assets/`
- A MuJoCo task scene with a table, dynamic cube, and open-top box
- Joint-space keyboard control
- Optional Cartesian XYZ nudging using MuJoCo Jacobian differential IK
- Waypoint save / undo / JSON export
- Smooth waypoint replay
- Cube reset and simple success detection

The robot model itself is fetched from the official SO-ARM100 repository rather
than duplicated here.

## 0. Setting up the board

This project runs on a Qualcomm Dragonwing IQ-9075 EVK. The board ships with a factory image, so flash it with Ubuntu first, then do everything below on the board.

1. Flash Ubuntu. Follow the [IQ-9075 EVK Ubuntu quickstart](https://docs.qualcomm.com/bundle/publicresource/80-90441-252/topics/iq9-ubuntu-qsg-landing-page-1.md) for the out-of-box steps, and the [update-software guide](https://dragonwingdocs.qualcomm.com/Ubuntu/devices/iq9075-evk/update-software/overview.md) to write the image. Either method works: [flash over QLI](https://dragonwingdocs.qualcomm.com/Ubuntu/devices/iq9075-evk/update-software/flash-over-qli.md) or the [Qualcomm launcher](https://dragonwingdocs.qualcomm.com/Ubuntu/devices/iq9075-evk/update-software/flash-using-qualcomm-launcher.md).
2. Connect the hardware. SO-101 over its USB-to-serial adapter, the two cameras over USB.
3. Continue with "Create an environment" below, on the board.

## 1. Create an environment

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e .
```

## 2. Fetch the SO-101 MuJoCo model

```bash
python scripts/fetch_so101_model.py
```

If you already cloned SO-ARM100:

```bash
python scripts/fetch_so101_model.py \
  --source ~/SO-ARM100/Simulation/SO101

   /home/ubuntu/Documents/lerobot-setup/rerun-view/claude/v4/so101_mujoco_pick_place/model


python scripts/fetch_so101_model.py   --source /home/ubuntu/Documents/lerobot-setup/rerun-view/chat/so101_teach_replay/SO-ARM100/Simulation/SO101/

```

After this, these should exist:

```text
model/
├── scene.xml
├── so101_new_calib.xml
├── assets/                 # official SO-101 meshes
└── task_assets/
    ├── cube.stl
    └── box.stl
```

## 3. Run

Linux:

```bash
so101-mujoco-pick-place --config config.json
```

macOS: MuJoCo's passive viewer requires `mjpython`:

```bash
mjpython -m so101_mujoco_pick_place.app --config config.json
```

## Controls

### General

| Key | Action |
| --- | --- |
| `V` | Toggle JOINT / CARTESIAN mode |
| `O` / `C` | Open / close gripper |
| `B` | Save current pose as a waypoint |
| `U` | Undo last waypoint |
| `G` | Save waypoints to JSON |
| `P` | Replay current waypoints |
| `H` | Move target back to home |
| `R` | Reset the cube |
| `X` | Clear waypoints |
| `Q` | Print status |

### JOINT mode

| Keys | Joint |
| --- | --- |
| `A / D` | shoulder pan - / + |
| `W / S` | shoulder lift + / - |
| `J / L` | elbow flex - / + |
| `I / K` | wrist flex + / - |
| `N / M` | wrist roll - / + |

Each press changes the selected joint by 3 degrees by default.

### CARTESIAN mode

| Keys | End-effector nudge |
| --- | --- |
| `W / S` | +X / -X |
| `A / D` | +Y / -Y |
| `Z / F` | +Z / -Z |

Each press requests a 5 mm movement. The project uses MuJoCo's end-effector
Jacobian and damped least-squares differential IK to convert that XYZ movement
into changes across the five arm joints.

## Suggested pick-and-place teaching sequence

1. Press `V` if you want Cartesian mode.
2. Move above the cube.
3. Press `B` to save `waypoint_01`.
4. Move down to the cube and press `B`.
5. Press `C` to close the gripper; press `B` if you want the grasp state saved.
6. Lift; save another waypoint.
7. Move above the box; save.
8. Lower into the box; save.
9. Press `O` to release.
10. Retreat; save.
11. Press `G` to write the trajectory JSON.
12. Press `R` to reset the cube, then `P` to replay.

Think of waypoints as bus stops. The replay code creates smooth motion between
the stops.

## Why the box uses simple collision primitives

The supplied `box.stl` is rendered exactly, but MuJoCo normally treats an STL
mesh collision as a convex hull. A convex hull around an open box can seal the
opening. To keep the task reliable, the STL is visual-only and five simple box
geometries form the floor and four walls for collision.

The cube follows the same principle: your STL is the visual geometry; a 30 mm
box primitive is the collision geometry.

## Configuration

`config.json` controls:

- cube start position
- box position
- home joint target
- joint nudge size
- Cartesian nudge size
- IK damping
- replay speed

The default cube position is 20 cm in front along negative X because that is a
convenient starting side for this SO-101 MJCF. If it is not where you want it,
change only `task.cube_xyz_m` and restart.

## Tests

Hardware-independent tests:

```bash
python -m pip install pytest
pytest -q
```

The tests verify waypoint save/load/interpolation and that the included STL
files are the expected 30 mm cube and 100 x 100 x 20 mm box.

## Notes

- This is a simulator project; physical-arm calibration is not required.
- The official SO-101 `new_calib` MJCF uses the newer zero convention.
- The upstream SO-101 model currently notes that its gripper mapping is not the
  same 0-100 convention used by LeRobot hardware. In this simulator we use the
  MuJoCo actuator's own control range directly.
- Contact/grasp behaviour depends on the upstream robot collision model. If a
  grasp slips, first try slower movements and a centered grasp before tuning
  friction or collision geometry.
