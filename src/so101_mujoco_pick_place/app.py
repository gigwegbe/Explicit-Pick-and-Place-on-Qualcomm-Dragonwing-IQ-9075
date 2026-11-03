from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import sys
import time

import numpy as np

from .config import load_config
from .controller import SO101Controller
from .trajectory import WaypointRecorder, interpolate_ctrl


def _import_mujoco():
    try:
        import mujoco
        import mujoco.viewer
        return mujoco
    except ImportError as exc:
        raise SystemExit(
            "MuJoCo is not installed. Run: python -m pip install -e .\n"
            "On macOS, launch this app with mjpython instead of python."
        ) from exc


def _set_task_poses(mujoco, model, data, cfg):
    box_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "task_box")
    if box_id >= 0:
        model.body_pos[box_id] = np.asarray(cfg.box_xyz_m)

    cube_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cube_freejoint")
    if cube_joint >= 0:
        qadr = model.jnt_qposadr[cube_joint]
        dadr = model.jnt_dofadr[cube_joint]
        data.qpos[qadr : qadr + 7] = [*cfg.cube_xyz_m, 1.0, 0.0, 0.0, 0.0]
        data.qvel[dadr : dadr + 6] = 0.0
        mujoco.mj_forward(model, data)


def _cube_xyz(mujoco, model, data) -> np.ndarray:
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    return np.array(data.xpos[bid], dtype=float, copy=True)


def _cube_in_box(cube_xyz: np.ndarray, cfg) -> bool:
    dx = abs(cube_xyz[0] - cfg.box_xyz_m[0])
    dy = abs(cube_xyz[1] - cfg.box_xyz_m[1])
    inside_xy = dx < cfg.box_inner_half_xy_m[0] and dy < cfg.box_inner_half_xy_m[1]
    return bool(inside_xy and 0.0 <= cube_xyz[2] <= 0.06)


def _replay(mujoco, model, data, viewer, controller, waypoints, seconds_per_segment: float) -> None:
    if not waypoints:
        print("No waypoints to replay.")
        return
    print(f"Replaying {len(waypoints)} waypoints...")
    for wp in waypoints:
        start = controller.ctrl.copy()
        target = np.asarray(wp.ctrl, dtype=float)
        t0 = time.monotonic()
        while viewer.is_running():
            elapsed = time.monotonic() - t0
            alpha = min(1.0, elapsed / seconds_per_segment)
            controller.ctrl[:] = interpolate_ctrl(start, target, alpha)
            controller.apply()
            mujoco.mj_step(model, data)
            viewer.sync()
            if alpha >= 1.0:
                break
            time.sleep(max(0.0, model.opt.timestep * 0.5))
    print("Replay complete.")


def run(config_path: str) -> None:
    mujoco = _import_mujoco()
    cfg = load_config(config_path)
    if not cfg.scene_path.exists():
        raise SystemExit(
            f"Scene is missing: {cfg.scene_path}\n"
            "Run: python scripts/fetch_so101_model.py"
        )

    model = mujoco.MjModel.from_xml_path(str(cfg.scene_path))
    data = mujoco.MjData(model)
    _set_task_poses(mujoco, model, data, cfg)
    controller = SO101Controller(mujoco, model, data, cfg)
    recorder = WaypointRecorder()

    output_dir = cfg.root / "trajectories"
    output_dir.mkdir(exist_ok=True)
    last_saved: Path | None = None
    replay_requested = False
    reset_cube_requested = False
    status_requested = True

    joint_keys = {
        "a": ("shoulder_pan", -1), "d": ("shoulder_pan", +1),
        "w": ("shoulder_lift", +1), "s": ("shoulder_lift", -1),
        "j": ("elbow_flex", -1), "l": ("elbow_flex", +1),
        "i": ("wrist_flex", +1), "k": ("wrist_flex", -1),
        "n": ("wrist_roll", -1), "m": ("wrist_roll", +1),
    }

    def key_callback(keycode: int) -> None:
        nonlocal last_saved, replay_requested, reset_cube_requested, status_requested
        try:
            key = chr(keycode).lower()
        except (ValueError, OverflowError):
            return

        if key == "v":
            print(f"Control mode -> {controller.toggle_mode().upper()}")
            status_requested = True
            return
        if key == "o":
            controller.set_gripper(True); print("Gripper: OPEN"); return
        if key == "c":
            controller.set_gripper(False); print("Gripper: CLOSED"); return
        if key == "h":
            controller.set_home(); print("Home target loaded."); return
        if key == "b":
            wp = recorder.add(controller.ctrl, controller.ee_xyz())
            print(f"Saved {wp.name} at ee={np.round(wp.ee_xyz_m, 3).tolist()}")
            return
        if key == "u":
            wp = recorder.undo(); print(f"Undo: {wp.name}" if wp else "No waypoint to undo."); return
        if key == "x":
            recorder.clear(); print("Waypoints cleared."); return
        if key == "p":
            replay_requested = True; return
        if key == "r":
            reset_cube_requested = True; return
        if key == "g":
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            last_saved = recorder.save(output_dir / f"pick_place_{stamp}.json")
            print(f"Saved trajectory: {last_saved}")
            return
        if key == "q":
            status_requested = True; return

        if controller.mode == "joint":
            if key in joint_keys:
                name, direction = joint_keys[key]
                controller.nudge_joint(name, direction)
        else:
            step = cfg.cartesian_step_m
            deltas = {
                "w": np.array([+step, 0, 0]), "s": np.array([-step, 0, 0]),
                "a": np.array([0, +step, 0]), "d": np.array([0, -step, 0]),
                "z": np.array([0, 0, +step]), "f": np.array([0, 0, -step]),
            }
            if key in deltas:
                controller.nudge_cartesian(deltas[key])

    print("\nSO-101 MuJoCo Pick & Place")
    print("--------------------------")
    print("V: toggle JOINT/CARTESIAN mode | O/C: gripper open/close")
    print("B: save waypoint | U: undo | G: save JSON | P: replay")
    print("H: home | R: reset cube | Q: print status | X: clear waypoints")
    print("JOINT: A/D pan, W/S shoulder, J/L elbow, I/K wrist flex, N/M wrist roll")
    print("CARTESIAN: W/S +/-X, A/D +/-Y, Z/F +/-Z (5 mm default)")
    print("Close the viewer window to exit.\n")

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        success_printed = False
        last_status = 0.0
        while viewer.is_running():
            if reset_cube_requested:
                _set_task_poses(mujoco, model, data, cfg)
                reset_cube_requested = False
                success_printed = False
                print("Cube reset.")

            if replay_requested:
                replay_requested = False
                _replay(mujoco, model, data, viewer, controller, recorder.waypoints, cfg.replay_segment_seconds)

            if not controller.paused:
                controller.apply()
                mujoco.mj_step(model, data)

            now = time.monotonic()
            cube = _cube_xyz(mujoco, model, data)
            if _cube_in_box(cube, cfg) and not success_printed:
                print("SUCCESS: cube is inside the box region.")
                success_printed = True

            if status_requested or now - last_status > 2.0:
                status_requested = False
                last_status = now
                ee = controller.ee_xyz()
                print(
                    f"mode={controller.mode:9s}  waypoints={len(recorder.waypoints):2d}  "
                    f"ee={np.round(ee, 3).tolist()}  cube={np.round(cube, 3).tolist()}"
                )

            viewer.sync()
            time.sleep(max(0.0, model.opt.timestep * 0.5))

    if recorder.waypoints and last_saved is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = recorder.save(output_dir / f"autosave_{stamp}.json")
        print(f"Autosaved waypoints to {path}")


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Interactive SO-101 cube-to-box MuJoCo demo")
    parser.add_argument("--config", default="config.json")
    args = parser.parse_args(argv)
    run(args.config)


if __name__ == "__main__":
    main()
