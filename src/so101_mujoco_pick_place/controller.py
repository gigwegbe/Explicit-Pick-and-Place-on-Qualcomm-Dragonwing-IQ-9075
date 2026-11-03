from __future__ import annotations

import numpy as np

from .config import ARM_JOINT_NAMES, JOINT_NAMES, AppConfig


class SO101Controller:
    """Small joint-space + differential-IK controller around MuJoCo arrays."""

    def __init__(self, mujoco, model, data, cfg: AppConfig):
        self.mujoco = mujoco
        self.model = model
        self.data = data
        self.cfg = cfg
        self.mode = "joint"
        self.paused = False

        self.actuator_ids = {
            name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in JOINT_NAMES
        }
        missing = [n for n, i in self.actuator_ids.items() if i < 0]
        if missing:
            raise RuntimeError(f"Missing expected actuators in SO-101 model: {missing}")

        self.joint_ids = {
            name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in JOINT_NAMES
        }
        self.site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
        if self.site_id < 0:
            raise RuntimeError("SO-101 model has no 'gripperframe' site.")

        self.arm_dofs = np.array(
            [model.jnt_dofadr[self.joint_ids[name]] for name in ARM_JOINT_NAMES], dtype=int
        )
        self.arm_qpos = np.array(
            [model.jnt_qposadr[self.joint_ids[name]] for name in ARM_JOINT_NAMES], dtype=int
        )

        self.ctrl = np.array(data.ctrl, dtype=float, copy=True)
        self.set_home()

    def set_home(self) -> None:
        for name in JOINT_NAMES:
            aid = self.actuator_ids[name]
            self.ctrl[aid] = self.cfg.home[name]
        self._clip_all()
        self.apply()

    def apply(self) -> None:
        self.data.ctrl[:] = self.ctrl

    def ee_xyz(self) -> np.ndarray:
        self.mujoco.mj_forward(self.model, self.data)
        return np.array(self.data.site_xpos[self.site_id], dtype=float, copy=True)

    def toggle_mode(self) -> str:
        self.mode = "cartesian" if self.mode == "joint" else "joint"
        return self.mode

    def nudge_joint(self, name: str, direction: float) -> None:
        aid = self.actuator_ids[name]
        self.ctrl[aid] += float(direction) * self.cfg.joint_step_rad
        self._clip_actuator(aid)
        self.apply()

    def set_gripper(self, opened: bool) -> None:
        aid = self.actuator_ids["gripper"]
        lo, hi = self.model.actuator_ctrlrange[aid]
        self.ctrl[aid] = hi if opened else lo
        self.apply()

    def nudge_cartesian(self, delta_xyz: np.ndarray) -> None:
        """One damped-least-squares differential IK step for position only."""
        self.mujoco.mj_forward(self.model, self.data)
        jacp = np.zeros((3, self.model.nv), dtype=float)
        jacr = np.zeros((3, self.model.nv), dtype=float)
        self.mujoco.mj_jacSite(self.model, self.data, jacp, jacr, self.site_id)
        j = jacp[:, self.arm_dofs]

        lam = self.cfg.ik_damping
        jj_t = j @ j.T
        dq = j.T @ np.linalg.solve(jj_t + (lam * lam) * np.eye(3), np.asarray(delta_xyz, dtype=float))

        # Convert current actuator targets into a joint target vector, then add dq.
        for idx, name in enumerate(ARM_JOINT_NAMES):
            aid = self.actuator_ids[name]
            self.ctrl[aid] += float(dq[idx])
            self._clip_actuator(aid)
        self.apply()

    def _clip_actuator(self, aid: int) -> None:
        lo, hi = self.model.actuator_ctrlrange[aid]
        self.ctrl[aid] = float(np.clip(self.ctrl[aid], lo, hi))

    def _clip_all(self) -> None:
        for aid in self.actuator_ids.values():
            self._clip_actuator(aid)
