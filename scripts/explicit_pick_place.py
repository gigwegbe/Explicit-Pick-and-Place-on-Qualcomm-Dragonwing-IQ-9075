#!/usr/bin/env python3
"""
explicit_pick_place.py  --  The rung BEFORE imitation learning.

The whole idea in one sentence:
    You TELL it where the cube is and where the tray is.
    It goes there, picks up, carries over, and drops.

No learning. No camera. No policy. Just two coordinates you type in by hand
and a fixed recipe of moves. This is "explicit" control -- the baseline that
imitation learning (the ACT policy) eventually replaces.

This script is SIM ONLY: a MuJoCo twin of the SO-101, nothing plugged in. Driving
a physical arm is a separate story (teach-by-demonstration -> teleop_so101.py).

The visible API is a single call:

    pick_and_place(cube=[0.18, 0.06, 0.015], tray=[0.20, -0.10, 0.0])

Run:
    pip install mujoco numpy
    python explicit_pick_place.py                      # headless, default reachable layout
    python explicit_pick_place.py --view               # OPEN A WINDOW and watch it live
    python explicit_pick_place.py --cube 0.26 -0.12 0.015 --tray 0.24 0.14 0
    python explicit_pick_place.py --render out.mp4      # save a video instead
"""
import os, time, argparse, numpy as np, mujoco

HERE = os.path.dirname(os.path.abspath(__file__))

# Find scene.xml wherever it lives: in ../model (scripts/ layout), alongside
# this file, or as the original scene_pick.xml. Override with --scene.
_CANDIDATES = [
    os.path.join(HERE, "..", "model", "scene.xml"),
    os.path.join(HERE, "scene.xml"),
    os.path.join(HERE, "scene_pick.xml"),
]
SCENE = next((p for p in _CANDIDATES if os.path.exists(p)), _CANDIDATES[0])

# Tool heights above the table, in metres.
HOVER, GRASP, PLACE = 0.12, 0.016, 0.045   # gripper-site heights (m)
OPEN,  CLOSE        = 1.0, -0.3           # gripper joint target

# Gripper pocket offset. The IK aims the gripperframe SITE at a point, but the
# finger pocket sits a little ahead of that site, so a raw target lands one
# finger on the middle of the cube instead of straddling it. Nudging the PICK
# target back by this much puts the pocket -- not the site -- over the cube.
# (Measured for this fixed demo layout; only applied to the pick, not the place.)
GRIP_DX, GRIP_DY = -0.018, -0.009

# The five arm joints, base to wrist. Looked up by name so scene ordering
# (where the cube/tray sit in qpos) can never shift them.
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


class Twin:
    """The MuJoCo twin of the SO-101. Knows how to reach a point and how to grip."""

    def __init__(self, scene=SCENE):
        self.m = mujoco.MjModel.from_xml_path(scene)
        self.d = mujoco.MjData(self.m)
        self.dt = self.m.opt.timestep
        self._scratch = mujoco.MjData(self.m)
        name = lambda k, s: mujoco.mj_name2id(self.m, k, s)
        def first(kind, *names):
            for n in names:
                i = name(kind, n)
                if i != -1:
                    return i, n
            return -1, None
        self.site = name(mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
        self.cube_body = name(mujoco.mjtObj.mjOBJ_BODY, "cube")
        cj, _ = first(mujoco.mjtObj.mjOBJ_JOINT, "cube_freejoint", "cube_free")
        if cj == -1:
            raise RuntimeError("no cube free joint found (looked for 'cube_freejoint', 'cube_free')")
        self.cube_qadr = self.m.jnt_qposadr[cj]
        # Adhesion is OPTIONAL. If the scene has no grip adhesion actuators, we
        # simply skip the sim 'stick' crutch; the weld (below) is the real hold.
        self.adh = [i for i in (name(mujoco.mjtObj.mjOBJ_ACTUATOR, "grip_fixed"),
                                name(mujoco.mjtObj.mjOBJ_ACTUATOR, "grip_jaw")) if i != -1]
        # Preferred hold is a weld (rigid, no swing); adhesion is the fallback.
        self.gripper_body = name(mujoco.mjtObj.mjOBJ_BODY, "gripper")
        self.weld_id = name(mujoco.mjtObj.mjOBJ_EQUALITY, "grip_weld")
        # Resolve the arm by NAME, never by position. The cube is a free body,
        # so depending on how the scene is ordered it can occupy the first qpos
        # slots -- then qpos[:5] would be the cube, not the arm, and the IK would
        # fling the cube off the table. Name lookups make body order irrelevant.
        jid = lambda s: name(mujoco.mjtObj.mjOBJ_JOINT, s)
        aid = lambda s: name(mujoco.mjtObj.mjOBJ_ACTUATOR, s)
        self.arm_q   = np.array([self.m.jnt_qposadr[jid(n)] for n in ARM_JOINTS])
        self.arm_v   = np.array([self.m.jnt_dofadr[jid(n)]  for n in ARM_JOINTS])
        self.arm_lo  = np.array([self.m.jnt_range[jid(n)][0] for n in ARM_JOINTS])
        self.arm_hi  = np.array([self.m.jnt_range[jid(n)][1] for n in ARM_JOINTS])
        self.arm_act = np.array([aid(n) for n in ARM_JOINTS])
        self.grip_act = aid("gripper")
        self._renderer = None
        self._viewer = None
        self._realtime = False

    # Put the cube where we were TOLD it is. It is a free body, so its pose
    # lives in qpos, NOT in body_pos -- setting body_pos would do nothing.
    def place_cube(self, xyz):
        a = self.cube_qadr
        self.d.qpos[a:a + 3] = np.asarray(xyz, float)
        self.d.qpos[a + 3:a + 7] = [1, 0, 0, 0]       # upright, no rotation
        mujoco.mj_forward(self.m, self.d)

    # --- inverse kinematics: MuJoCo's own Jacobian, gripper pointing down ---
    def reach_for(self, target, seed, iters=120):
        s = self._scratch
        s.qpos[:] = self.d.qpos; s.qpos[self.arm_q] = seed
        target = np.asarray(target, float)
        for _ in range(iters):
            mujoco.mj_forward(self.m, s)
            pos_err = target - s.site_xpos[self.site]
            jp = np.zeros((3, self.m.nv)); jr = np.zeros((3, self.m.nv))
            mujoco.mj_jacSite(self.m, s, jp, jr, self.site)
            approach = s.site_xmat[self.site].reshape(3, 3)[:, 0]
            rot_err = np.cross(approach, [0, 0, -1.0]) * 0.5
            err = np.concatenate([pos_err, rot_err])
            if np.linalg.norm(pos_err) < 5e-4:
                break
            J = np.vstack([jp[:, self.arm_v], jr[:, self.arm_v]])   # arm DOF cols
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-3 * np.eye(6), err)
            nrm = np.linalg.norm(dq)
            if nrm > 0.1:
                dq *= 0.1 / nrm
            s.qpos[self.arm_q] = np.clip(s.qpos[self.arm_q] + 0.6 * dq,
                                         self.arm_lo, self.arm_hi)
        return s.qpos[self.arm_q].copy()

    def _step(self):
        mujoco.mj_step(self.m, self.d)
        if self._viewer is not None:
            self._viewer.sync()
            if self._realtime:
                time.sleep(self.dt)
        if self._renderer is not None:
            self._rk += 1
            if self._rk % self._every == 0:
                self._renderer.update_scene(self.d, self._cam)
                self.frames.append(self._renderer.render().copy())

    def hold(self, sec):
        for _ in range(int(sec / self.dt)):
            self._step()

    def _weld(self, on):
        """Lock (or release) the cube rigidly to the gripper at its current pose."""
        if on:
            g, c = self.gripper_body, self.cube_body
            negq1 = np.zeros(4); mujoco.mju_negQuat(negq1, self.d.xquat[g])
            relpos = np.zeros(3)
            mujoco.mju_rotVecQuat(relpos, self.d.xpos[c] - self.d.xpos[g], negq1)
            relquat = np.zeros(4); mujoco.mju_mulQuat(relquat, negq1, self.d.xquat[c])
            self.m.eq_data[self.weld_id, 0:3] = 0.0          # anchor (unused here)
            self.m.eq_data[self.weld_id, 3:6] = relpos       # relative position
            self.m.eq_data[self.weld_id, 6:10] = relquat     # relative orientation
            self.m.eq_data[self.weld_id, 10] = 1.0           # torquescale
        self.d.eq_active[self.weld_id] = 1 if on else 0

    def set_grip(self, grip=None, stick=None):
        if grip is not None:
            self.d.ctrl[self.grip_act] = grip
        if stick is not None:
            if self.weld_id != -1:             # rigid hold, no swing
                self._weld(stick > 0.5)
            else:
                for a in self.adh:             # fallback: soft adhesion
                    self.d.ctrl[a] = stick

    def move_to(self, p0, p1, T=1.0):
        p0, p1 = np.array(p0, float), np.array(p1, float)
        n = int(T / self.dt)
        for i in range(n):
            a = (i + 1) / n
            self.d.ctrl[self.arm_act] = self.reach_for(p0 * (1 - a) + p1 * a, self.d.ctrl[self.arm_act])
            self._step()

    def enable_render(self, w=720, h=540, fps=30):
        import sys
        if sys.platform.startswith("linux"):
            os.environ.setdefault("MUJOCO_GL", "egl")   # macOS/Windows pick their own
        # Grow the offscreen framebuffer to fit the requested size, so the render
        # works no matter what offwidth/offheight the scene file declares.
        self.m.vis.global_.offwidth = max(self.m.vis.global_.offwidth, w)
        self.m.vis.global_.offheight = max(self.m.vis.global_.offheight, h)
        self._renderer = mujoco.Renderer(self.m, h, w)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [0.25, 0.01, 0.04]
        cam.distance, cam.azimuth, cam.elevation = 0.72, 158, -28
        self._cam = cam
        self._every = max(1, int(round((1.0 / fps) / self.dt)))
        self._rk = 0; self.frames = []

    def enable_view(self):
        """Open an interactive window and run the motion in real time."""
        import mujoco.viewer
        self._viewer = mujoco.viewer.launch_passive(self.m, self.d)
        self._realtime = True

    def close_view(self, keep_open=True):
        if self._viewer is not None and keep_open:
            print("\n(window is live -- close it to exit)")
            while self._viewer.is_running():
                time.sleep(0.05)
        if self._viewer is not None:
            self._viewer.close()


# =========================================================================
#  THE VISIBLE API  --  this is the whole explicit story in one function.
# =========================================================================
def pick_and_place(cube, tray, view=False, render=None, fps=30, scene=SCENE):
    """
    Explicit pick-and-place. YOU supply the two positions; the arm obeys.

        cube : [x, y, z] of the cube centre, in metres, in the base frame.
        tray : [x, y, z] of the tray, in metres.

    There is no perception here. The arm goes exactly where you said the cube
    is. If the cube is not actually there, it grabs empty air -- and that gap
    is exactly what imitation learning exists to close.
    """
    twin = Twin(scene)
    if view:
        twin.enable_view()
    if render:
        twin.enable_render(fps=fps)
    m, d = twin.m, twin.d

    cube = np.asarray(cube, float); tray = np.asarray(tray, float)
    cx, cy = cube[0], cube[1]
    tx, ty = tray[0], tray[1]
    # Pick targets carry the pocket offset so the fingers straddle the cube;
    # the place targets are left alone (release does not need pocket centring).
    gx, gy = cx + GRIP_DX, cy + GRIP_DY
    above_cube = [gx, gy, HOVER]; on_cube = [gx, gy, GRASP]
    above_tray = [tx, ty, HOVER]; in_tray = [tx, ty, PLACE]

    mujoco.mj_resetData(m, d)
    twin.place_cube(cube)                       # cube goes where we were TOLD
    d.ctrl[twin.arm_act] = twin.reach_for(above_cube, np.array([0, 0.3, -0.3, 0.3, 0.0]))
    twin.set_grip(OPEN, 0.0); twin.hold(0.4)
    start_z = d.xpos[twin.cube_body][2]

    # The fixed recipe -- eight hand-written moves, no learning anywhere.
    twin.move_to(above_cube, on_cube, 0.8); twin.hold(0.2)   # 1. descend
    twin.set_grip(CLOSE, 1.0); twin.hold(1.0)                # 2. close + grip
    twin.move_to(on_cube, above_cube, 1.0)                   # 3. lift
    lift_z = d.xpos[twin.cube_body][2]
    twin.move_to(above_cube, above_tray, 1.2)                # 4. carry
    twin.move_to(above_tray, in_tray, 0.8)                   # 5. lower
    twin.set_grip(OPEN, 0.0); twin.hold(1.0)                 # 6. release
    twin.move_to(in_tray, above_tray, 0.8); twin.hold(1.5)   # 7. retreat

    final = d.xpos[twin.cube_body].copy()
    placed = (abs(final[0] - tx) < 0.048 and
              abs(final[1] - ty) < 0.048 and final[2] < 0.05)

    # --- make the explicit contract, and its brittleness, VISIBLE ---
    print("-" * 56)
    print("EXPLICIT PICK-AND-PLACE  (no camera, no learning)")
    print("-" * 56)
    print("I was TOLD the cube is at : [%.3f %.3f %.3f] m" % tuple(cube))
    print("I was TOLD the tray is at : [%.3f %.3f %.3f] m" % tuple(tray))
    print("cube started at z         : %6.1f mm" % (start_z * 1000))
    print("cube lifted to z          : %6.1f mm" % (lift_z * 1000))
    print("cube ended at             : [%.3f %.3f %.3f] m" % tuple(final))
    print("RESULT                    : %s" %
          ("CUBE PLACED IN TRAY" if placed else "MISSED"))
    print("-" * 56)
    print("Note: I went exactly where I was told. Move the real cube even")
    print("a centimetre from that spot and I would have closed on empty air.")
    print("Giving the arm EYES to find the cube itself is the next rung:")
    print("that is imitation learning.")
    print("-" * 56)

    if render:
        import imageio
        imageio.mimsave(render, twin.frames, fps=fps)
        print("wrote %s  (%d frames)" % (render, len(twin.frames)))
    if view:
        twin.close_view()

    return placed


def main():
    ap = argparse.ArgumentParser(description="Explicit SO-101 pick-and-place in MuJoCo (sim only).")
    ap.add_argument("--cube", nargs=3, type=float, default=[0.18, 0.06, 0.015],
                    metavar=("X", "Y", "Z"), help="cube centre, metres (in FRONT of the arm)")
    ap.add_argument("--tray", nargs=3, type=float, default=[0.20, -0.10, 0.0],
                    metavar=("X", "Y", "Z"), help="box/tray position, metres (in FRONT of the arm)")
    ap.add_argument("--view", action="store_true", help="open a window and watch it live")
    ap.add_argument("--render", metavar="OUT.mp4", default=None, help="save a video instead")
    ap.add_argument("--scene", default=SCENE, help="path to scene.xml")
    ap.add_argument("--fps", type=int, default=30)
    a = ap.parse_args()
    if not a.view and not a.render:
        print("(running headless -- pass --view to watch it in a window)")
    pick_and_place(a.cube, a.tray, view=a.view, render=a.render, fps=a.fps, scene=a.scene)


if __name__ == "__main__":
    main()