# #!/usr/bin/env python3
# """
# teleop_so101.py  --  Teach by demonstration on a single SO-101 (rung 2).

# No sim, no IK, no calibration map. You physically move the arm; it records its
# OWN joint angles, so there is no sim-to-real mismatch to fix. One follower arm,
# your hand. (Kinesthetic teaching -- no leader arm needed.)

# RECORD  the arm goes limp (torque off). Move it by hand. Keys:
#           s = save the pose you are holding      c = close gripper (grabs for real)
#           o = open gripper                       q = finish and write waypoints.json
# REPLAY  torque on, reads waypoints.json, moves through the saved poses in order.

#     pip install lerobot
#     python teleop_so101.py record --port /dev/ttyACM2 --arm-id orange_robot
#     python teleop_so101.py replay --port /dev/ttyACM2 --arm-id orange_robot
# """
# import sys, json, time, argparse

# MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
# WAYPOINTS = "waypoints.json"
# GRIP_OPEN, GRIP_CLOSE = 100.0, 8.0     # gripper 0..100 scale (not degrees); tune if needed


# def connect(port, arm_id, max_rel):
#     """Connect to the follower. max_rel=None means no per-command rate cap."""
#     from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
#     robot = SO101Follower(SO101FollowerConfig(
#         port=port, id=arm_id, use_degrees=True, max_relative_target=max_rel))
#     robot.connect(calibrate=False)     # reuse the calibration already on the arm
#     return robot


# def read_pose(robot):
#     """Current joint positions as a plain {motor: degrees} dict."""
#     obs = robot.get_observation()
#     return {m: float(obs[f"{m}.pos"]) for m in MOTORS}


# def getch():
#     """Read one keypress, no Enter needed (POSIX)."""
#     import termios, tty
#     fd = sys.stdin.fileno()
#     old = termios.tcgetattr(fd)
#     try:
#         tty.setraw(fd)
#         return sys.stdin.read(1)
#     finally:
#         termios.tcsetattr(fd, termios.TCSADRAIN, old)


# def record(port, arm_id):
#     robot = connect(port, arm_id, max_rel=None)
#     robot.bus.disable_torque()                 # whole arm limp so you can move it
#     grip = GRIP_OPEN
#     waypoints = []
#     print("RECORD -- move the arm by hand.")
#     print("  s=save pose   c=close gripper   o=open gripper   q=finish")
#     try:
#         while True:
#             k = getch().lower()
#             if k == "s":
#                 pose = read_pose(robot)
#                 pose["gripper"] = grip         # store the gripper state we want here
#                 waypoints.append(pose)
#                 print(f"  saved pose {len(waypoints)}  (gripper={'CLOSED' if grip==GRIP_CLOSE else 'open'})")
#             elif k == "c":
#                 grip = GRIP_CLOSE
#                 robot.bus.enable_torque("gripper")      # power ONLY the gripper
#                 robot.send_action({"gripper.pos": grip})
#                 print("  gripper CLOSED")
#             elif k == "o":
#                 grip = GRIP_OPEN
#                 robot.bus.enable_torque("gripper")
#                 robot.send_action({"gripper.pos": grip})
#                 print("  gripper open")
#             elif k == "q":
#                 break
#     finally:
#         if waypoints:
#             with open(WAYPOINTS, "w") as f:
#                 json.dump(waypoints, f, indent=2)
#             print(f"\nwrote {len(waypoints)} poses to {WAYPOINTS}")
#         else:
#             print("\nno poses saved.")
#         robot.disconnect()                     # disables torque on the way out


# def replay(port, arm_id, dwell=0.8, steps=30, seg=1.2):
#     with open(WAYPOINTS) as f:
#         waypoints = json.load(f)
#     robot = connect(port, arm_id, max_rel=None)   # connect leaves torque ON
#     print(f"REPLAY -- {len(waypoints)} poses from {WAYPOINTS}")
#     try:
#         cur = read_pose(robot)                 # start from wherever the arm is
#         for i, target in enumerate(waypoints, 1):
#             # ease from the current pose to the target so motion is smooth, not a jump
#             for s in range(1, steps + 1):
#                 a = s / steps
#                 blend = {f"{m}.pos": (1 - a) * cur[m] + a * target[m] for m in MOTORS}
#                 robot.send_action(blend)
#                 time.sleep(seg / steps)
#             cur = target
#             print(f"  reached pose {i}/{len(waypoints)}")
#             time.sleep(dwell)                  # settle before the next one
#     finally:
#         robot.disconnect()


# def main():
#     ap = argparse.ArgumentParser(description="Teach-by-demonstration for one SO-101.")
#     ap.add_argument("mode", choices=["record", "replay"])
#     ap.add_argument("--port", required=True, help="serial port, e.g. /dev/ttyACM2")
#     ap.add_argument("--arm-id", required=True, help="LeRobot calibration id, e.g. orange_robot")
#     a = ap.parse_args()
#     if a.mode == "record":
#         record(a.port, a.arm_id)
#     else:
#         replay(a.port, a.arm_id)


# if __name__ == "__main__":
#     main()



#!/usr/bin/env python3
"""
teleop_so101.py  --  Teach by demonstration on a single SO-101 (rung 2).

No sim, no IK, no calibration map. You physically move the arm; it records its
OWN joint angles, so there is no sim-to-real mismatch to fix. One follower arm,
your hand. (Kinesthetic teaching -- no leader arm needed.)

RECORD  the arm goes limp (torque off). Move it by hand. Keys:
          s = save the pose you are holding      c = close gripper (grabs for real)
          o = open gripper                       q = finish and write waypoints.json
REPLAY  torque on, reads waypoints.json, moves through the saved poses in order.

    pip install lerobot
    python teleop_so101.py record --port /dev/ttyACM2 --arm-id orange_robot
    python teleop_so101.py replay --port /dev/ttyACM2 --arm-id orange_robot

Gripper overload: the gripper servo (id 6) trips a Feetech "Overload" alarm if it
stalls -- e.g. commanded to close past where the cube or its own travel stops it.
This script no longer crashes when that happens; it warns and carries on. To CLEAR
a tripped alarm, power-cycle the arm (unplug its power a few seconds). If it keeps
tripping on a grab, raise --grip-close (a larger number closes less hard).
"""
import sys, json, time, argparse

MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
ARM = MOTORS[:5]
WAYPOINTS = "waypoints.json"


def connect(port, arm_id, max_rel):
    """Connect. We keep torque-on-disconnect OFF and manage torque ourselves, so a
    faulted gripper can never crash shutdown."""
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
    robot = SO101Follower(SO101FollowerConfig(
        port=port, id=arm_id, use_degrees=True,
        max_relative_target=max_rel, disable_torque_on_disconnect=False))
    robot.connect(calibrate=False)     # reuse the calibration already on the arm
    return robot


def read_pose(robot):
    obs = robot.get_observation()
    return {m: float(obs[f"{m}.pos"]) for m in MOTORS}


def safe(fn, what):
    """Run a motor op; turn a Feetech fault into a warning instead of a crash."""
    try:
        fn(); return True
    except Exception as e:
        msg = str(e)
        if "Overload" in msg:
            print(f"  ! gripper OVERLOAD during {what}. Power-cycle the arm to clear it. "
                  f"(continuing)")
        else:
            print(f"  ! {what} failed: {msg} (continuing)")
        return False


def getch():
    """One keypress, no Enter (POSIX)."""
    import termios, tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def set_grip(robot, pos):
    """Power only the gripper and send it to pos; never raises."""
    safe(lambda: robot.bus.enable_torque("gripper"), "gripper power-on")
    safe(lambda: robot.send_action({"gripper.pos": float(pos)}), "gripper move")


def shutdown(robot):
    """Best-effort: drop torque on the arm (skip a faulted gripper), then disconnect."""
    safe(lambda: robot.bus.disable_torque(ARM), "arm torque-off")
    safe(lambda: robot.bus.disable_torque("gripper"), "gripper torque-off")
    safe(lambda: robot.disconnect(), "disconnect")


def record(port, arm_id, grip_open, grip_close):
    robot = connect(port, arm_id, max_rel=None)
    safe(lambda: robot.bus.disable_torque(), "going limp")   # whole arm limp
    grip = grip_open
    waypoints = []
    print("RECORD -- move the arm by hand.")
    print("  s=save pose   c=close gripper   o=open gripper   q=finish")
    try:
        while True:
            k = getch().lower()
            if k == "s":
                pose = read_pose(robot)
                pose["gripper"] = grip          # store the gripper state we want here
                waypoints.append(pose)
                print(f"  saved pose {len(waypoints)}  (gripper={'CLOSED' if grip==grip_close else 'open'})")
            elif k == "c":
                grip = grip_close; set_grip(robot, grip); print("  gripper -> close")
            elif k == "o":
                grip = grip_open;  set_grip(robot, grip); print("  gripper -> open")
            elif k == "q":
                break
    finally:
        if waypoints:
            with open(WAYPOINTS, "w") as f:
                json.dump(waypoints, f, indent=2)
            print(f"\nwrote {len(waypoints)} poses to {WAYPOINTS}")
        else:
            print("\nno poses saved.")
        shutdown(robot)


def replay(port, arm_id, dwell=0.8, steps=30, seg=1.2):
    with open(WAYPOINTS) as f:
        waypoints = json.load(f)
    robot = connect(port, arm_id, max_rel=None)   # connect leaves torque ON
    print(f"REPLAY -- {len(waypoints)} poses from {WAYPOINTS}")
    try:
        cur = read_pose(robot)                 # start from wherever the arm is
        for i, target in enumerate(waypoints, 1):
            for s in range(1, steps + 1):      # ease to the target, no jump
                a = s / steps
                blend = {f"{m}.pos": (1 - a) * cur[m] + a * target[m] for m in MOTORS}
                safe(lambda b=blend: robot.send_action(b), "move")
                time.sleep(seg / steps)
            cur = target
            print(f"  reached pose {i}/{len(waypoints)}")
            time.sleep(dwell)
    finally:
        shutdown(robot)


def main():
    ap = argparse.ArgumentParser(description="Teach-by-demonstration for one SO-101.")
    ap.add_argument("mode", choices=["record", "replay"])
    ap.add_argument("--port", required=True, help="serial port, e.g. /dev/ttyACM2")
    ap.add_argument("--arm-id", required=True, help="LeRobot calibration id, e.g. orange_robot")
    ap.add_argument("--grip-open", type=float, default=100.0, help="gripper OPEN target (0..100)")
    ap.add_argument("--grip-close", type=float, default=30.0,
                    help="gripper CLOSE target (0..100). LOWER = closes harder. Raise this "
                         "if the gripper trips an overload when grabbing.")
    a = ap.parse_args()
    if a.mode == "record":
        record(a.port, a.arm_id, a.grip_open, a.grip_close)
    else:
        replay(a.port, a.arm_id)


if __name__ == "__main__":
    main()