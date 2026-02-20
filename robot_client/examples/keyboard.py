#!/usr/bin/env python3
"""
keyboard.py — Keyboard joint control.

    1 / q   j0  +/-
    2 / w   j1  +/-
    3 / e   j2  +/-
    4 / r   j3  +/-
    5 / t   j4  +/-
    6 / y   j5  +/-
    7 / u   j6  +/-
    o / c   gripper open / close
    h       home (all zeros)
    x       exit
"""

import sys
import termios
import tty

from sawyer_robot import SawyerRobot, JointAngles


STEP = 0.05

BINDINGS: dict[str, tuple[str, int, float]] = {
    "1": ("j", 0, +STEP), "q": ("j", 0, -STEP),
    "2": ("j", 1, +STEP), "w": ("j", 1, -STEP),
    "3": ("j", 2, +STEP), "e": ("j", 2, -STEP),
    "4": ("j", 3, +STEP), "r": ("j", 3, -STEP),
    "5": ("j", 4, +STEP), "t": ("j", 4, -STEP),
    "6": ("j", 5, +STEP), "y": ("j", 5, -STEP),
    "7": ("j", 6, +STEP), "u": ("j", 6, -STEP),
}


def get_key() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def print_joints(joints: JointAngles) -> None:
    fields = ("j0", "j1", "j2", "j3", "j4", "j5", "j6")
    line = "  ".join(f"{f}:{getattr(joints, f):6.3f}" for f in fields)
    print(f"\r{line}  ", end="", flush=True)


def main() -> None:
    print("=== Keyboard Joint Control ===")
    print("1q 2w 3e 4r 5t 6y 7u  — joints 0-6  +/-")
    print("o / c                  — gripper open / close")
    print("h                      — home position")
    print("x                      — exit\n")

    with SawyerRobot() as robot:
        joints = robot.arm.get_joints()
        print_joints(joints)

        try:
            while True:
                key = get_key()

                if key == "x":
                    break

                elif key in BINDINGS:
                    _, idx, delta = BINDINGS[key]
                    field = f"j{idx}"
                    setattr(joints, field, getattr(joints, field) + delta)
                    robot.arm.move(joints)

                elif key == "o":
                    robot.gripper.open()
                    print("\n  gripper: OPEN")

                elif key == "c":
                    robot.gripper.close()
                    print("\n  gripper: CLOSE")

                elif key == "h":
                    joints = JointAngles()
                    robot.arm.move(joints)
                    print("\n  home")

                else:
                    continue

                print_joints(joints)
        except KeyboardInterrupt:
            print("\n\nInterrupted.")

    print("\n\nDone.")


if __name__ == "__main__":
    main()
