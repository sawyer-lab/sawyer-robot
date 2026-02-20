#!/usr/bin/env python3
"""
gripper.py — Interactive gripper control.

    o  open
    c  close
    s  show state
    q  quit
"""

from robot_client import SawyerRobot


def main() -> None:
    print("Gripper Control  (o=open  c=close  s=state  q=quit)")
    print("=" * 50)

    with SawyerRobot() as robot:
        while True:
            cmd = input("\n> ").strip().lower()

            if cmd == "o":
                ok = robot.gripper.open()
                print("open:", "OK" if ok else "FAIL")

            elif cmd == "c":
                ok = robot.gripper.close()
                print("close:", "OK" if ok else "FAIL")

            elif cmd == "s":
                s = robot.gripper.state
                print(f"  position    : {s.position:.4f} m")
                print(f"  is_grasping : {s.is_grasping}")
                print(f"  state       : {s.state.value}")
                print(f"  device      : {s.device}")

            elif cmd == "q":
                break

            else:
                print("unknown command")


if __name__ == "__main__":
    main()
