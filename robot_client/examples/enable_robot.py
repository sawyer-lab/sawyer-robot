#!/usr/bin/env python3
"""
enable_robot.py — Enable, disable, and reset the robot.

Usage:
    e  enable
    d  disable
    r  reset
    s  status
    q  quit
"""

from sawyer_robot import SawyerRobot


def main() -> None:
    print("Robot Enable Control  (e=enable  d=disable  r=reset  s=status  q=quit)")
    print("=" * 65)

    with SawyerRobot() as robot:
        while True:
            cmd = input("
> ").strip().lower()

            if cmd == "e":
                ok = robot.enable()
                print("enable:", "OK" if ok else "FAIL")

            elif cmd == "d":
                ok = robot.disable()
                print("disable:", "OK" if ok else "FAIL")

            elif cmd == "r":
                ok = robot.reset()
                print("reset:", "OK" if ok else "FAIL")

            elif cmd == "s":
                status = robot.get_robot_status()
                if not status:
                    print("  status: unavailable")
                else:
                    print(f"  enabled      : {status.get('enabled')}")
                    print(f"  stopped      : {status.get('stopped')}")
                    print(f"  error        : {status.get('error')}")
                    print(f"  estop_button : {status.get('estop_button')}")
                    print(f"  estop_source : {status.get('estop_source')}")

            elif cmd == "q":
                break

            else:
                print("unknown command")


if __name__ == "__main__":
    main()
