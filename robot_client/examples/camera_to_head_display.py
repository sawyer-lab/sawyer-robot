#!/usr/bin/env python3
"""camera_to_head_display.py — Stream a camera to the robot's head display.

Choose head or hand camera, see it live on the robot screen.
Press Ctrl-C to stop.
"""

import signal
import sys
import time

import cv2

from robot_client import SawyerRobot

_running = True


def _stop(sig, frame):
    global _running
    _running = False


def pick_camera(robot):
    print("Which camera to stream to the head display?")
    print("  [1] head  — head-mounted (1280×800)")
    print("  [2] hand  — wrist / Cognex (752×480)")
    while True:
        choice = input("\n> ").strip().lower()
        if choice in ("1", "head"):
            return robot.head_camera, "head"
        if choice in ("2", "hand"):
            return robot.hand_camera, "hand"
        print("  enter 1 or 2")


def main() -> None:
    print("Camera → Head Display  (Ctrl-C to stop)")
    print("=" * 42)

    signal.signal(signal.SIGINT, _stop)

    with SawyerRobot() as robot:
        cam, label = pick_camera(robot)

        if cam.get_state() is None:
            print("ERROR: cannot reach ZMQ server.")
            sys.exit(1)

        cam.start()
        print(f"\nStarted '{label}' camera — waiting for first frame...")

        for i in range(50):
            time.sleep(0.1)
            if (cam.get_state() or {}).get("has_image"):
                print("  ✓ receiving frames — streaming to head display")
                break
            if i == 49:
                print("  ✗ no image after 5 s — check camera topic")
                sys.exit(1)

        print("  Press Ctrl-C to stop.\n")

        frame_count = 0
        while _running:
            frame = cam.get_image()
            if frame is not None:
                display = cv2.resize(frame, (1024, 600))
                robot.head.display_image(display)
                frame_count += 1
                if frame_count % 30 == 1:
                    sys.stdout.write(f"\r  [{label}] frames sent: {frame_count}")
                    sys.stdout.flush()
            time.sleep(0.033)  # ~30 fps

        robot.head.display_clear()
        cam.stop()
        print(f"\nDone. {frame_count} frames sent.")


if __name__ == "__main__":
    main()



if __name__ == "__main__":
    main()
