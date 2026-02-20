#!/usr/bin/env python3
"""
camera.py — Live camera viewer.

Choose head or hand camera, stream frames in a window.

    s  save current frame to /tmp/sawyer_<camera>_XXXX.jpg
    q  quit
"""

import sys
import time

import cv2

from sawyer_robot import SawyerRobot
from sawyer_robot.components.camera import BoundCamera


def pick_camera(robot: SawyerRobot) -> BoundCamera:
    print("Which camera?")
    print("  [1] head  — head-mounted")
    print("  [2] hand  — wrist / end-effector")
    while True:
        choice = input("\n> ").strip().lower()
        if choice in ("1", "head"):
            return robot.head_camera
        if choice in ("2", "hand"):
            return robot.hand_camera
        print("  enter 1 or 2")


def main() -> None:
    print("Camera Viewer  (s=save  q=quit)")
    print("=" * 40)

    with SawyerRobot() as robot:
        cam = pick_camera(robot)
        label = cam._camera.value

        state = cam.get_state()
        if state is None:
            print("ERROR: cannot reach ZMQ server.")
            sys.exit(1)

        cam.start()
        print(f"\nStreaming '{label}' camera — waiting for first frame...")

        for _ in range(30):
            time.sleep(0.1)
            s = cam.get_state()
            if s and s.get("has_image"):
                break

        save_count = 0
        last_frame = None

        try:
            while True:
                frame = cam.get_image()
                if frame is not None:
                    last_frame = frame
                    cv2.imshow(f"Sawyer — {label}", frame)

                key = cv2.waitKey(30) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("s") and last_frame is not None:
                    path = f"/tmp/sawyer_{label}_{save_count:04d}.jpg"
                    cv2.imwrite(path, last_frame)
                    print(f"  saved: {path}")
                    save_count += 1
        except KeyboardInterrupt:
            print("\nInterrupted.")

        cv2.destroyAllWindows()
        cam.stop()

    print("Done.")


if __name__ == "__main__":
    main()
