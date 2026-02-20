#!/usr/bin/env python3
"""hand_camera_settings.py — Adjust strobe, exposure and gain on the hand camera.

Shows current settings, lets you toggle strobe and set exposure/gain,
then captures a frame to /tmp/ so you can compare results.

    s          toggle strobe on/off
    e <value>  set exposure  (0.01 – 100.0, e.g.  e 10.5)
    g <value>  set gain      (0 – 255,       e.g.  g 64)
    c          capture one frame to /tmp/sawyer_hand_capture.jpg
    q          quit
"""

import sys
import time

import cv2

from sawyer_robot import SawyerRobot


def print_state(cam):
    s = cam.get_state() or {}
    strobe = "● ON" if s.get("strobe") else "○ off"
    exp = s.get("exposure", "?")
    gain = s.get("gain", "?")
    print(f"  strobe={strobe}  exposure={exp}  gain={gain}")


def main() -> None:
    print("Hand Camera Settings  (s / e <val> / g <val> / c / q)")
    print("=" * 54)

    with SawyerRobot() as robot:
        cam = robot.hand_camera

        if cam.get_state() is None:
            print("ERROR: cannot reach ZMQ server.")
            sys.exit(1)

        cam.start()

        # Wait for first frame
        for _ in range(30):
            time.sleep(0.1)
            if (cam.get_state() or {}).get("has_image"):
                break

        print("\nCurrent settings:")
        print_state(cam)
        print()

        capture_count = 0

        while True:
            try:
                line = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if not line:
                continue

            parts = line.split(None, 1)
            cmd = parts[0].lower()

            if cmd in ("q", "quit"):
                break

            elif cmd == "s":
                # toggle strobe
                current = (cam.get_state() or {}).get("strobe", False)
                cam.set_strobe(not current)
                time.sleep(0.1)
                print_state(cam)

            elif cmd == "e" and len(parts) == 2:
                try:
                    val = float(parts[1])
                    if not 0.01 <= val <= 100.0:
                        raise ValueError
                    cam.set_exposure(val)
                    time.sleep(0.1)
                    print_state(cam)
                except ValueError:
                    print("  exposure must be 0.01 – 100.0")

            elif cmd == "g" and len(parts) == 2:
                try:
                    val = int(parts[1])
                    if not 0 <= val <= 255:
                        raise ValueError
                    cam.set_gain(val)
                    time.sleep(0.1)
                    print_state(cam)
                except ValueError:
                    print("  gain must be 0 – 255")

            elif cmd == "c":
                frame = cam.get_image()
                if frame is None:
                    print("  no frame available yet")
                else:
                    path = f"/tmp/sawyer_hand_capture_{capture_count:03d}.jpg"
                    cv2.imwrite(path, frame)
                    capture_count += 1
                    print(f"  saved: {path}  ({frame.shape[1]}×{frame.shape[0]})")

            else:
                print("  s | e <val> | g <val> | c | q")

        cam.stop()

    print("Done.")


if __name__ == "__main__":
    main()
