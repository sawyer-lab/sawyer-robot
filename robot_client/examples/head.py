#!/usr/bin/env python3
"""
head.py — Head pan and display control.

    l  pan left   (+0.2 rad)
    r  pan right  (-0.2 rad)
    c  pan centre (0.0 rad)
    d  display a test image on the head screen
    x  clear display
    s  show current pan angle
    q  quit
"""

import math

import cv2
import numpy as np

from sawyer_robot import SawyerRobot

PAN_STEP = 0.2


def make_test_image(text: str, size: tuple = (1024, 600)) -> np.ndarray:
    w, h = size
    img = np.zeros((h, w, 3), dtype=np.uint8)
    # Blue→green gradient
    for x in range(w):
        t = x / w
        img[:, x] = (int(255 * (1 - t)), int(255 * t), 0)
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale, thickness = 2.0, 3
    (tw, th), _ = cv2.getTextSize(text, font, scale, thickness)
    cv2.putText(img, text, ((w - tw) // 2, (h + th) // 2),
                font, scale, (255, 255, 255), thickness, cv2.LINE_AA)
    return img


def main() -> None:
    print("Head Control  (l=left  r=right  c=centre  d=display  x=clear  s=state  q=quit)")
    print("=" * 60)

    with SawyerRobot() as robot:
        try:
            while True:
                cmd = input("\n> ").strip().lower()

                if cmd == "l":
                    angle = robot.head.pan + PAN_STEP
                    robot.head.set_pan(angle)
                    print(f"  pan → {math.degrees(robot.head.pan):.1f}°")

                elif cmd == "r":
                    angle = robot.head.pan - PAN_STEP
                    robot.head.set_pan(angle)
                    print(f"  pan → {math.degrees(robot.head.pan):.1f}°")

                elif cmd == "c":
                    robot.head.set_pan(0.0)
                    print("  centred")

                elif cmd == "d":
                    img = make_test_image("Sawyer")
                    robot.head.display_image(img)
                    print("  image sent to head display")

                elif cmd == "x":
                    robot.head.display_clear()
                    print("  display cleared")

                elif cmd == "s":
                    angle = robot.head.pan
                    print(f"  pan: {angle:.4f} rad  ({math.degrees(angle):.1f}°)")

                elif cmd == "q":
                    break

                else:
                    print("  unknown command")
        except (EOFError, KeyboardInterrupt):
            print()

    print("Done.")


if __name__ == "__main__":
    main()
