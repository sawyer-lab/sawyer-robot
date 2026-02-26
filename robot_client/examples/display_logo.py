#!/usr/bin/env python3

import cv2
from sawyer_robot import SawyerRobot


def main():
    # 🔹 Hard-coded image path
    image_path = "/home/fausto/Projects/sawyer/sawyer-robot/robot_client/examples/logo2.jpg"

    # Load image
    image = cv2.imread(image_path)
    if image is None:
        print(f"Error: Could not read image at '{image_path}'")
        return

    # Resize to head display resolution
    image = cv2.resize(image, (1024, 600))

    # Display on robot head
    with SawyerRobot() as robot:
        robot.head.display_image(image)
        # print("Image displayed. Press Enter to clear.")
        # input()
        # robot.head.display_clear()


if __name__ == "__main__":
    main()