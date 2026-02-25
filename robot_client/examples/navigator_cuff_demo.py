#!/usr/bin/env python3
"""
Navigator and Cuff Demo — Test physical buttons and scroll wheel.

This script polls the robot state, prints events, and displays the
last event on the robot's head screen.
"""

import time
import sys
import os
import cv2
import numpy as np

# Add parent dir to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from sawyer_robot import SawyerRobot

def create_status_image(text):
    """Create a simple status image for the head display."""
    img = np.zeros((600, 1024, 3), np.uint8)
    img[:] = (40, 40, 40) # Dark gray background
    cv2.putText(img, text, (50, 300), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 5)
    return img

def main():
    host = os.environ.get("ROBOT_HOST", "localhost")
    print(f"Connecting to robot at {host}...")
    
    with SawyerRobot(host=host) as robot:
        print("\n--- Navigator and Cuff Button Demo ---")
        print("Press buttons on the Navigator or Cuff to see events.")
        print("Turning the scroll wheel updates the screen too.")
        print("Press Ctrl+C to exit.\n")
        
        robot.head.display_image(create_status_image("WAITING FOR INPUT..."))

        try:
            while True:
                # 1. Get States
                nav = robot.navigator.get_state(side="all")
                cuff = robot.cuff.get_state(side="right")
                
                # Filter for active events (ONLY BUTTONS, NO WHEELS)
                active_nav = {k: v for k, v in nav.items() if v != 'OFF' and 'wheel' not in k}
                active_cuff = [k for k, v in cuff.items() if v]
                
                event_str = ""
                if active_nav:
                    event_str = f"NAV: {list(active_nav.keys())[0]} ({list(active_nav.values())[0]})"
                elif active_cuff:
                    event_str = f"CUFF: {active_cuff[0]}"

                if event_str:
                    print(f"Event: {event_str}")
                    robot.head.display_image(create_status_image(event_str))
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nExiting demo.")
            robot.head.display_clear()

if __name__ == "__main__":
    main()
