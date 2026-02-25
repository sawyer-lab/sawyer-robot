#!/usr/bin/env python3
"""
Hardware interface for the Sawyer Arm Cuff buttons.
"""

import rospy
from .io_interface import IODeviceInterface

class Cuff:
    """Interface for Sawyer Cuff buttons."""

    def __init__(self):
        # Device name 'cuff' under node 'robot'
        self._io = IODeviceInterface("robot", "cuff")
        
    def get_state(self, side="right"):
        """Get cuff button states for a given side."""
        # Signal names: <side>_button_lower, <side>_button_upper, <side>_cuff
        # Val: 1 for pressed, 0 for unpressed
        return {
            'lower': bool(self._io.get_signal_value(f"{side}_button_lower")),
            'upper': bool(self._io.get_signal_value(f"{side}_button_upper")),
            'squeeze': bool(self._io.get_signal_value(f"{side}_cuff"))
        }

    def list_signals(self):
        return self._io.list_signal_names()
