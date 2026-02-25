#!/usr/bin/env python3
"""
Hardware interface for the Sawyer Navigator buttons and scroll wheel.
"""

import rospy
from .io_interface import IODeviceInterface

class Navigator:
    """Interface for Sawyer Navigator buttons and wheel."""

    def __init__(self):
        # The 'robot' node name and 'navigator' device name match standard SDK
        self._io = IODeviceInterface("robot", "navigator")
        
        # Standard button names for Sawyer Navigator
        # Format: <assembly>_button_<function>
        # Typical assemblies are 'right' (arm) and 'head'
        self._buttons = [
            "back", "show", "ok", "triangle", "square", "circle"
        ]
        
        self._button_lookup = {
            0: 'OFF',
            1: 'CLICK',
            2: 'LONG_PRESS',
            3: 'DOUBLE_CLICK'
        }

    def get_state(self, side="all"):
        """
        Get button and wheel states. 
        If side="all", returns every available navigator signal.
        Otherwise filters by side (e.g. "right", "head").
        """
        signals = self._io.list_signal_names()
        state = {}
        
        for sig in signals:
            if side != "all" and not sig.startswith(side):
                continue
                
            val = self._io.get_signal_value(sig)
            
            # Simple heuristic: if 'wheel' in name, it's a number, otherwise it's a button
            if 'wheel' in sig:
                state[sig] = val
            else:
                state[sig] = self._button_lookup.get(val, 'UNKNOWN')
                
        return state

    def list_signals(self):
        """List all available raw signal names."""
        return self._io.list_signal_names()
