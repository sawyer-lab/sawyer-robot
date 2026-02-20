"""
Head — Sawyer head pan and display component.
"""

import numpy as np


class Head:
    """Controls the Sawyer head. Obtained via ``robot.head``."""

    def __init__(self, client) -> None:
        self._client = client

    @property
    def pan(self) -> float:
        """Current head pan angle in radians."""
        return self._client.head_pan()

    def set_pan(self, angle: float, speed: float = 1.0) -> bool:
        """Set head pan angle (radians)."""
        return self._client.head_set_pan(angle, speed)

    def display_image(self, image: np.ndarray) -> bool:
        """Display a BGR image (H, W, 3) on the head screen."""
        return self._client.display_image(image)

    def display_clear(self) -> bool:
        """Clear the head display."""
        return self._client.display_clear()
