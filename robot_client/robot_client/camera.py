"""
CameraClient — Sawyer camera control, strongly typed.
"""

from typing import Optional
import numpy as np

from .base_client import BaseClient
from .sawyer import Camera


class CameraClient(BaseClient):
    """Controls Sawyer camera streaming and image capture."""

    def start(self, camera: Camera) -> bool:
        """Start streaming the given camera."""
        return self._client.camera_start(camera=camera.value)

    def stop(self, camera: Camera) -> bool:
        """Stop streaming the given camera."""
        return self._client.camera_stop(camera=camera.value)

    def get_image(self, camera: Camera) -> Optional[np.ndarray]:
        """
        Return the latest frame from the given camera.

        Returns:
            numpy array (H, W, 3) in BGR format, or None if unavailable.
        """
        return self._client.camera_get_image(camera=camera.value)

    def get_state(self, camera: Camera) -> Optional[dict]:
        """
        Return camera state dict with keys 'streaming', 'has_image', 'topic'.

        Returns None on error.
        """
        return self._client.camera_get_state(camera=camera.value)
