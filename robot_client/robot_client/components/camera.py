"""
BoundCamera — camera pre-bound to a specific Camera enum value.
"""

from typing import Optional
import numpy as np

from ..sawyer import Camera


class BoundCamera:
    """
    A camera whose identity (HEAD or HAND) is fixed at construction.
    Obtained via ``robot.head_camera`` or ``robot.hand_camera``.
    """

    def __init__(self, client, camera: Camera) -> None:
        self._client = client
        self._camera = camera

    def start(self) -> bool:
        """Start streaming this camera."""
        return self._client.camera_start(camera=self._camera.value)

    def stop(self) -> bool:
        """Stop streaming this camera."""
        return self._client.camera_stop(camera=self._camera.value)

    def get_image(self) -> Optional[np.ndarray]:
        """
        Return the latest frame.

        Returns numpy array (H, W, 3) BGR, or None if unavailable.
        """
        return self._client.camera_get_image(camera=self._camera.value)

    def get_state(self) -> Optional[dict]:
        """Return camera state dict (streaming, has_image, topic, strobe, exposure, gain)."""
        return self._client.camera_get_state(camera=self._camera.value)

    # ------------------------------------------------------------------ #
    # Cognex hand camera controls (no-op on head camera)

    def set_strobe(self, on: bool) -> bool:
        """Enable or disable the Cognex strobe flash."""
        return self._client.camera_set_strobe(on, camera=self._camera.value)

    def set_exposure(self, value: float) -> bool:
        """Set exposure (0.01 – 100.0). Cognex hand camera only."""
        return self._client.camera_set_exposure(value, camera=self._camera.value)

    def set_gain(self, value: int) -> bool:
        """Set gain (0 – 255). Cognex hand camera only."""
        return self._client.camera_set_gain(value, camera=self._camera.value)
