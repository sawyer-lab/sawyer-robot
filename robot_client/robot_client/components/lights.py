"""
Lights — Sawyer robot lights component.
"""


class Lights:
    """Controls the Sawyer lights. Obtained via ``robot.lights``."""

    def __init__(self, client) -> None:
        self._client = client

    def list_all(self) -> list:
        """List all available light names."""
        return self._client.lights_list()

    def set(self, name: str, on: bool = True) -> bool:
        """Turn a light on or off."""
        return self._client.lights_set(name, on)

    def get(self, name: str) -> bool:
        """Return the current state of a light."""
        return self._client.lights_get(name)
