from typing import Dict
from ..protocols.zmq import ZMQClient

class Navigator:
    """Sawyer Navigator buttons and scroll wheel."""

    def __init__(self, client: ZMQClient) -> None:
        self._client = client

    def get_state(self, side: str = "right") -> Dict:
        """
        Get current state of all buttons and wheel.
        Returns dict with: ok, back, show, triangle, square, circle, wheel
        """
        response = self._client._send_command('navigator_get_state', side=side)
        if response.get('status') == 'ok':
            return response.get('state', {})
        return {}
