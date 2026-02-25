from typing import Dict
from ..protocols.zmq import ZMQClient

class Cuff:
    """Sawyer Arm Cuff buttons."""

    def __init__(self, client: ZMQClient) -> None:
        self._client = client

    def get_state(self, side: str = "right") -> Dict:
        """
        Get current state of cuff buttons.
        Returns dict with: lower, upper, squeeze
        """
        response = self._client._send_command('cuff_get_state', side=side)
        if response.get('status') == 'ok':
            return response.get('state', {})
        return {}
