#!/usr/bin/env python3
import json
import rospy
from intera_core_msgs.msg import IOComponentCommand, IODeviceStatus

# All lights available on Sawyer
LIGHT_NAMES = [
    'head_red_light',
    'head_green_light',
    'head_blue_light',
    'right_hand_red_light',
    'right_hand_green_light',
    'right_hand_blue_light',
]


class Lights:
    def __init__(self):
        self._state = {}  # signal_name -> bool

        self._cmd_pub = rospy.Publisher(
            '/io/robot/robot/command', IOComponentCommand, queue_size=10)
        self._state_sub = rospy.Subscriber(
            '/io/robot/robot/state', IODeviceStatus, self._state_cb)

        # Wait for first state message
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while not self._state and not rospy.is_shutdown():
            rospy.sleep(0.1)
            if rospy.Time.now() > deadline:
                rospy.logwarn("Lights: timed out waiting for /io/robot/robot/state")
                break

        rospy.loginfo("Lights: initialized — signals: %s", list(self._state.keys()))

    # ------------------------------------------------------------------ #

    def list_all_lights(self):
        """Return names of all light signals seen on the robot."""
        return [n for n in self._state.keys() if 'light' in n]

    def set_light_state(self, name, on: bool = True) -> bool:
        """Turn a named light on (True) or off (False)."""
        cmd = IOComponentCommand()
        cmd.time = rospy.Time.now()
        cmd.op = 'set'
        cmd.args = json.dumps({
            'signals': {
                name: {
                    'format': {'type': 'bool'},
                    'data': [on],
                }
            }
        })
        self._cmd_pub.publish(cmd)
        return True

    def get_light_state(self, name) -> bool:
        """Return current known state of a light (True = on)."""
        return self._state.get(name, False)

    # ------------------------------------------------------------------ #

    def _state_cb(self, msg: IODeviceStatus):
        for sig in msg.signals:
            try:
                data = json.loads(sig.data)
                self._state[sig.name] = bool(data[0]) if data else False
            except Exception:
                pass
