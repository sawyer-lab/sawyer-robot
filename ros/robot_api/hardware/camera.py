#!/usr/bin/env python3
import rospy
import cv2
import json
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from intera_core_msgs.msg import IOComponentCommand, IODeviceStatus

CAMERA_TOPIC_MAP = {
    'head': 'head_camera',
    'hand': 'right_hand_camera',
}

# Cognex hand camera only — controls via IO device interface
_COGNEX_CONTROLS = ('set_strobe', 'set_exposure', 'set_gain')


class Camera:
    def __init__(self, camera_name='head_camera'):
        self.camera_name = camera_name
        self.bridge = CvBridge()
        self.current_image = None
        self._streaming = False
        self._is_cognex = (camera_name == 'right_hand_camera')
        self._io_state = {}

        # Image subscriber
        topic = f"/io/internal_camera/{camera_name}/image_raw"
        self.image_sub = rospy.Subscriber(
            topic,
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=2**24
        )

        # IO command publisher — used to start/stop streaming AND for Cognex controls
        io_path = f"io/internal_camera/{camera_name}"
        self._io_cmd_pub = rospy.Publisher(
            f"/{io_path}/command", IOComponentCommand, queue_size=10)
        self._io_state_sub = rospy.Subscriber(
            f"/{io_path}/state", IODeviceStatus, self._io_state_cb)

        rospy.sleep(0.5)
        rospy.loginfo(f"Camera: {camera_name} ready, subscribed to {topic}")

    # ------------------------------------------------------------------ #
    # Image

    def start_streaming(self):
        """Tell the camera hardware to start publishing images."""
        self._set_io_signal('camera_streaming', 'bool', True)
        self._streaming = True
        return True

    def stop_streaming(self):
        """Tell the camera hardware to stop publishing images."""
        self._set_io_signal('camera_streaming', 'bool', False)
        self._streaming = False
        return True

    def get_state(self):
        state = {
            'camera': self.camera_name,
            'streaming': self._streaming,
            'has_image': self.current_image is not None,
            'topic': f"/io/internal_camera/{self.camera_name}/image_raw",
        }
        if self._is_cognex:
            state['strobe'] = self._io_state.get('set_strobe', False)
            state['exposure'] = self._io_state.get('set_exposure', None)
            state['gain'] = self._io_state.get('set_gain', None)
        return state

    def get_image(self):
        if self.current_image is not None:
            return self.current_image.copy()
        rospy.logwarn_throttle(2.0, "Camera: No image received yet")
        return None

    def get_image_compressed(self):
        img = self.get_image()
        if img is not None:
            _, buffer = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])
            return buffer.tobytes()
        return None

    def _image_callback(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Camera: Failed to convert image: {e}")

    # ------------------------------------------------------------------ #
    # Cognex controls (hand camera only)

    def set_strobe(self, on: bool) -> bool:
        return self._set_io_signal('set_strobe', 'bool', on, cognex_only=True)

    def set_exposure(self, value: float) -> bool:
        """Exposure: 0.01 – 100.0"""
        return self._set_io_signal('set_exposure', 'float', float(value), cognex_only=True)

    def set_gain(self, value: int) -> bool:
        """Gain: 0 – 255"""
        return self._set_io_signal('set_gain', 'int', int(value), cognex_only=True)

    def _set_io_signal(self, name: str, dtype: str, value, cognex_only: bool = False) -> bool:
        if cognex_only and not self._is_cognex:
            rospy.logwarn(f"Camera: '{name}' only supported on Cognex hand camera")
            return False
        cmd = IOComponentCommand()
        cmd.time = rospy.Time.now()
        cmd.op = 'set'
        cmd.args = json.dumps({
            'signals': {name: {'format': {'type': dtype}, 'data': [value]}}
        })
        self._io_cmd_pub.publish(cmd)
        return True

    def _io_state_cb(self, msg: IODeviceStatus):
        for sig in msg.signals:
            if sig.name in _COGNEX_CONTROLS:
                try:
                    data = json.loads(sig.data)
                    self._io_state[sig.name] = data[0] if data else None
                except Exception:
                    pass
