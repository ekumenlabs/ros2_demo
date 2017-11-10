import rclpy
import time
import math

from remote_rodi_api import RemoteRodiAPI

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, Illuminance
from rodi_msgs.msg import GroundReflectance

class RODINode(rclpy.Node):

    def __init__(self):
        super().__init__('RODINode')
        self._ultrasound_publisher = self.create_publisher(Range, 'ultrasound')
        self._illuminance_publisher = self.create_publisher(Illuminance, 'illuminance')
        self._ground_reflectance_publisher = self.create_publisher(GroundReflectance, 'ground_reflectance')
        self._api = RemoteRodiAPI()

    def start_polling(self, runner):
        self._api.connect()
        self._setup_subscribers()

        self._last_update = int(round(time.time() * 1000))

        while runner.ok():
            self._poll_sensors()
            runner.spin_once(self, timeout_sec=0.05)
            self._reset_rodi_state_on_timeout()

    def _setup_subscribers(self):
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self._on_cmd_vel_update)
        assert self.cmd_vel_subscription

    def _reset_rodi_state_on_timeout(self):
        if (self._last_update < int(round(time.time() * 1000)) - 1000):
            self._api.stop()

    def _on_cmd_vel_update(self, msg):
        self._last_update = int(round(time.time() * 1000))
        # TODO: Implement this properly
        if msg.angular.z == 0 and msg.linear.x == 0:
            self._api.stop()
        elif msg.linear.x > 0:
            self._api.move_forward()
        elif msg.linear.x < 0:
            self._api.move_reverse()
        elif msg.angular.z > 0:
            self._api.move_left()
        elif msg.angular.z < 0:
            self._api.move_right()

    def _poll_sensors(self):
        self._poll_ultrasound_sensor()
        self._poll_illuminance_sensor()
        self._poll_ground_reflectance_sensor()

    def _poll_ultrasound_sensor(self):
        sensed_value = float(self._api.see())

        range_message = Range()
        range_message.radiation_type = 0  # ULTRASOUND
        range_message.header.frame_id = "/ultrasound"
        range_message.field_of_view = 0.52
        range_message.min_range = 0.2
        range_message.max_range = 1.0
        range_message.range = sensed_value

        self._ultrasound_publisher.publish(range_message)

    def _poll_illuminance_sensor(self):
        sensed_value = float(self._api.sense_light())
        photometric_illuminance = 18.61605717 * math.exp(0.0106241 * sensed_value)

        illuminance_message = Illuminance()
        illuminance_message.header.frame_id = "/illuminance"
        illuminance_message.variance = 0.0 # Unknown
        illuminance_message.illuminance = photometric_illuminance

        self._illuminance_publisher.publish(illuminance_message)

    def _poll_ground_reflectance_sensor(self):
        reflectance = self._api.sense_ground()

        reflectance_message = GroundReflectance()
        reflectance_message.header.frame_id = "/reflectance"
        reflectance_message.left = reflectance[0]
        reflectance_message.right = reflectance[1]

        self._ground_reflectance_publisher.publish(reflectance_message)
