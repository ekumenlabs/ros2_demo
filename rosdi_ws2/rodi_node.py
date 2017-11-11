import sys
import rclpy
import time
import math

from rodi_api import RodiAPI

from std_msgs.msg import UInt16, Bool, ColorRGBA
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, Illuminance
from rodi_msgs.msg import GroundReflectance, Buzzer
from overridable_action import OverridableAction

class RoDINode(rclpy.Node):


    def __init__(self, transport_class, debug=True):
        super().__init__('rodi_node')
        self._ultrasound_publisher = self.create_publisher(Range, 'ultrasound')
        self._illuminance_publisher = self.create_publisher(Illuminance, 'illuminance')
        self._ground_reflectance_publisher = self.create_publisher(GroundReflectance, 'ground_reflectance')
        self._api = RodiAPI(transport_class)
        self._actions = []
        self._timeout = 1000
        self._debug = debug

    def start_polling(self, runner):
        self._api.connect()
        self._setup_subscribers()


        while runner.ok():
            loop_start = time.time()
            self._poll_sensors()
            runner.spin_once(self, timeout_sec=0.1)
            for action in self._actions:
                action.verify_override()
            loop_end = time.time()
            if self._debug:
                print('Loop time: %d ms' % int((loop_end - loop_start) * 1000))

    def _setup_subscribers(self):
        self._actions.append(OverridableAction(
            self,
            'cmd_vel',
            Twist,
            self._on_cmd_vel_update,
            self._timeout,
            lambda : self._api.stop()))

        self._actions.append(OverridableAction(
            self,
            'blink',
            UInt16,
            lambda msg: self._api.blink(msg.data),
            self._timeout,
            lambda : self._api.blink(0)))

        self._actions.append(OverridableAction(
            self,
            'led',
            Bool,
            lambda msg: self._api.turn_on() if bool(msg.data) else self._api.turn_off(),
            self._timeout,
            lambda : self._api.turn_off()))

        self._actions.append(OverridableAction(
            self,
            'pixel',
            ColorRGBA,
            lambda msg: self._api.set_pixel(int(msg.r), int(msg.g), int(msg.b)),
            self._timeout,
            lambda : self._api.set_pixel(0, 0, 0)))

        self.create_subscription(
            Buzzer,
            'buzzer',
            lambda msg: self._api.sing(msg.tone, msg.duration))

    def _on_cmd_vel_update(self, msg):
        if msg.angular.z == 0 and msg.linear.x == 0:
            self._api.stop()
        else:
            # Very rough approximation: max RoDI speed is 20cm/seg
            linear = max(min(msg.linear.x * 500, 100), -100)
            # Very rough approximation to work with ROSJava teleop
            angular = max(min(math.degrees(msg.angular.z) * 2, 100), -100)

            left = int((linear - angular) / 2.0)
            right = int((linear + angular) / 2.0)
            self._api.move(left, right)

    def _poll_sensors(self):
        sense_start = time.time()
        self._poll_ultrasound_sensor()
        self._poll_illuminance_sensor()
        self._poll_ground_reflectance_sensor()
        sense_end = time.time()
        if self._debug:
            print('Sense time: %d ms' % int((sense_end - sense_start) * 1000))


    def _poll_ultrasound_sensor(self):
        sensed_value = self._api.see()

        range_message = Range()
        range_message.radiation_type = 0  # ULTRASOUND
        range_message.header.frame_id = "/ultrasound"
        range_message.field_of_view = 0.52
        range_message.min_range = 0.2
        range_message.max_range = 1.0
        range_message.range = sensed_value / 100.0

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
