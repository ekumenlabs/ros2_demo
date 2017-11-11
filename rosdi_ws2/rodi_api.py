import socket
import json
import sys
import time

class RodiAPI(object):

    def __init__(self, transport_class):
        self._transport = transport_class('192.168.4.1', 1234)

    def connect(self):
        self._transport.connect()

    def disconnect(self):
        self._transport.disconnect()

    def move(self, left_speed, right_speed):
        self._send_command(3, left_speed, right_speed)

    def move_forward(self):
        self.move(100, 100)

    def move_backward(self):
        self.move(-100, -100)

    def move_left(self):
        self.move(-100, 100)

    def move_right(self):
        self.move(100, -100)

    def stop(self):
        self.move(0, 0)

    def turn_on(self):
        self._set_led(1)

    def turn_off(self):
        self._set_led(0)

    # Not yet supported on both transports
    # Implemented as no-ops

    def blink(self, rate):
        pass
    #     self._send_command(1, rate)

    def sing(self, note, duration):
        pass
    #    self._send_command(4, note, duration)

    def set_pixel(self, red, green, blue):
        pass
    #    self._send_command(6, red, green, blue)

    def see(self):
        return self._send_command(5, expected_numbers=1, bytes_per_number=1)[0]

    def sense_light(self):
        return self._send_command(7, expected_numbers=1, bytes_per_number=2)[0]

    def sense_ground(self):
        return self._send_command(2, expected_numbers=2, bytes_per_number=2)

    def _set_led(self, state):
        pass
    #    self._send_command(8, state)

    def _send_command(self, *args, expected_numbers=0, bytes_per_number=1):
        r = self._transport.send_command(args, expected_numbers, bytes_per_number)
        return r
