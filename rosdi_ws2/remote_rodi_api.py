#!/usr/bin/env python3

from http.client import HTTPConnection
import json

class RemoteRodiAPI(object):

    def __init__(self):
        self.hostname = '192.168.4.1'
        self.port = '1234'

    def connect(self):
        try:
            self.conn = HTTPConnection(self.hostname, port=self.port, timeout=100)
        except Exception as e:
            print("the HTTP request failed: " + str(e))
            return 0

    def disconnect(self):
        try:
            self.conn.close()
        except Exception as e:
            print("the HTTP request failed: " + str(e))
            return 0

    def blink(self, rate):
        self._send_command(1, rate)

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

    def sing(self, note, duration):
        self._send_command(4, note, duration)

    def set_pixel(self, red, green, blue):
        self._send_command(6, red, green, blue)

    def see(self):
        return int(self._send_command(5).decode())

    def sense_light(self):
        return int(self._send_command(7).decode())

    def sense_ground(self):
        return json.loads(self._send_command(2).decode())

    def _set_led(self, state):
        self._send_command(8,state)

    def _send_command(self, *args):
        request = "/" + "/".join(map(str, args))

        try:
            self.conn.request("GET", request)
            response = self.conn.getresponse().read()
            return response
        except Exception as e:
            print("the HTTP request failed: " + str(e))
            return 0
