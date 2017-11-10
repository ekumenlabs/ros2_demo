#!/usr/bin/env python3

from http.client import HTTPConnection


class RemoteRodiAPI(object):

    def __init__(self):
        self.hostname = '192.168.4.1'
        self.port = '1234'
        
    def connect(self):
        try:
            self.conn = HTTPConnection(self.hostname, port=self.port)
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

    def see(self):
        return self._send_command(5)

    def sing(self, note, duration):
        self._send_command(4, note, duration)
 
    def set_pixel(self, red, green, blue):
        self._send_command(6, red, green, blue)

    def sense_light(self):
        return  self._send_command(7)
        
    def _send_command(self, *args):
        request = "/" + "/".join(map(str, args))

        try:
            self.conn.request("GET", request)
            response = self.conn.getresponse().read()
            return response
        except Exception as e:
            print("the HTTP request failed: " + str(e))
            return 0
            
