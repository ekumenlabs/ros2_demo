#!/usr/bin/env python3

from http.client import HTTPConnection


class Transport(object):

    def __init__(self):
        self.hostname = '192.168.4.1'
        self.port = '1234'

    def send_command(self, params):
        request = "/" + "/".join(map(str, params))

        try:
            self.conn = HTTPConnection(self.hostname, port=self.port)
            self.conn.request("GET", request)
            response = self.conn.getresponse().read()
            self.conn.close()
            return response
        except Exception as e:
            print("the HTTP request failed: " + str(e))
            return 0

    def move_forward(self):
        self.send_command([3, 100, 100])

    def move_reverse(self):
        self.send_command([3, -100, -100])

    def move_left(self):
        self.send_command([3, -100, 100])

    def move_right(self):
        self.send_command([3, 100, -100])

    def stop(self):
        self.send_command([3, 0, 0])

    def see(self):
        return self.send_command([5])
