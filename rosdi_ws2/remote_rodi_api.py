import socket
import json
import sys
import time

class RemoteRodiAPI(object):

    def __init__(self):
        self.hostname = '192.168.4.1'
        self.port = 1234
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                
    def connect(self):
        self.socket.connect((self.hostname,self.port))
    
    def disconnect(self):
        self.socket.close()
    
    def blink(self, rate):
        # TODO
        pass
        
    def move(self, left_speed, right_speed):
        self._send_command(55, left_speed, right_speed)

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
        return self._send_command(4, packet = 1)[0]

    def sing(self, note, duration):
        self._send_command(56, note)
        time.sleep(duration)
        self._send_command(57)
 
    def set_pixel(self, red, green, blue):
        # TODO
        pass

    def sense_light(self):
        return self._send_command(5, packet = 1, expectingBytes = 2)[0]

    def sense_ground(self):
        return self._send_command(1, packet = 2, expectingBytes = 2)

    def sense_ground_left(self):
        return self._send_command(2, packet = 1, expectingBytes = 2)[0]
    
    def sense_ground_right(self):
        return self._send_command(3, packet = 1, expectingBytes = 2)[0]

    def turn_on(self):
        self._set_led(51)

    def turn_off(self):
        self._set_led(52)

    def _set_led(self, state):
        self._send_command(state)

    def _send_command(self, *args, packet = 0, expectingBytes = 1):
        bytes = map(lambda byte:byte.to_bytes(1, byteorder='big', signed=True), args)
        self._send_bytes(bytes)
        if packet != 0 :
            return list(map(lambda byte:int.from_bytes(byte, byteorder='big'), self._recive_bytes(packet, expectingBytes)))

    def _send_bytes(self, bytes):
        for byte in bytes:
            sent = self.socket.send(byte)
            if sent == 0:
                raise Exception("Connection fail")

    def _recive_bytes(self, packet, expectingBytes):
        bytes = []
        while len(bytes) < packet:
            byte = self.socket.recv(expectingBytes)
            if byte == None:
                raise Exception('Connection fail')
            bytes.append(byte)
        return bytes