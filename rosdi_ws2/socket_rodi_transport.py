import socket

from rodi_transport import RoDITransport

class SocketRoDITransport(RoDITransport):

    def connect(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.hostname, self.port))

    def disconnect(self):
        self.socket.close()

    def send_command(self, parameters, expected_numbers, bytes_per_number):
        bytes = map(lambda byte: byte.to_bytes(2, byteorder='big', signed=True), parameters)
        self._send_bytes(bytes)
        if expected_numbers != 0 :
            response = self._recive_bytes(expected_numbers, bytes_per_number)
            return list(map(lambda byte: int.from_bytes(byte, byteorder='big'), response))

    def _send_bytes(self, bytes):
        for byte in bytes:
            sent = self.socket.send(byte)
            if sent == 0:
                raise Exception("Connection failed")

    def _recive_bytes(self, expected_numbers, bytes_per_number):
        bytes = []
        while len(bytes) < expected_numbers:
            byte = self.socket.recv(bytes_per_number)
            if byte == None:
                raise Exception('Connection failed')
            bytes.append(byte)
        return bytes
