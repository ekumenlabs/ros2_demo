class RoDITransport(object):

    def __init__(self, hostname, port):
        self.hostname = hostname
        self.port = port

    def connect(self):
        raise NotImplementedError

    def disconnect(self):
        raise NotImplementedError

    def send_command(self, parameters, expected_numbers=0, bytes_per_number=1):
        raise NotImplementedError

