import json
from http.client import HTTPConnection

from rodi_transport import RoDITransport

class HttpRoDITransport(RoDITransport):

    def connect(self):
        try:
            self.conn = HTTPConnection(self.hostname, port=self.port, timeout=100)
        except Exception as e:
            print("the HTTP request failed: " + str(e))

    def disconnect(self):
        try:
            self.conn.close()
        except Exception as e:
            print("the HTTP request failed: " + str(e))

    def send_command(self, parameters, expected_numbers=0, bytes_per_number=1):
        request = "/" + "/".join(map(str, parameters))
        try:
            self.conn.request("GET", request)
            response = self.conn.getresponse().read()
            if expected_numbers > 0:
                contents = response.decode()
                result = json.loads(contents)
                if not isinstance(result, list):
                    result = [result]
                return result
        except Exception as e:
            print("HTTP request failed: " + str(e))
