# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rodi_node import RoDINode
from http_rodi_transport import HttpRoDITransport
from socket_rodi_transport import SocketRoDITransport

def main(args=None):
    rclpy.init(args=args)

    rodi_node = RoDINode(HttpRoDITransport)
    rodi_node.start_polling(rclpy)

if __name__ == '__main__':
    main()
