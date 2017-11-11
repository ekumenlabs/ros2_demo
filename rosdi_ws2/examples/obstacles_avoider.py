import rclpy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

class ObstaclesAvoiderNode(rclpy.Node):

    def __init__(self):
        super().__init__('obstacles_avoider_node')

        # Configure cmd_vel publisher
        self._cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel')
        self._configure_subscribers()


    def _configure_subscribers(self):
        self.ultrasonic_sensor_subscriber = self.create_subscription(
            Range,
            '/ultrasound',
            self._on_ultrasonic_data)
        assert self.ultrasonic_sensor_subscriber

    def _on_ultrasonic_data(self, msg):
        cmd_vel = Twist()
        if msg.range < 20.0:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 1.0
        else:
            cmd_vel.linear.x = 1.0
            cmd_vel.angular.z = 0.0
        self._cmd_vel_publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)

    obstacles_avoider_node = ObstaclesAvoiderNode()
    rclpy.spin(obstacles_avoider_node)

if __name__ == '__main__':
    main()
