# This is a modified copy from https://github.com/ros2/turtlebot2_demo/blob/master/turtlebot2_teleop/launch/turtlebot_joy.py
# Attribution to Open Source Robotics Foundation, Inc.

from launch.exit_handler import default_exit_handler, restart_exit_handler
from ros2run.api import get_executable_path


def launch(launch_descriptor, argv):
    ld = launch_descriptor
    package = 'rosdi_ws2'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='obstacles_avoider')],
        name='obstacles_avoider',
        exit_handler=restart_exit_handler,
    )
    package = 'rosdi_ws2'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='rosdi_ws')],
        name='rosdi_ws',
        exit_handler=restart_exit_handler,
    )

    return ld

if __name__ == '__main__':
    launch()
