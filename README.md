# ros2_demo
Ekuthon multirobot ros2_demo

## Building

1. Create a workspace folder for the repo:

    ```
    cd ~
    mkdir -p rodi_ros2_ws/src
    ```

2. Clone the repository

    ```
    git clone https://github.com/ekumenlabs/ros2_demo.git
    ```

3. For development we can build the package using symlinks by running:

    ```
    cd ~/rodi_ros2_ws
    ament build --symlink-install
    ```

## Running

1. Source the workspace

    ```
    cd ~/rodi_ros2_ws
    source install/setup.bash
    ```

2. Run RoDI node

    ```
    ros2 run rosdi_ws2 rosdi_ws
    ```

3. In another terminal, send a velocity command:

    ```
    ros2 topic pub  /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}'
    ```
    Congrats! Your RoDI should start rotation anti-clockwise.


## Testing obstacles avoider demo

1. Source the workspace

    ```
    cd ~/rodi_ros2_ws
    source install/setup.bash
    ```

2. Launch the obstacles avoider launchfile

    ```
    launch `ros2 pkg prefix rosdi_ws2`/share/rosdi_ws2/launch/obstacles_avoider_launcher.py
    ```

    Your RoDI should go in a straight line and rotate if there's an obstacle at 0.2 or less from it.
