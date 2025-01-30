1. Mkdir intro_task
2. cd ~/intro_task
3. git clone https://github.com/youssefdammak/Space-Concordia-Intro-Task src
4. colcon build
5. ign gazebo -v 4 -r rover.sdf
6. source install/setup.bash
7. ros2 run move_rover_package cpp_executable
8. in another terminal: source install/setup.bash
9. ros2 run move_rover_package py_server.py
10. in one terminal write :ros2 run ros_ign_bridge parameter_bridge /lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan
11. in another terminal write: ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist
12. in another terminal write: ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry
