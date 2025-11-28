Micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -i 192.168.18.16

Telemetry for robot topics
ros2 run kalman_telemetry telem --ros-args -p laser_scan.lidar_model:=LDROBOT-LD19

Publish robot urdf and visualize
ros2 launch kalman_bringup inspect_urdf.launch.py joints:=none robot_model:=kalman_description

teleop
ros2 run kalman_teleop teleop_keyboard
ros2 topic echo /scan

FULL LAUNCH
ros2 launch kalman_bringup kalman_bringup.launch.py lidar_model:=LDROBOT-LD19 use_sim_time:=false use_rviz:=true use_uros:=false


ros2 launch kalman_bringup cartographer.launch.py use_sim_time:=false
ros2 run nav2_map_server map_saver_cli -f mapa_kalman









# Specify target location;; robot self-drives using an existing map
ros2 launch kaiaai_bringup navigation.launch.py map:=$HOME/maps/map.yaml

# Launch SLAM (simultaneous localization and mapping) - navigate and map simultaneously
ros2 launch kaiaai_bringup navigation.launch.py slam:=True