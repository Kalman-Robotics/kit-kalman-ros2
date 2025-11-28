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

MAPPING
ros2 launch kalman_bringup cartographer.launch.py use_sim_time:=false robot_model:=kalman_description
ros2 run nav2_map_server map_saver_cli -f mapa_kalman

NAVIGATION
ros2 launch kalman_bringup navigation.launch.py use_sim_time:=false robot_model:=kalman_description slam:=False



---
ros2 run nav2_map_server map_server --ros-args   -p yaml_filename:=/home/sinso/ros2_ws/install/kalman_bringup/share/kalman_bringup/map/mapa_kalman.yaml -p use_sim_time:=false