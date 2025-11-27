Micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -i 192.168.18.16

Telemetry for robot topics
ros2 run kalman_telemetry telem --ros-args -p laser_scan.lidar_model:=LDROBOT-LD19

Publish robot urdf and visualize
ros2 launch kalman_bringup inspect_urdf.launch.py joints:=nogui robot_model:=makerspet_mini

teleop
ros2 topic echo /scan

FULL LAUNCH
ros2 launch kalman_bringup kalman_bringup.launch.py lidar_model:=LDROBOT-LD19 use_sim_time:=false use_rviz:=true use_uros:=false