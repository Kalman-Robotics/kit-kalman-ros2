# Kit Kalman ROS2 - Guía de inicio rápido


### 5. Ejecutar nodo de telemetría para obtener tópicos del robot
```
ros2 run kalman_telemetry telem --ros-args -p laser_scan.lidar_model:="LDROBOT-LD19"
```


### 8. Teleoperar el robot
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

