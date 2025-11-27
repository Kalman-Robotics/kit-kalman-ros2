<!-- remove folder kalman_interfaces and re-import repos -->
rm -rf kalman_interfaces
vcs import ./ <  kalman_software.repos

### 4. Ejecutar el agente de micro-ROS
```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -i <IP_DE_LA_COMPUTADORA>
```

## Setup esencial (simplificado)

### 1. Launch: tópicos(telemetry) + robot_state(urdf)
```
ros2 launch kalman_bringup kalman_bringup.launch.py lidar_model:=LDROBOT-LD19 use_sim_time:=false use_rviz:=true use_uros:=false
```
### 2.  rviz
```
rviz2
```
## 3.  IMU
```
sudo apt update
sudo apt install ros-humble-imu-filter-madgwick
ros2 launch kalman_utils madgwick.launch.py
```
Calibrar
```
ros2 run kalman_utils imu_calibration
```


## Mapeo

```
ros2 launch kalman_bringup cartographer.launch.py use_sim_time:=false
```

