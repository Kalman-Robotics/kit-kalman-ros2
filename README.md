### 4. Ejecutar el agente de micro-ROS
```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -i <IP_DE_LA_COMPUTADORA>
```
## Setup esencial (simplificado)

### 1. Launch: tópicos(telemetry) + robot_state(urdf)
```
ros2 launch kaiaai_bringup kalman_bringup.launch.py lidar_model:=LDROBOT-LD19 use_sim_time:=false use_rviz:=false
```
### 2.  rviz
```
ros2 run rviz2 rviz2 -d src/kit-kalman-ros2/kaiaai_bringup/rviz/bringup.rviz
```
## 3.  IMu

ros2 launch kalman_utils madgwick.launch.py

## Mapeo

```
ros2 launch kaiaai_bringup cartographer.launch.py use_sim_time:=false
```

