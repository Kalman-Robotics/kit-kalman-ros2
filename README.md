# Kit-Kalman-ROS2
Este repositorio contiene los paquetes de ROS2 del robot

- [Kit-Kalman-ROS2](#kit-kalman-ros2)
  - [Pasos probados en Ubuntu 22.04:](#pasos-probados-en-ubuntu-2204)
    - [1. Clonar el repositorio en el workspace de ROS2](#1-clonar-el-repositorio-en-el-workspace-de-ros2)
    - [2. Instalar dependencias](#2-instalar-dependencias)
    - [3. Compilar el proyecto](#3-compilar-el-proyecto)
    - [4. Ejecutar el agente de micro-ROS](#4-ejecutar-el-agente-de-micro-ros)
    - [5. Ejecutar nodo de telemetría para obtener tópicos del robot](#5-ejecutar-nodo-de-telemetría-para-obtener-tópicos-del-robot)
    - [6. Publicar el urdf del robot y visualizarlo en RViz](#6-publicar-el-urdf-del-robot-y-visualizarlo-en-rviz)
    - [7. Revisar tópico `scan`](#7-revisar-tópico-scan)
    - [8. Teleoperar el robot](#8-teleoperar-el-robot)
  - [Setup esencial (simplificado)](#setup-esencial-simplificado)
    - [1. Ejecutar el agente de micro-ROS](#1-ejecutar-el-agente-de-micro-ros)
    - [2. Launch: tópicos(telemetry) + robot\_state(urdf) + rviz](#2-launch-tópicostelemetry--robot_stateurdf--rviz)
  - [Mapeo](#mapeo)
  - [Otros](#otros)

## Pasos probados en Ubuntu 22.04:
### 1. Clonar el repositorio en el workspace de ROS2
### 2. Instalar dependencias
```
rosdep update
cd ~/ros2_ws
rosdep install --from-paths src/Kit-Kalman-ROS2 --ignore-src -r -y
```
### 3. Compilar el proyecto
```
cd ~/ros2_ws
colcon build --packages-up-to kaiaai
source install/setup.bash
```
### 4. Ejecutar el agente de micro-ROS
```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -i <IP_DE_LA_COMPUTADORA>
```
### 5. Ejecutar nodo de telemetría para obtener tópicos del robot
```
ros2 run kaiaai_telemetry telem --ros-args -p laser_scan.lidar_model:="LDROBOT-LD19"
```
### 6. Publicar el urdf del robot y visualizarlo en RViz
```
ros2 launch kaiaai_bringup inspect_urdf.launch.py joints:=nogui robot_model:=makerspet_mini
```
### 7. Revisar tópico `scan` 
```
ros2 topic echo /scan
```
### 8. Teleoperar el robot
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

## Setup esencial (simplificado)
### 1. Ejecutar el agente de micro-ROS
```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -i <IP_DE_LA_COMPUTADORA>
```
### 2. Launch: tópicos(telemetry) + robot_state(urdf) + rviz
```
ros2 launch kaiaai_bringup kalman_bringup.launch.py robot_model:=makerspet_mini lidar_model:=LDROBOT-LD19 use_sim_time:=false use_rviz:=true
```

## Mapeo
Previamente realizar el setup esencial.
```
ros2 launch kaiaai_bringup cartographer.launch.py robot_model:=makerspet_mini use_sim_time:=false
```

## Otros