# kalman_bringup package

- [kalman\_bringup package](#kalman_bringup-package)
  - [Consideraciones previas](#consideraciones-previas)
  - [Uso](#uso)
    - [Publicar el urdf del robot y visualizarlo en RViz](#publicar-el-urdf-del-robot-y-visualizarlo-en-rviz)
    - [Lanzamiento inicial del robot](#lanzamiento-inicial-del-robot)
    - [Mapeo](#mapeo)
    - [Monitoreo por RViz](#monitoreo-por-rviz)


## Consideraciones previas

- Haber subido el firmware Kalman y configuración de WiFi a la placa ESP32 del robot siguiendo las [instrucciones](https://github.com/Kalman-Robotics/kit-kalman-firmware).
- Encienda la alimentación del robot.
- Asegúrese de que el robot esté conectado a la misma red Wi‑Fi 2.4G.
- Ejecutar el agente de Micro-ROS
- El robot debería conectarse automáticamente al agente de Micro-ROS que se está ejecutando en la computadora.
- Establecida la comunicación entre el robot y la computadora, podemos proceder a lanzar los nodos de ROS2 para el robot.

> [!NOTE] Puede verificar la conexión exitosa observando el patrón de parpadeo del LED de la placa ESP32.
Si el patrón de parpadeo indica un error, conecte el PC a la placa ESP32 del robot mediante un cable USB, abra un Monitor Serie.

## Uso 

### Publicar el urdf del robot y visualizarlo en RViz
```
ros2 launch kalman_bringup inspect_urdf.launch.py joints:=none robot_model:=makerspet_mini
```
Suscribe a tópicos:
- `/joint_states` : Suscribe los estados de las articulaciones del robot.

Publica los tópicos:
- `/robot_description` : Publica el modelo URDF del robot.
- `/tf` : Publica las transformaciones del robot.
- `/tf_static` : Publica las transformaciones estáticas del robot.

### Lanzamiento inicial del robot
```
ros2 launch kalman_bringup kalman_bringup.launch.py lidar_model:=LDROBOT-LD19 use_sim_time:=false use_rviz:=true use_uros:=false robot_ip:=192.168.18.16
```
Suscribe a tópicos:
- `/telemetry` : Suscribe la telemetría del robot.

- `/joint_states` : Suscribe los estados de las articulaciones del robot.

Publica los tópicos:
- `/battery_state` : Publica el estado de la batería del robot.
- `/odom` : Publica la odometría del robot.
- `/joint_states` : Publica los estados de las articulaciones del robot.
- `/scan` : Publica los datos del escáner láser del robot.
- `/wifi_state` : Publica el estado de la conexión Wi-Fi del robot.
- `/tf`: Publica la transformación de odometría del robot.
<!-- - `/control_status` : Publica el estado de control del robot. -->

- `/robot_description` : Publica el modelo URDF del robot.
- `/tf` : Publica las transformaciones del robot.
- `/tf_static` : Publica las transformaciones estáticas del robot.

### Mapeo
```
ros2 launch kalman_bringup cartographer.launch.py robot_model:=makerspet_mini use_sim_time:=false
```

### Monitoreo por RViz
```
ros2 launch kalman_bringup monitor_robot.launch.py use_sim_time:=false robot_model:=kalman_description
```