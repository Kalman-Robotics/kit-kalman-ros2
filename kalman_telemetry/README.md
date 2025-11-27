# kalman_telemetry package

Este paquete se comunica con el robot de Kalman, recibe datos de telemetría cruda de los sensores por WiFi usando Micro-ROS y vuelve a publicar la telemetría en tópicos estándares de ROS2.

- [kalman\_telemetry package](#kalman_telemetry-package)
  - [Uso](#uso)
    - [Con robot conectado via Micro-ROS](#con-robot-conectado-via-micro-ros)
    - [Desarrollo y pruebas sin robot](#desarrollo-y-pruebas-sin-robot)
  - [Detalle de telemetría del robot](#detalle-de-telemetría-del-robot)
  - [Notas](#notas)

## Uso
### Con robot conectado via Micro-ROS
```
ros2 run kalman_telemetry telem --ros-args -p laser_scan.lidar_model:="LDROBOT-LD19"
```
Suscribe a tópicos:
- `/telemetry` : Suscribe la telemetría del robot.

Publica los tópicos:
- `/battery_state` : Publica el estado de la batería del robot.
- `/odom` : Publica la odometría del robot.
- `/joint_states` : Publica los estados de las articulaciones del robot.
- `/scan` : Publica los datos del escáner láser del robot.
- `/wifi_state` : Publica el estado de la conexión Wi-Fi del robot.
- `/tf`: Publica la transformación de odometría del robot.

### Desarrollo y pruebas sin robot
Si no dispone de un robot, también puede ejecutar el nodo de prueba que publica datos de telemetría ficticios directamente en /telemetry, omitiendo Micro-ROS:
```
ros2 run kaiaai_telemetry test_pub
```

## Detalle de telemetría del robot
Telemetría recibida del robot en el tópico /telemetry:

```
---
stamp:
  sec: 1762967915
  nanosec: 874411000
seq: 193171
odom_pos_x: 0.026938902214169502
odom_pos_y: 0.5149940848350525
odom_pos_yaw: -0.1071779727935791
odom_vel_x: 0.0
odom_vel_yaw: 0.0
joint:
- pos: -124.45195007324219
  vel: 0.0
- pos: 123.92835235595703
  vel: 0.0
wifi_rssi_dbm: -33
battery_mv: 3700
distance_mm: []
bumper: []
cliff: []
touch: []
scan_start_hint: false
lds: [...]
---
```

## Notas
- El mensaje de telemetría es un mensaje personalizado de ROS2 diseñado para ser lo más compacto posible en cuanto a tamaño, con el fin de reducir la latencia de comunicación y minimizar paquetes perdidos, manteniendo la navegación del robot ágil y sensible.