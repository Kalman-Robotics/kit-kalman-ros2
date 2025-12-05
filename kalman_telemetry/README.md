# kalman_telemetry package

Este paquete se comunica con el robot de Kalman, recibe datos de telemetría cruda de los sensores por WiFi usando Micro-ROS y vuelve a publicar la telemetría en tópicos estándares de ROS2.

- [kalman\_telemetry package](#kalman_telemetry-package)
  - [Ejecutables](#ejecutables)
    - [`telem`](#telem)
    - [`telem_test_pub`](#telem_test_pub)
  - [Uso](#uso)
    - [Con robot conectado via Micro-ROS](#con-robot-conectado-via-micro-ros)
    - [Desarrollo y pruebas sin robot](#desarrollo-y-pruebas-sin-robot)
  - [Detalle del mensaje de telemetría proveniente robot](#detalle-del-mensaje-de-telemetría-proveniente-robot)
  - [Notas](#notas)

## Ejecutables

### `telem`
Nodo principal de telemetría que procesa datos del robot y los republica en tópicos estándar de ROS2.

**Parámetros:**
- `laser_scan.topic_name_pub`: Nombre del tópico de publicación del escaneo láser (default: `scan`)
- `laser_scan.frame_id`: Frame ID del sensor láser (default: `laser_link`)
- `laser_scan.lidar_model`: Modelo del LiDAR (default: `YDLIDAR-X4`)
  - Modelos soportados: `YDLIDAR-X4`, `LDROBOT-LD19`, `XIAOMI-LDS02RR`, `YDLIDAR-X2-X2L`, `3IROBOTIX-DELTA-2G`, `YDLIDAR-X3-PRO`, `YDLIDAR-X3`, `NEATO-XV11`, `SLAMTEC-RPLIDAR-A1`, `3IROBOTIX-DELTA-2A`, `3IROBOTIX-DELTA-2B`, `LDROBOT-LD14P`, `CAMSENSE-X1`, `YDLIDAR-SCL`
- `laser_scan.mask_radius_meters`: Radio de máscara para filtrar puntos cercanos (default: `0.0`)
- `laser_scan.discard_broken_scans`: Descartar escaneos rotos (default: `false`)
- `telemetry.topic_name_sub`: Nombre del tópico de suscripción de telemetría (default: `telemetry`)
- `tf.frame_id`: Frame ID de transformación (default: `odom`)
- `tf.child_frame_id`: Frame ID hijo de transformación (default: `base_footprint`)
- `joints.topic_name_pub`: Nombre del tópico de publicación de estados de articulaciones (default: `joint_states`)
- `joints.wheel.right`: Nombre de la articulación de rueda derecha (default: `wheel_right_joint`)
- `joints.wheel.left`: Nombre de la articulación de rueda izquierda (default: `wheel_left_joint`)
- `odometry.frame_id`: Frame ID de odometría (default: `odom`)
- `odometry.child_frame_id`: Frame ID hijo de odometría (default: `base_footprint`)
- `odometry.topic_name_pub`: Nombre del tópico de publicación de odometría (default: `odom`)
- `battery.topic_name_pub`: Nombre del tópico de publicación del estado de batería (default: `battery_state`)
- `battery.voltage_full`: Voltaje de batería completa en voltios (default: `4.1` V)
- `battery.voltage_empty`: Voltaje de batería vacía en voltios (default: `3.5` V)
- `wifi.topic_name_pub`: Nombre del tópico de publicación del estado WiFi (default: `wifi_state`)
- `control_status.topic_name_pub`: Nombre del tópico de publicación del estado de control (default: `control_status`)

**Configuración avanzada de LiDAR:**
- `lidar.model`: Vector de modelos de LiDAR soportados
- `lidar.angle_offset_deg`: Vector de offsets de ángulo en grados
- `lidar.clockwise`: Vector de dirección de rotación (sentido horario)
- `lidar.pub_scan_size`: Vector de tamaño de escaneo publicado
- `lidar.range_min_meters`: Vector de rango mínimo en metros
- `lidar.range_max_meters`: Vector de rango máximo en metros
- `lidar.intensity`: Vector de uso de intensidad

### `telem_test_pub`
Nodo de prueba que publica datos de telemetría ficticios sin necesidad de un robot físico.

**Parámetros:**
Este ejecutable no tiene parámetros configurables. Publica automáticamente en el tópico `/telemetry` con datos de prueba simulados.

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

## Detalle del mensaje de telemetría proveniente robot
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