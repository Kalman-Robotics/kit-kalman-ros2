# kalman_teleop package

Controlar un robot (físico o simulado) usando un teclado.

- [kalman\_teleop package](#kalman_teleop-package)
  - [Uso](#uso)
    - [Uso simple](#uso-simple)
    - [Uso personalizado](#uso-personalizado)
  - [Archivo de configuración](#archivo-de-configuración)
  - [Advertencia - no utilizar en otros .launch.py](#advertencia---no-utilizar-en-otros-launchpy)


## Uso
### Uso simple
Inicia el teleop por teclado para el modelo de robot predeterminado. 
```
ros2 run kalman_teleop teleop_keyboard
```

Ahora deberías poder controlar tu robot usando el teclado de tu PC.
```
root@e3043f4ccd4c:/ros_ws# ros2 run kalman_teleop teleop_keyboard
YAML file name : /ros_ws/install/kalman_description/share/kalman_description/config/teleop_keyboard.yaml
Max linear velocity 0.200        Max angular velocity 3.860
Control Kaia.ai-compatible Robot
--------------------------------
Moving around:
  w
 a    s    d
  x
w/x   : increase/decrease linear  velocity
a/d   : increase/decrease angular velocity
s     : keep straight
CAPS  : large step
Space : force stop
CTRL-C to quit
```

### Uso personalizado
Inicia el teleop por teclado para una configuración específica.

<!-- - Inicia el teleop por teclado para el modelo de robot.
```
ros2 run kalman_teleop teleop_keyboard robot_model:=makerspet_loki
``` -->

- Inicia el teleop por teclado usando el archivo de configuración `/path/to/teleop_keyboard.yaml`.
```
ros2 run kalman_teleop teleop_keyboard --ros-args --params-file /path/to/teleop_keyboard.yaml
```

## Archivo de configuración
`teleop_keyboard.yaml` es un archivo de configuración que contiene parámetros de velocidad del robot:
- velocidades lineales y angulares mínimas y máximas
- steps grandes y pequeños para las velocidades lineal y angular
```
teleop_keyboard_node:
  ros__parameters:
    max_lin_vel: 0.2
    max_ang_vel: 3.86
    lin_vel_step: 0.05
    ang_vel_step: 0.2
    lin_vel_step_large: 0.2
    ang_vel_step_large: 2.5
```

## Advertencia - no utilizar en otros .launch.py
El nodo `teleop_keyboard` no puede iniciarse desde un archivo `launch`, incluyendo `.launch.py`.  
Si lo intentas, obtendrás un error (en Linux) `termios.error: (25, 'Inappropriate ioctl for device')`.  
El error se debe a que `teleop_keyboard.py` necesita acceso directo al dispositivo TTY STDIN para capturar las pulsaciones del teclado.
