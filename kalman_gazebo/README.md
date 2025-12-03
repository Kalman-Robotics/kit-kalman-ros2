# kalman_gazebo package 

- [kalman\_gazebo package](#kalman_gazebo-package)
  - [Uso](#uso)
  - [Aplicaciones](#aplicaciones)
    - [Comando por Teclado](#comando-por-teclado)
    - [Navegación Autónoma](#navegación-autónoma)

## Uso
Para lanzar la simulación de Gazebo con el robot Kalman, use el siguiente comando:
```
ros2 launch kalman_gazebo simulation.launch.py robot_model:=kalman_description
```

## Aplicaciones
Para ejecutar las utilidades de los paquetes de Kalman, asegurese de configurar el argumento `use_sim_time` a `true` para sincronizar con el tiempo simulado de Gazebo.

### Comando por Teclado
Para controlar el robot Kalman en la simulación de Gazebo usando el teclado, use el siguiente comando:
```
ros2 run kalman_teleop teleop_keyboard 
```

### Navegación Autónoma
Para lanzar la navegación autónoma en un entorno simulado, use el siguiente comando:
```
ros2 launch kalman_bringup navigation.launch.py use_sim_time:=false robot_model:=kalman_description slam:=False map:=/ros2_ws/src/kalman_bringup/map/living_room.yaml
```

![ejm navegación](images/navigation.png)