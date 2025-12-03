# kalman_imu package

Este paquete contiene nodos para el procesamiento de datos del IMU, incluyendo una funcionalidad de autocalibración para mejorar la precisión de las lecturas del sensor.

- [kalman\_imu package](#kalman_imu-package)
  - [Uso](#uso)
  - [Consideraciones](#consideraciones)

## Uso
Este nodo se encarga de recibir los datos del IMU de telemétría, realizar la autocalibración, filtrar los datos de orientación y publicar los datos del sensor en un tópico de ROS2.
El siguiente launch file inicia el nodo de procesamiento del IMU, incluyendo una autocalibración:
```
ros2 launch kalman_imu imu_processor.launch.py
```

## Consideraciones
- El IMU es de 6DOF, por lo que no proporciona datos de magnetómetro.
- Un sensor inercial de 6DOF experimentará una deriva en la orientación yaw con el tiempo, ya que no tiene un valor adicional para corregir esta deriva (como un magnetómetro).