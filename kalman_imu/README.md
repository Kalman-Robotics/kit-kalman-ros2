# kalman_imu package

Este paquete contiene nodos para el procesamiento de datos del IMU, incluyendo una funcionalidad de autocalibración para mejorar la precisión de las lecturas del sensor.

- [kalman\_imu package](#kalman_imu-package)
  - [Archivos Launch](#archivos-launch)
    - [`imu_processor.launch.py`](#imu_processorlaunchpy)
    - [`imu_processor2.launch.py`](#imu_processor2launchpy)
  - [Uso](#uso)
  - [Consideraciones](#consideraciones)

## Archivos Launch

### `imu_processor.launch.py`
Inicia el nodo de procesamiento del IMU con autocalibración.

**Argumentos:**
Este launch file no tiene argumentos de línea de comandos. Los parámetros se configuran internamente:
- `imu.topic_name_sub`: Tópico de entrada de datos IMU (default: `/imu_telem`)
- `imu.topic_name_pub`: Tópico de salida de datos procesados (default: `/imu_processed`)
- `imu.accel_scale`: Escala del acelerómetro en LSB/g (default: `16384.0` para ±2g)
- `imu.gyro_scale`: Escala del giroscopio en LSB/°/s (default: `131.0` para ±250°/s)
- `imu.gravity_accel`: Aceleración gravitacional en m/s² (default: `9.81`)
- `imu.complementary_alpha`: Coeficiente del filtro complementario (default: `0.96`)
- `imu.calibration_samples`: Número de muestras para calibración (default: `1000`)
- `imu.auto_calibrate`: Habilitar autocalibración al inicio (default: `True`)

### `imu_processor2.launch.py`
Inicia el nodo de procesamiento del IMU con control de frecuencia de publicación.

**Argumentos:**
Este launch file no tiene argumentos de línea de comandos. Los parámetros se configuran internamente:
- `imu.topic_name_sub`: Tópico de entrada de datos IMU (default: `/imu_telem`)
- `imu.topic_name_pub`: Tópico de salida de datos procesados (default: `/imu_processed`)
- `imu.accel_scale`: Escala del acelerómetro en LSB/g (default: `16384.0` para ±2g)
- `imu.gyro_scale`: Escala del giroscopio en LSB/°/s (default: `131.0` para ±250°/s)
- `imu.gravity_accel`: Aceleración gravitacional en m/s² (default: `9.81`)
- `imu.complementary_alpha`: Coeficiente del filtro complementario (default: `0.96`)
- `imu.calibration_samples`: Número de muestras para calibración (default: `1000`)
- `imu.auto_calibrate`: Habilitar autocalibración al inicio (default: `True`)
- `imu.publish_rate`: Frecuencia de publicación en Hz (default: `50.0`, ajustable: 10-100 Hz)

## Uso
Este nodo se encarga de recibir los datos del IMU de telemétría, realizar la autocalibración, filtrar los datos de orientación y publicar los datos del sensor en un tópico de ROS2.
El siguiente launch file inicia el nodo de procesamiento del IMU, incluyendo una autocalibración:
```
ros2 launch kalman_imu imu_processor.launch.py
```

## Consideraciones
- El IMU es de 6DOF, por lo que no proporciona datos de magnetómetro.
- Un sensor inercial de 6DOF experimentará una deriva en la orientación yaw con el tiempo, ya que no tiene un valor adicional para corregir esta deriva (como un magnetómetro).