#!/usr/bin/env python3
"""
IMU Calibration Tool para ICM6500
Recolecta datos raw del IMU en reposo y calcula offsets (bias)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from kalman_interfaces.msg import ImuData
from ament_index_python.packages import get_package_share_directory
import numpy as np
import yaml
import os
from datetime import datetime


class IMUCalibration(Node):
    def __init__(self):
        super().__init__('imu_calibration')

        # Parámetros configurables
        self.declare_parameter('imu_topic', '/imu_telem')
        self.declare_parameter('calibration_duration', 10)  # segundos
        self.declare_parameter('samples_per_second', 100)

        imu_topic = self.get_parameter('imu_topic').value
        self.duration = self.get_parameter('calibration_duration').value
        self.samples_per_sec = self.get_parameter('samples_per_second').value

        # Buffers para almacenar datos
        self.accel_data = {'x': [], 'y': [], 'z': []}
        self.gyro_data = {'x': [], 'y': [], 'z': []}

        # Estadísticas esperadas
        # En reposo: aceleración = [0, 0, 1g] en LSB = [0, 0, 2048]
        # En reposo: giroscopio = [0, 0, 0]

        self.sample_count = 0
        self.target_samples = int(self.duration * self.samples_per_sec)

        # QoS compatible con SensorDataQoS del nodo IMU en C++
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Crear suscriptor
        self.imu_sub = self.create_subscription(
            ImuData,
            imu_topic,
            self.imu_callback,
            qos_profile
        )

        self.get_logger().info(
            f'\n{"="*60}\n'
            f'Iniciando calibración del IMU\n'
            f'Duración: {self.duration} segundos\n'
            f'Topic: {imu_topic}\n'
            f'Muestras esperadas: {self.target_samples}\n'
            f'\n⚠️  IMPORTANTE: Coloca el IMU en reposo sobre una superficie plana\n'
            f'    NO lo muevas durante la calibración\n'
            f'{"="*60}\n'
        )

    def imu_callback(self, msg):
        """Callback que recolecta datos del IMU"""
        if self.sample_count >= self.target_samples:
            return

        # Almacenar datos
        self.accel_data['x'].append(msg.accel_x)
        self.accel_data['y'].append(msg.accel_y)
        self.accel_data['z'].append(msg.accel_z)

        self.gyro_data['x'].append(msg.gyro_x)
        self.gyro_data['y'].append(msg.gyro_y)
        self.gyro_data['z'].append(msg.gyro_z)

        self.sample_count += 1

        # Mostrar progreso cada 10% de las muestras
        if self.sample_count % max(1, self.target_samples // 10) == 0:
            progress = (self.sample_count / self.target_samples) * 100
            self.get_logger().info(f'Progreso: {progress:.0f}% ({self.sample_count}/{self.target_samples})')

        # Cuando se recolectaron todas las muestras
        if self.sample_count >= self.target_samples:
            self.compute_calibration()

    def compute_calibration(self):
        """Calcula los offsets y los guarda"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('Calibración completada. Calculando offsets...')
        self.get_logger().info('='*60)

        # Calcular media y desviación estándar
        accel_offsets = {
            'x': {'mean': np.mean(self.accel_data['x']), 'std': np.std(self.accel_data['x'])},
            'y': {'mean': np.mean(self.accel_data['y']), 'std': np.std(self.accel_data['y'])},
            'z': {'mean': np.mean(self.accel_data['z']), 'std': np.std(self.accel_data['z'])},
        }

        gyro_offsets = {
            'x': {'mean': np.mean(self.gyro_data['x']), 'std': np.std(self.gyro_data['x'])},
            'y': {'mean': np.mean(self.gyro_data['y']), 'std': np.std(self.gyro_data['y'])},
            'z': {'mean': np.mean(self.gyro_data['z']), 'std': np.std(self.gyro_data['z'])},
        }

        # Mostrar resultados
        self.get_logger().info('\n📊 DATOS RAW RECOLECTADOS:\n')

        self.get_logger().info('Aceleración (LSB):')
        for axis in ['x', 'y', 'z']:
            self.get_logger().info(
                f'  {axis.upper()}: {accel_offsets[axis]["mean"]:.2f} ± {accel_offsets[axis]["std"]:.2f}'
            )

        self.get_logger().info('\nGiroscopio (LSB):')
        for axis in ['x', 'y', 'z']:
            self.get_logger().info(
                f'  {axis.upper()}: {gyro_offsets[axis]["mean"]:.2f} ± {gyro_offsets[axis]["std"]:.2f}'
            )

        # Información sobre qué esperar
        self.get_logger().info('\n📝 VALORES ESPERADOS (si el IMU estaba en reposo):\n')
        self.get_logger().info('Aceleración:')
        self.get_logger().info('  X: ~0 LSB (debería estar cerca de 0)')
        self.get_logger().info('  Y: ~0 LSB (debería estar cerca de 0)')
        self.get_logger().info('  Z: ~2048 LSB (debería estar cerca de 2048 para ±16g)')
        self.get_logger().info('\nGiroscopio:')
        self.get_logger().info('  X, Y, Z: ~0 LSB (debería estar cerca de 0)')

        # Guardar configuración
        calibration_config = {
            'timestamp': datetime.now().isoformat(),
            'duration_seconds': self.duration,
            'total_samples': self.sample_count,
            'accelerometer': {
                'offset': {
                    'x': float(accel_offsets['x']['mean']),
                    'y': float(accel_offsets['y']['mean']),
                    'z': float(accel_offsets['z']['mean']),
                },
                'noise_std': {
                    'x': float(accel_offsets['x']['std']),
                    'y': float(accel_offsets['y']['std']),
                    'z': float(accel_offsets['z']['std']),
                }
            },
            'gyroscope': {
                'offset': {
                    'x': float(gyro_offsets['x']['mean']),
                    'y': float(gyro_offsets['y']['mean']),
                    'z': float(gyro_offsets['z']['mean']),
                },
                'noise_std': {
                    'x': float(gyro_offsets['x']['std']),
                    'y': float(gyro_offsets['y']['std']),
                    'z': float(gyro_offsets['z']['std']),
                }
            }
        }

        # Ruta del archivo de calibración en el paquete
        try:
            package_share_dir = get_package_share_directory('kalman_utils')
            calibration_file = os.path.join(package_share_dir, 'config', 'imu_calibration.yaml')
        except Exception as e:
            self.get_logger().error(f'Error al obtener directorio del paquete: {e}')
            # Fallback a directorio home
            config_dir = os.path.expanduser('~/kalman_calibration')
            os.makedirs(config_dir, exist_ok=True)
            calibration_file = os.path.join(config_dir, 'imu_calibration.yaml')

        try:
            with open(calibration_file, 'w') as f:
                yaml.dump(calibration_config, f, default_flow_style=False)

            self.get_logger().info(f'\n✅ Calibración guardada en: {calibration_file}')
        except Exception as e:
            self.get_logger().error(f'❌ Error al guardar calibración: {e}')

        # Mostrar archivo guardado
        self.get_logger().info(f'\n📄 Contenido del archivo de calibración:\n')
        try:
            with open(calibration_file, 'r') as f:
                content = f.read()
                self.get_logger().info(content)
        except Exception as e:
            self.get_logger().error(f'Error al leer archivo: {e}')

        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('Puedes usar estos offsets para mejorar la precisión del IMU')
        self.get_logger().info('='*60 + '\n')

        # Detener el nodo
        raise KeyboardInterrupt()


def main(args=None):
    rclpy.init(args=args)

    try:
        calibration_node = IMUCalibration()
        rclpy.spin(calibration_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
