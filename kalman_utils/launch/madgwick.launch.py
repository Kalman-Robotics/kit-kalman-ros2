# Este archivo launch configura y ejecuta el nodo IMU Madgwick
# para procesar datos del IMU y calcular la orientación usando AHRS

from launch_ros.actions import Node
from launch import LaunchDescription  # Contenedor principal que agrupa todo lo que se va a lanzar
from ament_index_python.packages import get_package_share_directory   # Obtiene la ruta del directorio share de un paquete
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution    # Obtiene valores de argumentos definidos en el launch, Une rutas de archivos de manera segura
from launch.actions import DeclareLaunchArgument # Define argumentos que se pueden pasar al launch

def generate_launch_description():

    package_name = "kalman_utils"

    #~~~~~~~~~~~~~~~~~~~~~~~~ PATHS ~~~~~~~~~~~~~~~~~~~~~~~~~~~+
    package_share = get_package_share_directory(package_name)

    madgwick_config_path = PathJoinSubstitution([package_share, 'config', 'madgwick.yaml'])


    #~~~~~~~~~~~~~~~~~~~~~~~~ ARGUMENTS ~~~~~~~~~~~~~~~~~~~~~~~~~~~+

    arg_imu_topic_pub = DeclareLaunchArgument(
        'imu_topic_pub',
        default_value='imu',
        description='Output IMU topic (with orientation)'
    )

    arg_frequency = DeclareLaunchArgument(
        'frequency',
        default_value='50.0',
        description='IMU processing frequency in Hz'
    )

    arg_gain = DeclareLaunchArgument(
        'gain',
        default_value='0.1',
        description='Madgwick filter gain parameter'
    )

    config_imu_topic_pub = LaunchConfiguration('imu_topic_pub')
    config_frequency = LaunchConfiguration('frequency')
    config_gain = LaunchConfiguration('gain')


    #~~~~~~~~~~~~~~~~~~~~~~~~ NODES ~~~~~~~~~~~~~~~~~~~~~~~~~~~+
    # Nodo IMU: convierte datos personalizados a formato estándar ROS2
    imu_node = Node(
            package='kalman_utils',
            executable='imu',
            name='imu_node',
            emulate_tty=True,
            parameters=[
                {'imu.topic_name_sub': '/imu_telem'},
                {'imu.topic_name_pub': '/imu_raw'},
                {'imu.accel_scale': 2048.0},       # ICM6500: ±16g range, 2048 LSB/g
                {'imu.gyro_scale': 16.384},        # ICM6500: ±2000°/s range, 16.384 LSB/°/s
                {'imu.gravity_accel': 9.81},       # Standard gravity in m/s²
                {'imu.filter_alpha': 0.15},        # Low-pass filter: 0.1=strong, 0.5=medium, 0.9=light
            ],
            output='screen'
    )

    # Nodo IMU Madgwick para fusión de sensores y cálculo de orientación
    imu_madgwick_node = Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_madgwick_node',
            emulate_tty=True,
            remappings=[
                ('imu/data_raw', '/imu_raw'),
            ],
            parameters=[
                madgwick_config_path,
                {'frequency': config_frequency},
                {'gain': config_gain},
            ],
            output='screen'
    )

    #~~~~~~~~~~~~~~~~~~~~~~~~ LAUNCH ~~~~~~~~~~~~~~~~~~~~~~~~~~~+
    return LaunchDescription([
        arg_imu_topic_pub,
        arg_frequency,
        arg_gain,
        imu_node,
        imu_madgwick_node,
    ])
