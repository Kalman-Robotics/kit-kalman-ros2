from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('kalman_utils'),
        'config'
    )
    
    imu_processor_node = Node(
        package='kalman_utils',
        executable='imu_processor',
        name='imu_processor',
        output='screen',
        parameters=[{
            'imu.topic_name_sub': '/imu_telem',
            'imu.topic_name_pub': '/imu_processed',
            'imu.accel_scale': 16384.0,      # LSB/g for ±2g
            'imu.gyro_scale': 131.0,         # LSB/°/s for ±250°/s
            'imu.gravity_accel': 9.81,       # m/s²
            'imu.complementary_alpha': 0.96,  # Complementary filter
            'imu.calibration_samples': 1000, # Calibration samples
            'imu.auto_calibrate': True       # Auto-calibrate on startup            
        }]
    )
    
    return LaunchDescription([
        imu_processor_node
    ])