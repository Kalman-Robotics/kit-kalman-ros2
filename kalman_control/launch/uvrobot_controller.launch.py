import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, EnvironmentVariable
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessIO
from launch.events import Shutdown
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals,launch_configuration_not_equals
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
)
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():

    robot_name = "uvrobot"
    package_controller = robot_name + "_control"
    package_description = robot_name + '_description'

    #~~~~~~~~~~~~~~~~~~~~~~~~ PATHS ~~~~~~~~~~~~~~~~~~~~~~~~~
    package_description_share = FindPackageShare(package_description)
    package_controller_share = FindPackageShare(package_controller)
    
    path_xacro = PathJoinSubstitution([package_description_share,'urdf','uvrobot.urdf.xacro'])
    path_controller = PathJoinSubstitution([package_controller_share,'config','diff_drive_controller_real.yaml'])

    #~~~~~~~~~~~~~~~~~~~~~~~~ ARGUMENTS ~~~~~~~~~~~~~~~~~~~~~~~~~~~+

    arg_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value= "False",
        description='Use simulation or not'
    )
    config_use_sim = LaunchConfiguration('use_sim')
    
    arg_use_ros2_control = DeclareLaunchArgument(
        'use_ros2_control',
        default_value= "true",
        description='Use ros2 control or not'
    )
    config_use_ros2_control = LaunchConfiguration('use_ros2_control')
    
    arg_scale = DeclareLaunchArgument(
        'scale',
        default_value= "0.001",
        description='Scale of robot'
    )
    config_scale = LaunchConfiguration('scale')
    
    #~~~~~~~~~~~~~~~~~~~~~~~~ NODES ~~~~~~~~~~~~~~~~~~~~~~~~~~~+
    robot_description_content = Command(['xacro ', path_xacro, " use_sim:=",config_use_sim, 
                                                        " use_ros2_control:=",config_use_ros2_control,
                                                        " scale:=", config_scale])
    
    
    robot_description = {'robot_description': robot_description_content}
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {'use_sim_time': config_use_sim}],
        emulate_tty=True,
        condition=UnlessCondition(config_use_sim),
    )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[path_controller],
        remappings=[
            ("~/motors_cmd", "/_motors_cmd"),
            ("~/motors_response", "/_motors_response"),
            ("/uvrobot_diff_base_controller/cmd_vel_unstamped", "/cmd_vel"),
            ("~/robot_description", "/robot_description"),
        
        ],
        condition=UnlessCondition(config_use_sim)
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "20",
        ],
    )
    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "uvrobot_diff_base_controller",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "20",
        ],
    )
    # spawners expect ros2_control_node to be running
    delayed_controllers = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster,diff_drive_controller]
    )

    return LaunchDescription([
        arg_use_sim,
        arg_use_ros2_control,
        arg_scale,
        control_node,
        robot_state_publisher,
        delayed_controllers
    ])