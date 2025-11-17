cmake_minimum_required(VERSION 3.8)
project(kalman_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclpy REQUIRED)  
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)


install (DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)


# Python package installation
ament_python_install_package(${PROJECT_NAME})


# Install Python scripts
install(PROGRAMS
  scripts/kalman_control.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
