cmake_minimum_required(VERSION 3.8)
project(kalman_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_python REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kalman_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(imu src/imu.cpp)
ament_target_dependencies(imu rclcpp ament_index_cpp sensor_msgs kalman_interfaces)
target_link_libraries(imu yaml-cpp)


install(TARGETS
  imu
  DESTINATION lib/${PROJECT_NAME})

install (DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)


# Install Python scripts
install(PROGRAMS
  scripts/imu_calibration.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
