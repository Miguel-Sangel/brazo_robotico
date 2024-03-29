cmake_minimum_required(VERSION 3.5)
project(mis_controladores_hardware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# set(CMAKE_POSITION_INDEPENDENT_CODE ON) ## no parece funcionar
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)


## COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/diffbot_system.cpp
  src/system_position_only.cpp
  src/system_multi_interface.cpp
  src/system_with_sensor.cpp
  src/actuator.cpp
  src/external_force_torque_sensor.cpp
  src/Brazo_posicion.cpp
  src/arips_hardware.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  serial
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROS2_CONTROL_DEMO_HARDWARE_BUILDING_DLL")

# Export hardware pligins
pluginlib_export_plugin_description_file(hardware_interface ros2_control_hardware.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)
ament_package()
