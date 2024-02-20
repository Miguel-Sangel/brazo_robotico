cmake_minimum_required(VERSION 3.8)
project(arips_arm_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(serial REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# add_executable(arips_arm_node src/arips_arm_node.cpp)
add_executable(arips_arm_node src/arips_arm_node.cpp)
ament_target_dependencies(arips_arm_node rclcpp std_msgs serial)

add_executable(comando_J_T_C src/comando_J_T_C.cpp)
ament_target_dependencies(comando_J_T_C rclcpp sensor_msgs trajectory_msgs)


install(TARGETS
    arips_arm_node
    comando_J_T_C
    DESTINATION lib/${PROJECT_NAME})

install(  DIRECTORY
  .
  DESTINATION share/${PROJECT_NAME}/
  
)

ament_export_include_directories(
  include
)

ament_package()
