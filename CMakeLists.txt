cmake_minimum_required(VERSION 3.8)
project(drone_pkg)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(PID_node src/PID.cpp)
ament_target_dependencies(PID_node rclcpp px4_msgs geometry_msgs std_msgs)

add_executable(SMC_Circle_node src/SMC_Circle.cpp)
ament_target_dependencies(SMC_Circle_node rclcpp px4_msgs geometry_msgs std_msgs)

add_executable(PID_Circle_node src/PID_Circle.cpp)
ament_target_dependencies(PID_Circle_node rclcpp px4_msgs geometry_msgs std_msgs)

add_executable(ISMC_Circle_node src/ISMC_Circle.cpp)
ament_target_dependencies(ISMC_Circle_node rclcpp px4_msgs geometry_msgs std_msgs)

add_executable(SITL_node src/SITL.cpp)
ament_target_dependencies(SITL_node rclcpp px4_msgs geometry_msgs std_msgs)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  PID_node
  PID_Circle_node
  SMC_Circle_node
  ISMC_Circle_node
  SITL_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
