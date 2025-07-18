cmake_minimum_required(VERSION 3.8)
project(drone_pkg)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ISMC_thesis_node src/ISMC_thesis.cpp)
ament_target_dependencies(ISMC_thesis_node rclcpp px4_msgs geometry_msgs std_msgs)

add_executable(ISMC_node src/ISMC.cpp)
ament_target_dependencies(ISMC_node rclcpp px4_msgs geometry_msgs std_msgs)

add_executable(PID_node src/PID.cpp)
ament_target_dependencies(PID_node rclcpp px4_msgs geometry_msgs std_msgs)

add_executable(SITL_node src/SITL.cpp)
ament_target_dependencies(SITL_node rclcpp px4_msgs geometry_msgs std_msgs)

add_executable(SMC_node src/SMC.cpp)
ament_target_dependencies(SMC_node rclcpp px4_msgs geometry_msgs std_msgs)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ISMC_thesis_node
  ISMC_node
  PID_node
  SITL_node
  SMC_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
