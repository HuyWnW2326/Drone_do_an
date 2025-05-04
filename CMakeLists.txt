cmake_minimum_required(VERSION 3.8)
project(drone_pkg)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(drone_node src/drone.cpp)
ament_target_dependencies(drone_node rclcpp px4_msgs std_msgs)

add_executable(drone_SMC_node src/drone_SMC.cpp)
ament_target_dependencies(drone_SMC_node rclcpp px4_msgs std_msgs)

install(TARGETS
  drone_node
  drone_SMC_node
  DESTINATION lib/${PROJECT_NAME})
  
ament_package()
