cmake_minimum_required(VERSION 3.8)
project(drone_pkg)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(drone_PID_node src/drone_PID.cpp)
ament_target_dependencies(drone_PID_node rclcpp px4_msgs std_msgs)

add_executable(drone_SMC_Circle_node src/drone_SMC_Circle.cpp)
ament_target_dependencies(drone_SMC_Circle_node rclcpp px4_msgs std_msgs)

add_executable(drone_SMC_Hold_node src/drone_SMC_Hold.cpp)
ament_target_dependencies(drone_SMC_Hold_node rclcpp px4_msgs std_msgs)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  drone_PID_node
  drone_SMC_Circle_node
  drone_SMC_Hold_node
  DESTINATION lib/${PROJECT_NAME})
  
ament_package()
