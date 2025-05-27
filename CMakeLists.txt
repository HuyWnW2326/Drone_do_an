cmake_minimum_required(VERSION 3.8)
project(drone_pkg)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(PID_node src/PID.cpp)
ament_target_dependencies(PID_node rclcpp px4_msgs geometry_msgs)

add_executable(SMC_Circle_node src/SMC_Circle.cpp)
ament_target_dependencies(SMC_Circle_node rclcpp px4_msgs geometry_msgs)

add_executable(SMC_Hold_node src/SMC_Hold.cpp)
ament_target_dependencies(SMC_Hold_node rclcpp px4_msgs geometry_msgs)

add_executable(PID_Torque_node src/PID_Torque.cpp)
ament_target_dependencies(PID_Torque_node rclcpp px4_msgs geometry_msgs)

add_executable(test_node src/test.cpp)
ament_target_dependencies(test_node rclcpp px4_msgs geometry_msgs)

add_executable(ISMC_Circle_node src/ISMC_Circle.cpp)
ament_target_dependencies(ISMC_Circle_node rclcpp px4_msgs geometry_msgs)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  PID_node
  PID_Torque_node
  SMC_Circle_node
  SMC_Hold_node
  test_node
  ISMC_Circle_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
