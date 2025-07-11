cmake_minimum_required(VERSION 3.8)
project(turtlebot3_joy_teleop)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Teleop executable
add_executable(teleop_node src/teleop_node.cpp)
ament_target_dependencies(teleop_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

# Install targets
install(TARGETS teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py"
)

ament_package()
