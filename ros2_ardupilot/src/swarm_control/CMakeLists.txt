cmake_minimum_required(VERSION 3.5)
project(swarm_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(fastcdr REQUIRED)

# Specify additional locations of header files
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${mavros_msgs_INCLUDE_DIRS}
  ${fastcdr_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(leader_swarm src/leader_swarm.cpp)
ament_target_dependencies(leader_swarm rclcpp std_msgs geometry_msgs mavros_msgs)

# Install executables
install(TARGETS
  leader_swarm
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/leader_swarm.py
  scripts/f1_swarm.py
  scripts/f2_swarm.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()