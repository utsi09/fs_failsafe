cmake_minimum_required(VERSION 3.8)
project(hesai_zed_fusion)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleInfo.msg"
  "msg/ObstacleArray.msg"
  DEPENDENCIES std_msgs
)

# Install Python node
install(PROGRAMS
  scripts/fusion_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
