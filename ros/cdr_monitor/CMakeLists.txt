cmake_minimum_required(VERSION 3.8)
project(cdr_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(hesai_zed_fusion REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_executable(cdr_monitor_node
  src/main.cpp
  src/update_system_state.cpp
  src/calculate_ftti.cpp
)

target_include_directories(cdr_monitor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(cdr_monitor_node
  behaviortree_cpp::behaviortree_cpp
)

ament_target_dependencies(cdr_monitor_node
  rclcpp
  geometry_msgs
  sensor_msgs
  ament_index_cpp
  behaviortree_ros2
  visualization_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  hesai_zed_fusion
  diagnostic_msgs
)

add_executable(benchmark_node
  src/benchmark_node.cpp
)

target_include_directories(benchmark_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(benchmark_node
  rclcpp
  sensor_msgs
)

add_executable(recorder_node
  src/recorder_node.cpp
)

target_include_directories(recorder_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(recorder_node
  rclcpp
  sensor_msgs
  geometry_msgs
  hesai_zed_fusion
)

add_executable(latency_compare_node
  src/latency_compare_node.cpp
)

target_include_directories(latency_compare_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(latency_compare_node
  rclcpp
  sensor_msgs
)

install(TARGETS cdr_monitor_node benchmark_node recorder_node latency_compare_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/awcl_adapter.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY behavior_tree config urdf launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
