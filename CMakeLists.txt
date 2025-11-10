cmake_minimum_required(VERSION 3.10)
project(bt_aruco_landing)

# Set C++ version
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS ON) #...with compiler extensions like gnu++11

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(OpenCV REQUIRED)

find_package(dji_msdk_ros REQUIRED)

include_directories(
  /home/tom/aruco-3.1.12/src
)

link_directories(
  /home/tom/aruco-3.1.12/build
)
# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Behavior Tree nodes
add_library(bt_nodes
  src/bt_nodes/detect_aruco.cpp
  src/bt_nodes/align_to_aruco.cpp
  src/bt_nodes/land_on_marker.cpp
  src/bt_nodes/go_to_GPS.cpp
  src/bt_nodes/get_GPS_goal.cpp
  src/bt_nodes/land_requested_check.cpp
)
ament_target_dependencies(bt_nodes
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  behaviortree_cpp_v3
  tf2_geometry_msgs
  sensor_msgs
  cv_bridge
  dji_msdk_ros
)

# BT Landing main executable
add_executable(bt_landing_node src/bt_landing_node.cpp)
target_link_libraries(bt_landing_node bt_nodes)
ament_target_dependencies(bt_landing_node
  rclcpp
  behaviortree_cpp_v3
  geometry_msgs
  ament_index_cpp
  cv_bridge
  sensor_msgs
  dji_msdk_ros
)
target_link_libraries(bt_nodes
  ${OpenCV_LIBRARIES}
  aruco
)
# Install executables
install(TARGETS
  bt_nodes
  bt_landing_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install BT tree XMLs
install(DIRECTORY behaviour_trees
  DESTINATION share/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()