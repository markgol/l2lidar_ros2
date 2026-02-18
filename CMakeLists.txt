cmake_minimum_required(VERSION 3.22)

project(l2lidar_ros2
	VERSION 0.1.1
	LANGUAGES CXX
)

# ============================
# Compiler settings
# ============================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Enable Qt MOC/UIC/RCC automatically
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# ============================
# ROS2
# ============================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# ============================
# Qt 6.10 (force your install)
# ============================
set(QT610_PATH "/opt/Qt/6.10.2/gcc_arm64")

list(PREPEND CMAKE_PREFIX_PATH
  ${QT610_PATH}
  ${QT610_PATH}/lib/cmake
)

set(Qt6_DIR "${QT610_PATH}/lib/cmake/Qt6")

find_package(Qt6 REQUIRED COMPONENTS Core Network)

# ============================
# Include directories
# ============================
include_directories(
  include
)

# ============================
# Sources
# ============================
set(SOURCES
  src/l2lidar_node.cpp
  src/L2lidar.cpp
)

set(HEADERS
  include/L2lidar.h
  include/PCpoint.h
  include/unitree_lidar_protocolL2.h
  include/unitree_lidar_utilitiesL2.h
  include/quaternion.h
)

# ============================
# Executable
# ============================
add_executable(l2lidar_node
  ${SOURCES}
  ${HEADERS}
  src/main.cpp
  include/l2lidar_node.hpp
)

target_link_libraries(l2lidar_node
  Qt6::Core
  Qt6::Network
)

ament_target_dependencies(l2lidar_node
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  geometry_msgs
)

target_include_directories(l2lidar_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ============================
# Install
# ============================
install(TARGETS l2lidar_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# ============================
# Package export
# ============================
ament_package()
