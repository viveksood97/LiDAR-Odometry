cmake_minimum_required(VERSION 3.8)
project(lidar_odometry)

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

# Cant fix PCL warnings for now as PCL is just TRASH!!!
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(${PROJECT_NAME}_LIDAR_ODOM_SRCS
  src/lidar_odometry_node.cpp
)

set(${PROJECT_NAME}_LIDAR_ODOM_HEADERS
  include/${PROJECT_NAME}/lidar_odometry_node.hpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_LIDAR_ODOM_SRCS}
  ${${PROJECT_NAME}_LIDAR_ODOM_HEADERS}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "lidar_odometry::LidarOdometryNode"
  EXECUTABLE ${PROJECT_NAME}_lidar_odom_node_exe
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)

