cmake_minimum_required(VERSION 3.8)
project(lidar_odometry)

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

# Cant fix PCL warnings for now as PCL is just TRASH!!!
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# --- Lidar Odometry Node Component ---
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

# --- Odometry Analyzer (CSV Logger) Component ---
set(${PROJECT_NAME}_ODOM_ANALYZER_SRCS
  src/odom_analyzer.cpp
)

set(${PROJECT_NAME}_ODOM_ANALYZER_HEADERS
  include/${PROJECT_NAME}/odom_analyzer.hpp
)

ament_auto_add_library(${PROJECT_NAME}_odom_analyzer SHARED
  ${${PROJECT_NAME}_ODOM_ANALYZER_SRCS}
  ${${PROJECT_NAME}_ODOM_ANALYZER_HEADERS}
)

rclcpp_components_register_node(${PROJECT_NAME}_odom_analyzer
  PLUGIN "lidar_odometry::OdometryAnalyzer"
  EXECUTABLE  ${PROJECT_NAME}_odom_analyzer_node_exe
)

install(PROGRAMS
  scripts/orchestrator
  scripts/post_process_results
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE launch
)
