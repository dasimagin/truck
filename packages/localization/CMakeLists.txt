cmake_minimum_required(VERSION 3.8)
project(localization)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geom REQUIRED)
find_package(common REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(
  Boost
  COMPONENTS thread
             filesystem
             system
             program_options
             date_time
             chrono
             REQUIRED
)

add_executable(
  ${PROJECT_NAME}_node src/main.cpp src/localization_node.cpp
                       src/conversion.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}_node
  rclcpp
  geom
  common
  libpointmatcher
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions
  rosbag2_cpp
  Boost
)

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

target_include_directories(
  ${PROJECT_NAME}_node
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
         "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
