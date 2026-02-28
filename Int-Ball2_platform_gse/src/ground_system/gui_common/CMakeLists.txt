cmake_minimum_required(VERSION 3.8)
project(gui_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ib2_msgs REQUIRED)
find_package(platform_msgs REQUIRED)
find_package(communication_software REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Qt5 - use system Qt5
find_package(Qt5 REQUIRED COMPONENTS Widgets Network)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)

# Create code from a list of Qt designer ui files
set(CMAKE_AUTOUIC ON)

add_compile_options(-fPIC -DQT_FORCE_ASSERTS)

file(GLOB_RECURSE SRC_FILES
    src/*.cpp
    src/model/*.cpp
    src/view/*.cpp
    src/exception/*.cpp
    src/ui/*.cpp
)

file(GLOB_RECURSE UI_FILES src/ui/*.ui)

add_library(${PROJECT_NAME} SHARED
  ${SRC_FILES}
  ${UI_FILES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  ib2_msgs
  platform_msgs
  communication_software
  std_msgs
  geometry_msgs
  builtin_interfaces
  action_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rviz_common
  rviz_default_plugins
  ament_index_cpp
)

target_link_libraries(${PROJECT_NAME}
  yaml-cpp
  Qt5::Widgets
  Qt5::Network
)

# Install library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY src/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  ib2_msgs
  platform_msgs
  communication_software
  std_msgs
  geometry_msgs
  builtin_interfaces
  action_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rviz_common
  rviz_default_plugins
  ament_index_cpp
)

ament_package()
