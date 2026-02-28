cmake_minimum_required(VERSION 3.8)
project(operator_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ib2_msgs REQUIRED)
find_package(communication_software REQUIRED)
find_package(gui_common REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Qt5
find_package(Qt5 REQUIRED COMPONENTS Widgets Multimedia Svg Xml Scxml Network)

# VLC
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBVLC REQUIRED libvlc)

# yaml-cpp
find_package(yaml-cpp REQUIRED)

# Qt auto-processing
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

add_compile_options(-fPIC -DQT_FORCE_ASSERTS)

# Qt5 resources
qt5_add_resources(QT5_RESOURCES "resources.qrc")

# rviz2 plugin library (custom view controller + tool)
add_library(operator_gui_rviz_plugins SHARED
  src/main_camera_view_controller.cpp
  src/set_target_orientation_tool.cpp
)
target_include_directories(operator_gui_rviz_plugins PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
ament_target_dependencies(operator_gui_rviz_plugins
  rclcpp
  pluginlib
  gui_common
  rviz_common
  rviz_default_plugins
  rviz_rendering
  rviz_ogre_vendor
)
target_link_libraries(operator_gui_rviz_plugins
  Qt5::Widgets
  fmt
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# Source files (exclude plugin sources from executable to avoid duplicate symbols)
file(GLOB_RECURSE SRC_FILES
  src/*.cpp
  src/model/*.cpp
  src/view/*.cpp
)
list(REMOVE_ITEM SRC_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main_camera_view_controller.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/set_target_orientation_tool.cpp
)

file(GLOB_RECURSE UI_FILES src/*.ui)

# SCXML state charts
file(GLOB_RECURSE SCXML_FILES scxml/*.scxml)
qt5_add_statecharts(QT5_SCXML_COMPILED ${SCXML_FILES})

add_executable(${PROJECT_NAME}
  ${SRC_FILES}
  ${UI_FILES}
  ${QT5_SCXML_COMPILED}
  ${QT5_RESOURCES}
)

# Version script: spdlogシンボルの衝突を防止する.
# rviz_rendering等のspdlog ABI不一致によるクラッシュを回避.
target_link_options(${PROJECT_NAME} PRIVATE
  "LINKER:--version-script=${CMAKE_CURRENT_SOURCE_DIR}/hide_spdlog.ver"
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${LIBVLC_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pluginlib
  ib2_msgs
  communication_software
  gui_common
  rviz_common
  rviz_default_plugins
  rviz_rendering
  rviz_ogre_vendor
  visualization_msgs
  geometry_msgs
  sensor_msgs
  builtin_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(${PROJECT_NAME}
  operator_gui_rviz_plugins
  Qt5::Widgets
  Qt5::Multimedia
  Qt5::Svg
  Qt5::Xml
  Qt5::Scxml
  Qt5::Network
  ${LIBVLC_LIBRARIES}
  fmt
  yaml-cpp
)

# Install
install(TARGETS ${PROJECT_NAME} operator_gui_rviz_plugins
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "*.launch" EXCLUDE
)

ament_package()
