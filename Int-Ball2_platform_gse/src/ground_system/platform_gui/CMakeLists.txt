cmake_minimum_required(VERSION 3.8)
project(platform_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ib2_msgs REQUIRED)
find_package(platform_msgs REQUIRED)
find_package(communication_software REQUIRED)
find_package(gui_common REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Qt5
find_package(Qt5 REQUIRED COMPONENTS Widgets Multimedia Network)

# Qt auto-processing
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

add_compile_options(-fPIC -DQT_FORCE_ASSERTS)

# Hide internal symbols to avoid spdlog ABI conflict with ROS 2's rcl_logging_spdlog
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

# Qt5 resources
qt5_add_resources(QT5_RESOURCES "resources.qrc")

# Source files
file(GLOB_RECURSE SRC_FILES
  src/*.cpp
  src/model/*.cpp
  src/view/*.cpp
)
file(GLOB_RECURSE UI_FILES src/*.ui)

add_executable(${PROJECT_NAME}
  ${SRC_FILES}
  ${UI_FILES}
  ${QT5_RESOURCES}
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/oss
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  ib2_msgs
  platform_msgs
  communication_software
  gui_common
  ament_index_cpp
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  Qt5::Multimedia
  Qt5::Network
  fmt
)

# Install
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "*.launch" EXCLUDE
)

ament_package()
