cmake_minimum_required(VERSION 3.8)
project(debug_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ib2_msgs REQUIRED)
find_package(communication_software REQUIRED)
find_package(gui_common REQUIRED)

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
)

file(GLOB_RECURSE UI_FILES
    src/*.ui
    src/model/*.ui
)

add_executable(${PROJECT_NAME}
  ${SRC_FILES}
  ${UI_FILES}
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  ib2_msgs
  communication_software
  gui_common
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
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
