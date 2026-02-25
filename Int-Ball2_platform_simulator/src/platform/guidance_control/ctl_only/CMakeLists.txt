cmake_minimum_required(VERSION 3.8)
project(ctl_only)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ib2_msgs REQUIRED)
find_package(guidance_control_common REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(${PROJECT_NAME}
  src/att_controller.cpp
  src/att_profiler.cpp
  src/ctl.cpp
  src/ctl_body.cpp
  src/ctl_elements.cpp
  src/dtc.cpp
  src/pos_att_controller.cpp
  src/pos_att_profiler.cpp
  src/pos_controller.cpp
  src/pos_profiler.cpp
  src/thrust_allocator.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  ib2_msgs
  guidance_control_common
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
