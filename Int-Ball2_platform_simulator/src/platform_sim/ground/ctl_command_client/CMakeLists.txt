cmake_minimum_required(VERSION 3.8)
project(ctl_command_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ib2_msgs REQUIRED)
find_package(guidance_control_common REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(${PROJECT_NAME}
  src/ctl_command_client.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  std_msgs
  ib2_msgs
  guidance_control_common
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  commandStandBy.txt
  commandKeepPose.txt
  commandStopping.txt
  commandRelativeTarget.txt
  commandRelativeTarget2.txt
  commandAbsoluteTarget.txt
  commandAbsoluteTarget02.txt
  commandRelease.txt
  commandDocking.txt
  commandDockingWithoutCorrection.txt
  commandScan.txt
  commandCollisionCase.txt
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
