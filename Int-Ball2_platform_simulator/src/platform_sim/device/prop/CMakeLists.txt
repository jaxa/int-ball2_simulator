cmake_minimum_required(VERSION 3.8)
project(prop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(WITH_PCA9685 "execute with PCA9685 board" OFF)
if(WITH_PCA9685)
  add_definitions(-DWITH_PCA9685)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ib2_msgs REQUIRED)

add_executable(${PROJECT_NAME}
  src/prop.cpp
  src/prop_manager.cpp
  src/prop_tlmcmd.cpp
  src/prop_pca9685.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE include)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  ib2_msgs
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
