cmake_minimum_required(VERSION 3.8)
project(airflow)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sim_msgs REQUIRED)
find_package(coordinate_transform REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(PCL REQUIRED COMPONENTS common io octree)

add_library(${PROJECT_NAME} SHARED
  src/airflow.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  ament_index_cpp
  geometry_msgs
  sim_msgs
  coordinate_transform
)

target_link_libraries(${PROJECT_NAME}
  gz-sim::gz-sim
  gz-plugin::register
  ${PCL_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Install data files (force tables - small, direct copy)
install(DIRECTORY force_table/
  DESTINATION share/${PROJECT_NAME}/force_table
)

# Install PCD data directories as symlinks (too large to copy - 868MB)
install(CODE "
  set(_src \"${CMAKE_CURRENT_SOURCE_DIR}/pcd_mhi_plus_noise\")
  set(_dst \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/pcd_mhi_plus_noise\")
  if(EXISTS \${_src} AND NOT EXISTS \${_dst})
    execute_process(COMMAND \${CMAKE_COMMAND} -E create_symlink \${_src} \${_dst})
  endif()
  set(_src2 \"${CMAKE_CURRENT_SOURCE_DIR}/pcd_steady_flow_0_1\")
  set(_dst2 \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/pcd_steady_flow_0_1\")
  if(EXISTS \${_src2} AND NOT EXISTS \${_dst2})
    execute_process(COMMAND \${CMAKE_COMMAND} -E create_symlink \${_src2} \${_dst2})
  endif()
")

ament_package()
