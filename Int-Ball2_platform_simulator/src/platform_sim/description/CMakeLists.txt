cmake_minimum_required(VERSION 3.8)
project(description)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install URDF files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install model files (URDF models, meshes, textures)
install(DIRECTORY model/
  DESTINATION share/${PROJECT_NAME}/model
)

# Install media files (meshes, materials, textures)
install(DIRECTORY media/
  DESTINATION share/${PROJECT_NAME}/media
)

# Install rviz config
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Install scripts
install(DIRECTORY script/
  DESTINATION share/${PROJECT_NAME}/script
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
