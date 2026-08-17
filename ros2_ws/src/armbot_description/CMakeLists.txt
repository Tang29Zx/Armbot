cmake_minimum_required(VERSION 3.8)
project(armbot_description)

find_package(ament_cmake REQUIRED)

# Install URDF files
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
