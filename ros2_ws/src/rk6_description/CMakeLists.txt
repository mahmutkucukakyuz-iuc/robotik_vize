cmake_minimum_required(VERSION 3.8)
project(rk6_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config launch rviz worlds meshes
  DESTINATION share/${PROJECT_NAME}
)

# Gazebo'nun model kaynaklarini bulabilmesi icin (GZ_SIM_RESOURCE_PATH)
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
