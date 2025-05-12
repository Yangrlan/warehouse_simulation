cmake_minimum_required(VERSION 3.5)
project(bcr_bot)

find_package(catkin REQUIRED)

install(DIRECTORY launch meshes models urdf worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

catkin_package()
