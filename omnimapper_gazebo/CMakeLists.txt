cmake_minimum_required(VERSION 3.5)
project(omnimapper_ros)

# find dependencies
find_package(omnimapper_common REQUIRED)

omnimapper_package()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
