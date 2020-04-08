cmake_minimum_required(VERSION 3.5)
project(omnimapper_ros_msgs)

find_package(ament_cmake REQUIRED)
find_package(omnimapper_common REQUIRED)
find_package(rosidl_default_generators REQUIRED)

omnimapper_package()

set(srv_files
  "srv/OutputMapTSDF.srv"
  "srv/PublishModel.srv"
  "srv/VisualizeFullCloud.srv"
  "srv/WriteTrajectoryFile.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
