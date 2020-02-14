cmake_minimum_required(VERSION 3.5)
project(omnimapper_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(omnimapper REQUIRED)
find_package(omnimapper_ros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PkgConfig)

pkg_check_modules(csm REQUIRED csm)
link_directories(${csm_LIBRARY_DIRS})

find_package(GTSAM REQUIRED)

find_package(Boost REQUIRED)

find_package(PCL 1.7 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})

include_directories(
  include
)

# Set include directories
include_directories(
  SYSTEM
  ${omnimapper_ros_msgs_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${csm_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${rclcpp_INCLUDE_DIR}
  ${tf2_INCLUDE_DIR}
  ${tf2_ros_INCLUDE_DIR}
  ${tf2_eigen_INCLUDE_DIR}
  ${visualization_msgs_INCLUDE_DIR}
  ${geometry_msgs_INCLUDE_DIR}
  ${sensor_msgs_INCLUDE_DIR}
  ${laser_geometry_INCLUDE_DIR}
  ${pcl_conversions_INCLUDE_DIR}
  ${tf2_geometry_msgs_INCLUDE_DIR}
)

set(library_name ${PROJECT_NAME})
add_library(${library_name} SHARED
  src/canonical_scan.cpp
  src/canonical_scan_matcher_plugin.cpp
  src/csm_math_functions.cpp
  src/csm_visualizer.cpp
  src/get_transform_functor_tf.cpp
  src/omnimapper_ros.cpp
  src/omnimapper_visualizer_rviz.cpp
  src/ros_tf_utils.cpp
  src/ros_time_utils.cpp
  src/tf_pose_plugin.cpp
)

set(dependencies
  omnimapper
  omnimapper_ros_msgs
  rclcpp
  pcl_conversions
  omnimapper
  csm
  tf2
  tf2_ros
  tf2_msgs
  laser_geometry
  sensor_msgs
  visualization_msgs
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_link_libraries( ${library_name}
  omnimapper
  gtsam
  ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${orocos_kdl_LIBRARIES} ${csm_LIBRARIES} gsl)

add_executable(omnimapper_ros_node src/omnimapper_ros_node.cpp)
ament_target_dependencies(omnimapper_ros_node
  ${dependencies}
)

target_link_libraries(omnimapper_ros_node
  omnimapper
  ${library_name}
  # gtsam
  # ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${orocos_kdl_LIBRARIES} ${csm_LIBRARIES} gsl
)

install(TARGETS
  ${library_name}
  omnimapper_ros_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_copyright_FOUND TRUE)
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_export_include_directories(include)
ament_export_libraries(
  ${library_name}
)
ament_package()
