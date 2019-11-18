cmake_minimum_required(VERSION 3.5)
project(omnimapper_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(omnimapper_ros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(GTSAM REQUIRED)
set(EIGEN_INCLUDE_DIRS ${GTSAM_DIR}/../../../include/gtsam/3rdparty/Eigen)

find_package(Boost REQUIRED COMPONENTS system)

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
  ${EIGEN_INCLUDE_DIRS}
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
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(ros_tf_utils src/ros_tf_utils.cpp)

add_library(csm_math_functions src/csm_math_functions.cpp)

add_library(ros_time_utils src/ros_time_utils.cpp)

add_library(canonical_scan src/canonical_scan.cpp)

add_library(try_headers src/try_headers.cpp)

add_library(canonical_scan_matcher_plugin src/canonical_scan_matcher_plugin.cpp)

add_library(tf_pose_plugin src/tf_pose_plugin.cpp)

add_library(get_transform_functor_tf src/get_transform_functor_tf.cpp)

add_library(csm_visualizer src/csm_visualizer.cpp)

add_library(omnimapper_visualizer_rviz src/omnimapper_visualizer_rviz.cpp)

add_library(omnimapper_ros_node src/omnimapper_ros_node.cpp)

add_library(omnimapper_ros src/omnimapper_ros.cpp)

ament_target_dependencies(
  omnimapper_ros omnimapper_ros_msgs
  ${Boost_LIBRARIES} ${PCL_LIBRARIES} rclcpp pcl_conversions
  omnimapper gtsam organized_segmentation_tools tcmalloc profiler tbb csm)

ament_target_dependencies(
  omnimapper_ros_node omnimapper_ros_msgs
  ${Boost_LIBRARIES} ${PCL_LIBRARIES} rclcpp pcl_conversions
  omnimapper gtsam organized_segmentation_tools tcmalloc profiler tbb csm)

ament_target_dependencies(
  canonical_scan_matcher_plugin omnimapper_ros_msgs
  ${Boost_LIBRARIES} ${PCL_LIBRARIES} rclcpp pcl_conversions
  omnimapper gtsam organized_segmentation_tools tcmalloc profiler tbb csm)

ament_target_dependencies(
  try_headers omnimapper_ros_msgs
  ${Boost_LIBRARIES} ${PCL_LIBRARIES} rclcpp pcl_conversions
  omnimapper gtsam organized_segmentation_tools tcmalloc profiler tbb csm)

ament_target_dependencies(
  omnimapper_visualizer_rviz omnimapper_ros_msgs
  ${Boost_LIBRARIES} ${PCL_LIBRARIES} rclcpp pcl_conversions
  omnimapper gtsam organized_segmentation_tools tcmalloc profiler tbb csm)

add_executable(time_test src/time_test.cpp)
ament_target_dependencies(
  time_test omnimapper_ros_msgs
  ${Boost_LIBRARIES} ${PCL_LIBRARIES} rclcpp pcl_conversions
  omnimapper gtsam organized_segmentation_tools tcmalloc profiler tbb csm)

target_link_libraries(
  time_test ros_time_utils ${Boost_LIBRARIES} ${PCL_LIBRARIES}
  omnimapper gtsam organized_segmentation_tools tcmalloc
  profiler tbb csm)

ament_target_dependencies(
  csm_visualizer omnimapper_ros_msgs
  ${Boost_LIBRARIES} ${PCL_LIBRARIES} rclcpp pcl_conversions
  omnimapper gtsam organized_segmentation_tools tcmalloc profiler tbb csm)

install(TARGETS
  time_test ros_time_utils
  DESTINATION lib/${PROJECT_NAME})

# add_library(omnimapper_ros
#   src/ros_time_utils.cpp
#   src/ros_tf_utils.cpp
#   src/tf_pose_plugin.cpp
#   src/omnimapper_visualizer_rviz.cpp
#   src/get_transform_functor_tf.cpp
#   src/canonical_scan_matcher_plugin.cpp
#   src/canonical_scan.cpp
#   src/csm_math_functions.cpp
#   src/csm_visualizer.cpp
#   src/omnimapper_ros.cpp
# )
# target_link_libraries(
#   omnimapper_ros
#   ${Boost_LIBRARIES} ${PCL_LIBRARIES} omnimapper gtsam
#   organized_segmentation_tools tcmalloc profiler tbb csm)
# ament_target_dependencies(
#   omnimapper_ros
#   ${Boost_LIBRARIES} ${PCL_LIBRARIES}
#   omnimapper gtsam organized_segmentation_tools tcmalloc profiler tbb csm)
#
# add_executable(omnimapper_ros_node src/omnimapper_ros_node.cpp)
# target_link_libraries(
#   omnimapper_ros_node
#   ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES}
#   omnimapper gtsam tcmalloc profiler)
# ament_target_dependencies(omnimapper_ros_node
#   rclcpp tf2 tf2_ros visualization_msgs interactive_markers geometry_msgs
#   sensor_msgs laser_geometry pcl_conversions)
#
# install(TARGETS omnimapper_ros_node DESTINATION lib/${PROJECT_NAME})
#
ament_package()
