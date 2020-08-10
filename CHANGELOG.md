# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.0.6] - 2020-07-06

### Changed

- ROS2 Migration
  - Target ROS2 Foxy
- Update underlays
  - pin down gtsam and csm versions 

### Fixed

- Docker build
  - Optimize build

## [0.0.5] - 2020-04-02

### Added

- Docker Demo
  - New Dockerfile
- QoS control param

### Changed

- Launch files
  - Migrate launch format for ROS2
  - Update example plane plugin launch file
- Update underlays
  - build pcl_conversion unreleased in eloquent 

### Fixed

- Rviz
  - Correct tf_funtor for sensor to base

## [0.0.4] - 2020-02-25

### Added

- Documentation
  - New tutorials

### Changed

- ROS2 Migration
  - Target ROS2 Eloquent
  - Updates to tf2 API
- Omnimapper Migration
  - Plane plugin callbacks

### Fixed

- Cmake
  - header paths and linking
- Build warnings
  - Use system version of PCL

## [0.0.3] - 2019-11-20

### Changed

- Code Style
  - Adopt clang format
- ROS2 Migration
  - Target ROS2 Dashing
  - Use build to ament and colcon
  - Use rclcpp client API
  - Use package v3 format
  - Move msgs into separate package
  - Upgrade tf_conversions includes
  - Use appropriate sensor QoS

### Fixed

- Cmake
  - GTSAM header setup
- Header files
  - Scope CSM macros
- Build warnings
  - Build for Ubuntu 18.04
- C++ library patterns
  - Use of boost shared pointers

### Removed

- ROS1 Support

## [0.0.2] - 2015-02-12

### Added

- Rviz Demo Config
- Launch files
    - Updated Hardware Robot Demo

## [0.0.1] - 2014-06-01

### Added

- Initial release
- ROS1 wrapper
  - Services
    - Write trajectory to file
    - Visualize full cloud
    - Publish model
  - Launch files
    - OpenNI Camera Demo
    - Hardware Robot Demo
    - Simulated Robot Demo
  - Integrations
    - TF
      - http://wiki.ros.org/tf
    - Canonical Scan Matcher (CSM)
      - https://censi.science/software/csm/
    - Rviz
      - http://wiki.ros.org/rviz

[unreleased]: https://github.com/CogRob/omnimapper_ros/releases/tag/v0.0.6...HEAD
[0.0.6]: https://github.com/CogRob/omnimapper_ros/releases/tag/v0.0.5...v0.0.6
[0.0.5]: https://github.com/CogRob/omnimapper_ros/releases/tag/v0.0.4...v0.0.5
[0.0.4]: https://github.com/CogRob/omnimapper_ros/releases/tag/v0.0.3...v0.0.4
[0.0.3]: https://github.com/CogRob/omnimapper_ros/releases/tag/v0.0.2...v0.0.3
[0.0.2]: https://github.com/CogRob/omnimapper_ros/releases/tag/v0.0.1...v0.0.2
[0.0.1]: https://github.com/CogRob/omnimapper_ros/releases/tag/v0.0.1
