# omnimapper_ros

## Installation

### Install Omnimapper

TODO: integrate this section with old installation instructions and new updates

Get omnimapper:
``
git clone https://github.com/CogRob/omnimapper.git
``

Install Omnimapper dependencies as detailed on this page: https://github.com/CognitiveRobotics/omnimapper/wiki/Installation-&-Dependencies

Install omnimapper as detailed on above page

Make sure to sudo make install



### Install Canonical Scan Matcher (csm)

NOTES / TODO:
currently install csm from source (below). couldn't get ros-kinetic-csm to link properly 


In your ros_workspace/src folder:

``
git clone https://github.com/AndreaCensi/csm.git
``

GSL is a depencency of CSM, so install: 

``
sudo apt install libgsl-dev
``

In your ros_workspace:
``
catkin_make
catkin_make install
``

### Install 


Clone this repository into your ros_workspace/src folder:
``
git clone https://github.com/CogRob/omnimapper_ros.git
``

TODO: don't think we still need this:

Update PKG_CONFIG_PATH so it can find csm.pc:
``
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:<ros_workspace>/install/lib/pkgconfig/
``

From ros_workspace/src:
``
catkin_make
``



TODO: fix library linking in CMakeLists since it's stupid right now


