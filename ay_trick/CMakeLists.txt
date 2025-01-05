cmake_minimum_required(VERSION 3.0.2)
project(ay_trick)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  ay_py
  rospy
)


## Uncomment this if the package has a setup.py.
catkin_python_setup()


## The catkin_package macro generates cmake config files for your package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ay_trick
#  CATKIN_DEPENDS ay_py rospy
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


## Install environment is not configured.

## Testing method is not configured.

