cmake_minimum_required(VERSION 2.8.3)
project(test_py)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
#  test_cpp
)

catkin_python_setup()

catkin_package()
