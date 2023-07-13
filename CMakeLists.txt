cmake_minimum_required(VERSION 3.0.2)
project(HubMotor_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES HubMotor_pkg
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/HubMotor_pkg.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(control src/control.cpp)
add_executable(control_mode1 src/control_mode1.cpp)
add_executable(control_mode2 src/control_mode2.cpp)
add_executable(control_mode3 src/control_mode3.cpp)
add_executable(control_mode4 src/control_mode4.cpp)
add_executable(control_mode5 src/control_mode5.cpp)
add_executable(HubMotor src/HubMotor.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# 寻找opencv库
#find_package(OpenCV REQUIRED)
# 添加头文件
#include_directories(${OpenCV_INCLUDE_DIRS})

target_link_libraries(control
 ${catkin_LIBRARIES}
)
target_link_libraries(control_mode1
 ${catkin_LIBRARIES}
)
target_link_libraries(control_mode2
 ${catkin_LIBRARIES}
)
target_link_libraries(control_mode3
 ${catkin_LIBRARIES}
)
target_link_libraries(control_mode4
 ${catkin_LIBRARIES}
)
target_link_libraries(control_mode5
 ${catkin_LIBRARIES}
)
target_link_libraries(HubMotor 
 ${catkin_LIBRARIES}
 libpthread.so
 #libcontrolcan.so 
 #${PROJECT_SOURCE_DIR}/lib/lib_emuc2_64.a
)

