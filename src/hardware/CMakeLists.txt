cmake_minimum_required(VERSION 3.0.2)
project(hardware)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  geometry_msgs
)

find_package(dynamixel_sdk REQUIRED)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

## INCLUDE_DIRS: uncomment this if your package contains header files
catkin_package(
 INCLUDE_DIRS include
)

## Specify additional locations of header files
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(servo src/servo_mx28.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(servo ${catkin_LIBRARIES} ${dynamixel_sdk_LIBRARIES})

# add_dependencies(stm_main iris_msgs_generate_messages_cpp)
# add_dependencies(stm_ballhandling iris_msgs_generate_messages_cpp)