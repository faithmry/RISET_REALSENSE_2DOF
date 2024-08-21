cmake_minimum_required(VERSION 3.0.2)
project(vision)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Eigen3 REQUIRED)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vision
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

set(REALSENSE_DEPS
  realsense2
)

add_executable(realsense src/realsense.cpp include/vision/cv-helpers.hpp)
set_property(TARGET realsense PROPERTY CXX_STANDARD 11)
target_link_libraries(realsense  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${REALSENSE_DEPS})

add_executable(capture src/capture.cpp include/vision/cv-helpers.hpp include/vision/example-imgui.hpp)
set_property(TARGET capture PROPERTY CXX_STANDARD 11)
target_link_libraries(capture  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${REALSENSE_DEPS})

add_executable(ball src/ball.cpp)
set_property(TARGET ball PROPERTY CXX_STANDARD 11)
target_link_libraries(ball  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${EIGEN3_LIBRARIES})

# include_directories(../ ../../examples)