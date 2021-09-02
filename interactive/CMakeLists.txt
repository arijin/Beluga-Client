cmake_minimum_required (VERSION 3.5)
project(interact)
set(CMAKE_CXX_STANDARD 11)

#---find_package---
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  jsk_recognition_msgs
  me120_msgs
)
find_package(OpenCV REQUIRED) 
find_package(Eigen3 REQUIRED)
#------------------

catkin_package()

#---include_directories---
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}  
)
#-------------------------

#---生成可执行程序---

#----------1-----------#
add_executable(pointing_ball
src/pointing_ball.cpp
src/marker.cpp
)
target_link_libraries(pointing_ball
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
)

#----------2-----------#
add_executable(range
src/pointing_range.cpp
src/marker.cpp
)
target_link_libraries(range
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
)

#----------3-----------#
add_executable(start
src/start.cpp
)
target_link_libraries(start
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
)
