cmake_minimum_required(VERSION 2.8.3)
project(course_agv_slam_task)



set (CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  tf_conversions
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)


add_definitions(-std=c++11 -O2 -g -DNDEBUG)

catkin_package (
  # CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  # DEPENDS EIGEN3 PCL
  # INCLUDE_DIRS include
)


include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS
)



add_executable(icp src/icp.cpp)
target_link_libraries(icp 
${catkin_LIBRARIES}
${PCL_LIBRARIES} 
${OpenCV_LIBS}
)

add_executable(myicp src/myicp.cpp)
target_link_libraries(myicp 
${catkin_LIBRARIES}
${PCL_LIBRARIES} 
${OpenCV_LIBS}
)

add_executable(extraction src/extraction.cpp)
target_link_libraries(extraction
${catkin_LIBRARIES}
${PCL_LIBRARIES} 
)

add_executable(icp_lm src/icp_lm.cpp)
target_link_libraries(icp_lm
${catkin_LIBRARIES}
${PCL_LIBRARIES} 
)
