cmake_minimum_required(VERSION 2.8.3)
project(lpms_imu)

set(CMAKE_CXX_FLAGS "-std=c++14")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs 
  timesync
)

catkin_package(
   CATKIN_DEPENDS timesync)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

set(lpms_imu_node_SRCS
    src/lpms_imu_node.cpp)

find_library(Lp_LIBRARY LpSensor)

add_executable(lpms_imu_node ${lpms_imu_node_SRCS})

target_link_libraries(lpms_imu_node
  ${catkin_LIBRARIES}
  ${Lp_LIBRARY}
 )

add_dependencies(lpms_imu_node ${catkin_EXPORTED_TARGETS})
