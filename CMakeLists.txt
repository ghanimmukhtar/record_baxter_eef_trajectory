cmake_minimum_required(VERSION 2.8.3)
project(record_baxter_eef_trajectory)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Boost REQUIRED COMPONENTS system filesystem)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

catkin_package(
 #INCLUDE_DIRS include
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(record_baxter_eef_trajectory src/record_baxter_eef_trajectory.cpp)
target_link_libraries(record_baxter_eef_trajectory ${catkin_LIBRARIES}  )
