cmake_minimum_required(VERSION 2.8.3)
project(record_baxter_eef_trajectory)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  moveit_ros_planning_interface
  image_geometry
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  tf
  message_generation
)

find_package(OpenCV REQUIRED COMPONENTS features2d nonfree)

# EIGEN
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system )
find_package(PCL 1.7 REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_path(RGBD_UTILS_INCLUDE_DIR rgbd_utils/rgbd_subscriber.hpp)
find_library(RGBD_UTILS_LIBRARY NAMES rgbd_utils)

find_path(IMAGE_PROCESSING_INCLUDE_DIR image_processing/MotionDetection.h)
find_library(IMAGE_PROCESSING_LIBRARY NAMES image_processing)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")

add_service_files(
  FILES
  Recordtraj.srv
  Getrealmodelstate.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
#  LIBRARIES crustcrawler_cppkdl
#  CATKIN_DEPENDS moveit_ros_planning_interface roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(include /usr/local/include ${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

file ( GLOB_RECURSE all_sources src/lib/*.cpp )

## independent programs
add_executable(record_baxter_eef_trajectory src/record_baxter_eef_trajectory.cpp)
target_link_libraries(record_baxter_eef_trajectory ${catkin_LIBRARIES}  )

add_executable(convert_quaterion_angles_to_matrix src/convert_quaterion_angles_to_matrix.cpp)
target_link_libraries(convert_quaterion_angles_to_matrix ${catkin_LIBRARIES}  )

add_executable(record_object_and_eef_position src/record_object_and_eef_position.cpp ${all_sources} src/parameters.hpp)
target_link_libraries(record_object_and_eef_position /usr/local/lib/libaruco.so aruco ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES} rgbd_utils image_processing)

## service program
add_executable(record_left_eef_trajectory_service src/services/record_left_eef_trajectory_service.cpp ${all_sources} src/parameters.hpp)
target_link_libraries(record_left_eef_trajectory_service /usr/local/lib/libaruco.so aruco ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} 
${catkin_LIBRARIES} ${Boost_LIBRARIES}
rgbd_utils image_processing )
add_dependencies(record_left_eef_trajectory_service ${catkin_EXPORTED_TARGETS})

add_executable(get_real_model_state_service src/services/get_real_model_state_service.cpp ${all_sources} src/parameters.hpp)
target_link_libraries(get_real_model_state_service /usr/local/lib/libaruco.so aruco ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}
${catkin_LIBRARIES} ${Boost_LIBRARIES}
rgbd_utils image_processing )
add_dependencies(get_real_model_state_service ${catkin_EXPORTED_TARGETS})
