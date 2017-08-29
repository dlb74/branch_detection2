cmake_minimum_required(VERSION 2.8.3)
project(branch_detection2)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
   pcl_conversions
   pcl_ros
   genmsg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



add_executable(cloud_segmenting_test src/cloud_segmenting_test.cpp)
target_link_libraries(cloud_segmenting_test ${catkin_LIBRARIES})
add_dependencies(cloud_segmenting_test branch_detection_gencpp)
