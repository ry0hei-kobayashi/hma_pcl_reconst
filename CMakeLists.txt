cmake_minimum_required(VERSION 3.0.2)
project(hma_pcl_reconst)

find_package(catkin REQUIRED COMPONENTS
  image_proc
  depth_image_proc
  rospy
  sensor_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
