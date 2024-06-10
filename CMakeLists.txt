cmake_minimum_required(VERSION 3.0.2)
project(hma_pcl_reconst)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  pluginlib
  image_transport
  message_filters
  cv_bridge
  sensor_msgs
  image_geometry
  depth_image_proc
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hma_pcl_reconst
  CATKIN_DEPENDS roscpp nodelet pluginlib image_transport message_filters cv_bridge sensor_msgs image_geometry depth_image_proc
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/nodelets/point_cloud_xyzrgb_nodelet.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
  nodelet_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

