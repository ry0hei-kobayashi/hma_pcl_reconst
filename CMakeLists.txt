cmake_minimum_required(VERSION 3.0.2)
project(hma_pcl_reconst)

find_package(catkin REQUIRED COMPONENTS
  rospy
  image_proc
  depth_image_proc
  sensor_msgs

  pluginlib
  nodelet
)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    )


add_library(${PROJECT_NAME} src/nodelets/point_cloud_xyzrgb.cpp)

include_directories(
	  ${catkin_INCLUDE_DIRS}
	  )


target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
#target_link_libraries(hma_pcl_reconst ${catkin_LIBRARIES})



install(FILES plugin_description.xml
         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
