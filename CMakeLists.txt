cmake_minimum_required(VERSION 3.0.2)
project(hma_pcl_reconst)

find_package(catkin REQUIRED COMPONENTS
  image_proc
  depth_image_proc
  rospy
  sensor_msgs
  cmake_modules
  cv_bridge
  eigen_conversions
  image_geometry
  image_transport
  message_filters
  nodelet
  sensor_msgs
  stereo_msgs
  tf2
  tf2_ros

)

#catkin_package(
#) 
catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp nodelet sensor_msgs pcl_ros image_geometry sensor_msgs 
    )


#find_package(Boost REQUIRED)
#find_package(Eigen3 QUIET)
#if(NOT EIGEN3_FOUND)
#  find_package(Eigen REQUIRED)
#  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
#endif()
#find_package(OpenCV REQUIRED)

#include_directories(include ${BOOST_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(hma_pcl_reconst src/point_cloud_xyzrgb.cpp)

target_link_libraries(hma_pcl_reconst ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(DIRECTORY include/hma_pcl_reconst_nodelets
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h")

#install(TARGETS hma_pcl_reconst_nodelets
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)
install(FILES hma_pcl_reconst.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


target_link_libraries(hma_pcl_reconst ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(hma_pcl_reconst ${catkin_EXPORTED_LIBRARIES})
endif()


include_directories(
  ${catkin_INCLUDE_DIRS}
)
