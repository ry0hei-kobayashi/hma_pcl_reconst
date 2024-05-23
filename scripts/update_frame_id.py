#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image 
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2

rospy.init_node("update_frame_id")

#Updating frame id for the error depth_front frame id does not match rgb_front frame id
class update_frame_id:
    def __init__(self):
        self.image = Image()
        self.pcl = PointCloud2()
        #subscribe to your specific sensors
        self.sub_raw = rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_raw/compressed", Image, self.callback_raw)
        self.sub_info = rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/camera_info", CameraInfo, self.callback_info)
        self.sub_pcl = rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", PointCloud2, self.callback_pcl)
        self.pub_raw = rospy.Publisher("/rgb/image_raw/image", Image, queue_size = 1)
        self.pub_info = rospy.Publisher("/rgb/camera_info", CameraInfo, queue_size = 1)
        self.pub_pcl = rospy.Publisher("/pcl/rectified_points", PointCloud2 ,queue_size = 1)
    def callback_raw(self, message):
        message.header.frame_id = "head_rgbd_sensor_depth_frame"
        self.pub_raw.publish(message)
    def callback_info(self, message):
        message.header.frame_id = "head_rgbd_sensor_depth_frame"
        self.pub_info.publish(message)
    def callback_pcl(self, message):
        message.header.frame_id = "head_rgbd_sensor_depth_frame"
        self.pub_pcl.publish(message)


update_frame_id = update_frame_id()
rospy.spin()

print("\nNode shutdown\n")
