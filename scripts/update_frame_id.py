#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo

class UpdateFrameID:
    def __init__(self):
        rospy.init_node("update_frame_id")

        self.pub_raw = rospy.Publisher("/rgb/image_rect_color", Image, queue_size=1)
        self.pub_depth = rospy.Publisher("/depth_registered/image_rect", Image, queue_size=1)
        self.pub_info = rospy.Publisher("/rgb/camera_info", CameraInfo, queue_size=1)
        #important
        rospy.sleep(1)


        self.sub_raw = rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_rect_color", Image, self.callback_raw)
        self.sub_depth = rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw", Image, self.callback_depth)
        self.sub_info = rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/camera_info", CameraInfo, self.callback_info)

        #important
        rospy.sleep(1)
        rospy.loginfo("UpdateFrameID node initialized.")

    def callback_raw(self, message):
        try:
            message.header.frame_id = "head_rgbd_sensor_rgb_frame"
            self.pub_raw.publish(message)
            #rospy.loginfo("Published updated RGB image frame ID.")
        except Exception as e:
            rospy.logerr("Error in callback_raw: %s", str(e))

    def callback_depth(self, message):
        try:
            message.header.frame_id = "head_rgbd_sensor_rgb_frame"
            self.pub_depth.publish(message)
            #rospy.loginfo("Published updated depth image frame ID.")
        except Exception as e:
            rospy.logerr("Error in callback_depth: %s", str(e))

    def callback_info(self, message):
        try:
            message.header.frame_id = "head_rgbd_sensor_rgb_frame"
            self.pub_info.publish(message)
            #rospy.loginfo("Published updated camera info frame ID.")
        except Exception as e:
            rospy.logerr("Error in callback_info: %s", str(e))

if __name__ == '__main__':
    try:
        update_frame_id = UpdateFrameID()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node shutdown.")

