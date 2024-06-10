#ifndef HMA_PCL_RECONST_POINT_CLOUD_XYZRGB_NODELET_H_
#define HMA_PCL_RECONST_POINT_CLOUD_XYZRGB_NODELET_H_

#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_proc/depth_traits.h>
#include <cv_bridge/cv_bridge.h>

namespace hma_pcl_reconst {

class PointCloudXyzrgbNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();

private:
  ros::NodeHandlePtr rgb_nh_;
  boost::shared_ptr<image_transport::ImageTransport> rgb_it_, depth_it_;
  
  image_transport::SubscriberFilter sub_depth_, sub_rgb_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> ExactSyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
  typedef message_filters::Synchronizer<ExactSyncPolicy> ExactSynchronizer;
  boost::shared_ptr<Synchronizer> sync_;
  boost::shared_ptr<ExactSynchronizer> exact_sync_;

  boost::mutex connect_mutex_;
  typedef sensor_msgs::PointCloud2 PointCloud;
  ros::Publisher pub_point_cloud_;

  image_geometry::PinholeCameraModel model_;
  sensor_msgs::CameraInfoConstPtr info_msg;

  void connectCb();
  void imageCb(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& rgb_msg);
  template<typename T>
  void convert(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& rgb_msg, const PointCloud::Ptr& cloud_msg, int red_offset, int green_offset, int blue_offset, int color_step);
};

} // namespace hma_pcl_reconst

#endif // HMA_PCL_RECONST_POINT_CLOUD_XYZRGB_NODELET_H_

