#ifndef ORB_SLAM2_ROS2_STEREO_NODE_HPP_
#define ORB_SLAM2_ROS2_STEREO_NODE_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "System.h"
#include "ros2/base_node.hpp"

namespace ORB_SLAM2
{
class StereoNode : public BaseNode
{
public:
  explicit StereoNode(const rclcpp::NodeOptions & options);

private:
  void imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr left_img_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr right_img_msg);

  using FilterSub = message_filters::Subscriber<sensor_msgs::msg::Image>;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;
  FilterSub left_image_sub_;
  FilterSub right_image_sub_;
  std::shared_ptr<Sync> sync_;
};
}  // namespace ORB_SLAM2
#endif  // ORB_SLAM2_ROS2_MONO_NODE_HPP_