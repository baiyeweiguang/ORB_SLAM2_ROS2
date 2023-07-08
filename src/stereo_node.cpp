#include "ros2/stereo_node.hpp"

#include <cv_bridge/cv_bridge.h>

#include <memory>
#include <sensor_msgs/msg/detail/image__struct.hpp>

#include "ros2/base_node.hpp"

namespace ORB_SLAM2
{

StereoNode::StereoNode(const rclcpp::NodeOptions & options)
: BaseNode(System::eSensor::STEREO, options)
{
  RCLCPP_INFO(this->get_logger(), "Starting ORB-SLAM2 Stereo node");
  std::string left_image_topic =
    this->declare_parameter("camera.left_image_topic", std::string("left/image_raw"));
  std::string right_image_topic =
    this->declare_parameter("camera.right_image_topic", std::string("right/image_raw"));
  left_image_sub_.subscribe(this, left_image_topic, rmw_qos_profile_sensor_data);
  right_image_sub_.subscribe(this, right_image_topic, rmw_qos_profile_sensor_data);
  sync_ = std::make_shared<Sync>(SyncPolicy(10), left_image_sub_, right_image_sub_);
  sync_->registerCallback(
    std::bind(&StereoNode::imageCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void StereoNode::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr left_img_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr right_img_msg)
{
  if (this->orb_slam_ != nullptr) {
    cv::Mat left_image = cv_bridge::toCvShare(left_img_msg)->image;
    cv::Mat right_image = cv_bridge::toCvShare(right_img_msg)->image;
    this->orb_slam_->TrackStereo(
      left_image, right_image, rclcpp::Time(left_img_msg->header.stamp).seconds());
    this->publishData();
  } else {
    RCLCPP_INFO(this->get_logger(), "ORB-SLAM2 system is not ready");
  }
}
}  // namespace ORB_SLAM2

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ORB_SLAM2::StereoNode)