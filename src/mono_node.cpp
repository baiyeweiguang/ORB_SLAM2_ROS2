#include "ros2/mono_node.hpp"

#include <cv_bridge/cv_bridge.h>
#include <memory>

namespace ORB_SLAM2
{
MonoNode::MonoNode(const rclcpp::NodeOptions & options)
: BaseNode(System::eSensor::MONOCULAR, options)
{
  RCLCPP_INFO(this->get_logger(), "Starting ORB-SLAM2 Monocular node");
  std::string camera_topic_name =
    this->declare_parameter("camera.image_topic", std::string("image_raw"));
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    camera_topic_name, rclcpp::SensorDataQoS(),
    std::bind(&MonoNode::imageCallback, this, std::placeholders::_1));
}

void MonoNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr image_msg)
{
  if (this->orb_slam_ != nullptr) {
    cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
    this->orb_slam_->TrackMonocular(image, rclcpp::Time(image_msg->header.stamp).seconds());
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
RCLCPP_COMPONENTS_REGISTER_NODE(ORB_SLAM2::MonoNode)