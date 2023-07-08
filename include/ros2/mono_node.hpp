#ifndef ORB_SLAM2_ROS2_MONO_NODE_HPP_
#define ORB_SLAM2_ROS2_MONO_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "System.h"
#include "ros2/base_node.hpp"

namespace ORB_SLAM2
{
class MonoNode : public BaseNode
{
public:
  explicit MonoNode(const rclcpp::NodeOptions & options);

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr image_msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};
}  // namespace ORB_SLAM2
#endif  // ORB_SLAM2_ROS2_MONO_NODE_HPP_