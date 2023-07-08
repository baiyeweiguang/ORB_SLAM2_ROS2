#ifndef ORB_SLAM_2_ROS2_NODE_HPP_
#define ORB_SLAM_2_ROS2_NODE_HPP_

// std
#include <memory>
#include <string>
// ros2
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "System.h"

namespace ORB_SLAM2
{

class BaseNode : public rclcpp::Node
{
public:
  explicit BaseNode(System::eSensor sensor, const rclcpp::NodeOptions & options);
  virtual ~BaseNode() = default;

protected:
  void publishData();

  // orb_slam2 system
  std::unique_ptr<System> orb_slam_;

  tf2::Transform current_transform_;
  rclcpp::Time current_frame_time_;

private:
  std::unique_ptr<System> createSystem(const sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info);

  void publishMapPoints(const std::vector<MapPoint *> & map_points) const;

  void publishPosition() const;

  void publishTF() const;

  void publishDebugImage() const;

  tf2::Transform getTransformFromMat(const cv::Mat & cv_transform) const;

  ORBParameters getORBParameters();

  // tf2
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;

  // publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  image_transport::Publisher image_pub_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  bool is_publish_pointcloud_;
  bool is_publish_pose_;
  bool is_publish_tf_;
  bool use_ros_coordinate_;
  std::string world_frame_id_;
  std::string camera_frame_id_;
  System::eSensor sensor_;
  int min_observations_per_point_;
};
}  // namespace ORB_SLAM2

#endif  // ORB_SLAM_2_ROS2_NODE_HPP_