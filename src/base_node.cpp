#include "ros2/base_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/message_traits.h>
#include <sensor_msgs/PointField.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

namespace ORB_SLAM2 {
BaseNode::BaseNode(System::eSensor sensor, const rclcpp::NodeOptions &options)
    : Node("orb_slam2", options), sensor_(sensor),
      min_observations_per_point_(2), orb_slam_(nullptr) {
  // Declare parameters
  is_publish_pointcloud_ = this->declare_parameter("publish_pointcloud", true);
  is_publish_pose_ = this->declare_parameter("publish_pose", true);
  is_publish_tf_ = this->declare_parameter("publish_tf", true);
  use_ros_coordinate_ = this->declare_parameter("use_ros_cordinate", true);
  world_frame_id_ =
      this->declare_parameter("world_frame_id_", std::string("map"));
  camera_frame_id_ =
      this->declare_parameter("camera_frame_id", std::string("camera_link"));

  // Create ORB-SLAM2 System unitil get camera info
  std::string camera_info_topic = this->declare_parameter(
      "camera.camera_info_topic", std::string("camera_info"));
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
        RCLCPP_INFO(this->get_logger(), "Get camera info, create slam system");
        orb_slam_ = createSystem(camera_info);
        camera_info_sub_.reset();
        RCLCPP_INFO(this->get_logger(), "ORB-SLAM2 is ready");
      });

  // Create tf buffer
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  // Create publishers
  if (is_publish_pointcloud_) {
    pointcloud_publisher_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "pointcloud", rclcpp::SensorDataQoS());
  }
  if (is_publish_pose_) {
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "pose", rclcpp::SensorDataQoS());
  }
  if (is_publish_tf_) {
    tf2_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }
  image_pub_ = image_transport::create_publisher(this, "debug_image");
}

std::unique_ptr<System> BaseNode::createSystem(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
  ORBParameters parameters;
  parameters.maxFrames = this->declare_parameter("camera.fps", 30);
  parameters.RGB = this->declare_parameter("camera.rgb_encoding", true);
  parameters.nFeatures =
      this->declare_parameter("ORBextractor.nFeatures", 1200);
  parameters.scaleFactor = this->declare_parameter("ORBextractor.scaleFactor",
                                                   static_cast<float>(1.2));
  parameters.nLevels = this->declare_parameter("ORBextractor.nLevels", 8);
  parameters.iniThFAST = this->declare_parameter("ORBextractor.iniThFAST", 20);
  parameters.minThFAST = this->declare_parameter("ORBextractor.minThFAST", 7);
  parameters.fx = camera_info->k[0];
  parameters.fy = camera_info->k[4];
  parameters.cx = camera_info->k[2];
  parameters.cy = camera_info->k[5];
  parameters.k1 = camera_info->d[0];
  parameters.k2 = camera_info->d[1];
  parameters.p1 = camera_info->d[2];
  parameters.p2 = camera_info->d[3];
  parameters.k3 = camera_info->d[4];
  parameters.baseline = this->declare_parameter("camera.baseline", 0.05);

  std::string voc_file_name =
      this->declare_parameter("vocabulary_file", std::string(""));
  std::string map_file_name =
      this->declare_parameter("map_file", std::string(""));
  std::string pkg_path =
      ament_index_cpp::get_package_share_directory("orb_slam2_ros2");
  std::string voc_path = pkg_path + "/" + voc_file_name;

  bool load_from_map = this->declare_parameter("load_map", false);
  return std::make_unique<System>(voc_path, sensor_, parameters, map_file_name,
                                  load_from_map);
}

void BaseNode::publishData() {
  current_frame_time_ = this->now();
  publishDebugImage();

  auto position = orb_slam_->GetCurrentPosition();
  if (!position.empty()) {
    current_transform_ = getTransformFromMat(position);
    RCLCPP_INFO(this->get_logger(), "Get pose, transform:[%f, %f, %f]",
                current_transform_.getOrigin().x(),
                current_transform_.getOrigin().y(),
                current_transform_.getOrigin().z());
  } else {
    RCLCPP_WARN(this->get_logger(), "No Position get");
  }

  if (is_publish_pose_) {
    publishPosition();
  }
  if (is_publish_tf_) {
    publishTF();
  }
  if (is_publish_pointcloud_) {
    auto map_points = orb_slam_->GetAllMapPoints();
    if (!map_points.empty()) {
      publishMapPoints(std::move(map_points));
    } else {
      RCLCPP_WARN(this->get_logger(), "No Map Points get");
    }
  }
}

tf2::Transform
BaseNode::getTransformFromMat(const cv::Mat &cv_transform) const {
  tf2::Matrix3x3 rotation(
      cv_transform.at<float>(0, 0), cv_transform.at<float>(0, 1),
      cv_transform.at<float>(0, 2), cv_transform.at<float>(1, 0),
      cv_transform.at<float>(1, 1), cv_transform.at<float>(1, 2),
      cv_transform.at<float>(2, 0), cv_transform.at<float>(2, 1),
      cv_transform.at<float>(2, 2));
  tf2::Vector3 translation(cv_transform.at<float>(0, 3),
                           cv_transform.at<float>(1, 3),
                           cv_transform.at<float>(2, 3));
  if (use_ros_coordinate_) {
    // clang-format off
    const tf2::Matrix3x3 tf_orb_to_ros
                            (0, 0, 1, 
                            -1, 0, 0, 
                            0, -1, 0);
    // clang-format on
    // Transform from orb coordinate system to ros coordinate system on camera
    // coordinates
    rotation = tf_orb_to_ros * rotation;
    translation = tf_orb_to_ros * translation;

    // Inverse matrix
    rotation = rotation.transpose();
    translation = -(rotation * translation);

    // Transform from orb coordinate system to ros coordinate system on map
    // coordinates
    rotation = tf_orb_to_ros * rotation;
    translation = tf_orb_to_ros * translation;
  }
  return tf2::Transform(rotation, translation);
}

void BaseNode::publishPosition() const {
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = world_frame_id_;
  msg.header.stamp = current_frame_time_;
  auto t = tf2::toMsg(current_transform_);
  msg.pose.position.x = t.translation.x;
  msg.pose.position.y = t.translation.y;
  msg.pose.position.z = t.translation.z;
  msg.pose.orientation = t.rotation;
  pose_publisher_->publish(msg);
}

void BaseNode::publishTF() const {
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = world_frame_id_;
  t.child_frame_id = camera_frame_id_;
  t.header.stamp = current_frame_time_;
  t.transform = tf2::toMsg(current_transform_);
  tf2_broadcaster_->sendTransform(t);
}

void BaseNode::publishMapPoints(
    const std::vector<MapPoint *> &map_points) const {
  // x y z
  const int num_channels = 3;

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = world_frame_id_;
  cloud.header.stamp = current_frame_time_;
  cloud.height = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.width = map_points.size();
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::array<std::string, 3> channel_id{"x", "y", "Z"};

  for (size_t i = 0; i < num_channels; ++i) {
    cloud.fields[i].name = channel_id[i];
    cloud.fields[i].offset = i * sizeof(float);
    cloud.fields[i].count = 1;
    cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);

  unsigned char *cloud_data_ptr = &(cloud.data[0]);
  for (size_t i = 0; i < cloud.width; ++i) {
    if (map_points.at(i)->nObs >= min_observations_per_point_) {
      auto p = map_points.at(i)->GetWorldPos();
      if (use_ros_coordinate_) {
        auto tmp_p = p.clone();
        p.at<float>(0) = tmp_p.at<float>(2);
        p.at<float>(1) = -tmp_p.at<float>(0);
        p.at<float>(2) = -tmp_p.at<float>(1);
      }
      memcpy(cloud_data_ptr + (i * cloud.point_step), p.data,
             num_channels * sizeof(float));
    }
  }
  pointcloud_publisher_->publish(cloud);
}

void BaseNode::publishDebugImage() const {
  cv::Mat debuge_image = orb_slam_->DrawCurrentFrame();
  std_msgs::msg::Header header;
  header.frame_id = camera_frame_id_;
  header.stamp = current_frame_time_;
  image_pub_.publish(
      cv_bridge::CvImage(header, "rgb8", debuge_image).toImageMsg());
}

} // namespace ORB_SLAM2
