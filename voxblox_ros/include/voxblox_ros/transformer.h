#ifndef VOXBLOX_ROS_TRANSFORMER_H_
#define VOXBLOX_ROS_TRANSFORMER_H_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <voxblox/core/common.h>

namespace voxblox {

/**
 * Transformer adapted for ROS2: reads transforms from TF2 or from a transform
 * topic. Parameters should be declared on the ROS2 node and passed into the
 * constructor where needed.
 */
class Transformer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit Transformer(const rclcpp::Node::SharedPtr& node);

  bool lookupTransform(
      const std::string& from_frame, const std::string& to_frame,
      const rclcpp::Time& timestamp, Transformation* transform);

  void transformCallback(const geometry_msgs::msg::TransformStamped& transform_msg);

  Transformation getStaticTransform();

  Transformation getModelTransform();

 private:
  bool lookupTransformTf(
      const std::string& from_frame, const std::string& to_frame,
      const rclcpp::Time& timestamp, Transformation* transform);

  bool lookupTransformQueue(
      const rclcpp::Time& timestamp, Transformation* transform);

  rclcpp::Node::SharedPtr node_;

  std::string world_frame_;
  std::string sensor_frame_;
  bool use_tf_transforms_;
  int64_t timestamp_tolerance_ns_;

  Transformation T_B_C_;
  Transformation T_B_D_;
  Transformation T_D_C_;
  Transformation T_C_CH_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr transform_sub_;

  AlignedDeque<geometry_msgs::msg::TransformStamped> transform_queue_;
};

}  // namespace voxblox

#endif  // VOXBLOX_ROS_TRANSFORMER_H_
