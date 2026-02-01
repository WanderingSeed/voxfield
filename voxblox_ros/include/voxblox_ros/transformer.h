#ifndef VOXBLOX_ROS_TRANSFORMER_H_
#define VOXBLOX_ROS_TRANSFORMER_H_

#include <string>
#include <vector>

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

  /// Contains all the information needed to setup the Transformer class.
  struct Config {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::string world_frame = "world";
    std::string sensor_frame = "";
    bool use_tf_transforms = true;
    double timestamp_tolerance_sec = 0.001;  // 1ms default
    
    // Transform matrices (16 elements each for 4x4 matrix)
    std::vector<double> T_B_D_vector;  // Transform from base_link to depth camera
    std::vector<double> T_B_C_vector;  // Transform from base_link to color camera
    std::vector<double> T_C_CH_vector; // Transform from color camera to checkerboard
    
    bool invert_T_B_D = false;
    bool invert_T_B_C = false;
    bool invert_T_C_CH = false;
  };

  explicit Transformer(const rclcpp::Node::SharedPtr& node);
  Transformer(const rclcpp::Node::SharedPtr& node, const Config& config);

  bool lookupTransform(
      const std::string& from_frame, const std::string& to_frame,
      const rclcpp::Time& timestamp, Transformation* transform);

  void transformCallback(const geometry_msgs::msg::TransformStamped& transform_msg);

  Transformation getStaticTransform();

  Transformation getModelTransform();

 private:
  void initializeFromConfig(const Config& config);
  
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
