#include "voxblox_ros/transformer.h"

#include <cinttypes>
#include <vector>

#include <Eigen/Core>
#include <tf2_eigen/tf2_eigen.hpp>

// Local helper to convert a ROS2 Transform message into a voxblox Transformation
static void transformMsgToTransformation(const geometry_msgs::msg::Transform& t_msg,
                                         voxblox::Transformation* T) {
  Eigen::Quaterniond rotation(t_msg.rotation.w, t_msg.rotation.x,
                              t_msg.rotation.y, t_msg.rotation.z);
  Eigen::Vector3d translation(t_msg.translation.x, t_msg.translation.y,
                              t_msg.translation.z);
  *T = voxblox::Transformation(rotation.cast<voxblox::FloatingPoint>(),
                               translation.cast<voxblox::FloatingPoint>());
}

namespace voxblox {

Transformer::Transformer(const rclcpp::Node::SharedPtr& node)
    : node_(node), world_frame_("world"), sensor_frame_(""), use_tf_transforms_(true), timestamp_tolerance_ns_(1000000) {
  // Read parameters (declare defaults where appropriate)
  node_->declare_parameter<std::string>("world_frame", world_frame_);
  node_->get_parameter("world_frame", world_frame_);
  node_->declare_parameter<std::string>("sensor_frame", sensor_frame_);
  node_->get_parameter("sensor_frame", sensor_frame_);

  double timestamp_tolerance_sec = static_cast<double>(timestamp_tolerance_ns_) / 1.0e9;
  node_->declare_parameter<double>("timestamp_tolerance_sec", timestamp_tolerance_sec);
  node_->get_parameter("timestamp_tolerance_sec", timestamp_tolerance_sec);
  timestamp_tolerance_ns_ = static_cast<int64_t>(timestamp_tolerance_sec * 1.0e9);

  node_->declare_parameter<bool>("use_tf_transforms", use_tf_transforms_);
  node_->get_parameter("use_tf_transforms", use_tf_transforms_);

  // TF2 setup
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if (!use_tf_transforms_) {
    transform_sub_ = node_->create_subscription<geometry_msgs::msg::TransformStamped>(
        "transform", 40, std::bind(&Transformer::transformCallback, this, std::placeholders::_1));

    // Retrieve T_D_C from params.
    // Transform from base_link to depth camera.
    std::vector<double> T_B_D_vector;
    node_->declare_parameter("T_B_D", rclcpp::ParameterValue(T_B_D_vector));
    if (node_->get_parameter("T_B_D", T_B_D_vector) && !T_B_D_vector.empty()) {
      if (T_B_D_vector.size() != 16) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Parameter T_B_D must be a 4x4 matrix (16 doubles), but has %zu values.",
                     T_B_D_vector.size());
      } else {
        Eigen::Matrix4f T_B_D_matrix;
      for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
          T_B_D_matrix(i, j) = static_cast<float>(T_B_D_vector[i * 4 + j]);
        }
      }
      T_B_D_ = Transformation(T_B_D_matrix);
      }
    }

    bool invert_T_B_D = false;
    node_->declare_parameter<bool>("invert_T_B_D", invert_T_B_D);
    node_->get_parameter("invert_T_B_D", invert_T_B_D);
    if (invert_T_B_D) {
      T_B_D_ = T_B_D_.inverse();
    }

    // Transform from base_link to color camera.
    std::vector<double> T_B_C_vector;
    node_->declare_parameter("T_B_C", rclcpp::ParameterValue(T_B_C_vector));
    if (node_->get_parameter("T_B_C", T_B_C_vector) && !T_B_C_vector.empty()) {
      if (T_B_C_vector.size() != 16) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Parameter T_B_C must be a 4x4 matrix (16 doubles), but has %zu values.",
                     T_B_C_vector.size());
      } else {
        Eigen::Matrix4f T_B_C_matrix;
      for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
          T_B_C_matrix(i, j) = static_cast<float>(T_B_C_vector[i * 4 + j]);
        }
      }
      T_B_C_ = Transformation(T_B_C_matrix);
      }
    }

    bool invert_T_B_C = false;
    node_->declare_parameter<bool>("invert_T_B_C", invert_T_B_C);
    node_->get_parameter("invert_T_B_C", invert_T_B_C);
    if (invert_T_B_C) {
      T_B_C_ = T_B_C_.inverse();
    }
  }
  T_D_C_ = T_B_D_.inverse() * T_B_C_;

  // Model transformation
  // Transform from color camera to checkerboard, if such exists.
  std::vector<double> T_C_CH_vector;
  node_->declare_parameter("T_C_CH", rclcpp::ParameterValue(T_C_CH_vector));
  if (node_->get_parameter("T_C_CH", T_C_CH_vector) && !T_C_CH_vector.empty()) {
    if (T_C_CH_vector.size() != 16) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Parameter T_C_CH must be a 4x4 matrix (16 doubles), but has %zu values.",
                   T_C_CH_vector.size());
    } else {
      Eigen::Matrix4f T_C_CH_matrix;
      for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
          T_C_CH_matrix(i, j) = static_cast<float>(T_C_CH_vector[i * 4 + j]);
        }
      }
      T_C_CH_ = Transformation(T_C_CH_matrix);
    }
  }

  bool invert_T_C_CH = false;
  node_->declare_parameter<bool>("invert_T_C_CH", invert_T_C_CH);
  node_->get_parameter("invert_T_C_CH", invert_T_C_CH);
  if (invert_T_C_CH) {
    T_C_CH_ = T_C_CH_.inverse();
  }
}

void Transformer::transformCallback(const geometry_msgs::msg::TransformStamped& transform_msg) {
  transform_queue_.push_back(transform_msg);
}

Transformation Transformer::getStaticTransform() { return T_B_C_; }

Transformation Transformer::getModelTransform() { return T_C_CH_; }

bool Transformer::lookupTransform(const std::string& from_frame, const std::string& to_frame,
                                  const rclcpp::Time& timestamp, Transformation* transform) {
  CHECK_NOTNULL(transform);
  if (use_tf_transforms_) {
    return lookupTransformTf(from_frame, to_frame, timestamp, transform);
  } else {
    return lookupTransformQueue(timestamp, transform);
  }
}

bool Transformer::lookupTransformTf(const std::string& from_frame, const std::string& to_frame,
                                    const rclcpp::Time& timestamp, Transformation* transform) {
  CHECK_NOTNULL(transform);
  // Prefer TF2 buffer lookup; convert rclcpp::Time to builtin_interfaces::msg::Time
  geometry_msgs::msg::TransformStamped tf_transform;
  std::string from_frame_modified = from_frame;
  if (!sensor_frame_.empty()) {
    from_frame_modified = sensor_frame_;
  }
  try {
    tf_transform = tf_buffer_->lookupTransform(to_frame, from_frame_modified, timestamp);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(node_->get_logger(), "Error getting TF transform: %s", ex.what());
    return false;
  }
  // Convert msg to voxblox Transformation
  transformMsgToTransformation(tf_transform.transform, transform);
  return true;
}

bool Transformer::lookupTransformQueue(const rclcpp::Time& timestamp, Transformation* transform) {
  CHECK_NOTNULL(transform);
  if (transform_queue_.empty()) {
    RCLCPP_WARN(node_->get_logger(), "No match found for transform timestamp: %" PRIu64 " as transform queue is empty.", timestamp.nanoseconds());
    return false;
  }

  bool match_found = false;
  auto it = transform_queue_.begin();
  for (; it != transform_queue_.end(); ++it) {
    rclcpp::Time msg_time(it->header.stamp);
    if (msg_time > timestamp) {
      if ((msg_time - timestamp).nanoseconds() < timestamp_tolerance_ns_) {
        match_found = true;
      }
      break;
    }
    if ((timestamp - msg_time).nanoseconds() < timestamp_tolerance_ns_) {
      match_found = true;
      break;
    }
  }

  Transformation T_G_D;
  if (match_found) {
    transformMsgToTransformation(it->transform, &T_G_D);
  } else {
    if (it == transform_queue_.begin() || it == transform_queue_.end()) {
      RCLCPP_WARN(node_->get_logger(), "No match found for transform timestamp (queue bounds)");
      return false;
    }
    Transformation T_G_D_newest; transformMsgToTransformation(it->transform, &T_G_D_newest);
    auto offset_newest_ns = (rclcpp::Time(it->header.stamp) - timestamp).nanoseconds();
    it--;
    Transformation T_G_D_oldest; transformMsgToTransformation(it->transform, &T_G_D_oldest);
    auto offset_oldest_ns = (timestamp - rclcpp::Time(it->header.stamp)).nanoseconds();

    FloatingPoint t_diff_ratio = static_cast<FloatingPoint>(offset_oldest_ns) /
        static_cast<FloatingPoint>(offset_newest_ns + offset_oldest_ns);
    Transformation::Vector6 diff_vector = (T_G_D_oldest.inverse() * T_G_D_newest).log();
    T_G_D = T_G_D_oldest * Transformation::exp(t_diff_ratio * diff_vector);
  }

  *transform = T_G_D * T_D_C_;
  transform_queue_.erase(transform_queue_.begin(), it);
  return true;
}

}  // namespace voxblox
