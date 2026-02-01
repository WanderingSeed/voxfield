#include "voxblox_ros/transformer.h"
#include "voxblox_ros/ros_params.h"

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
    : Transformer(node, getTransformerConfigFromRosParam(node)) {}

Transformer::Transformer(const rclcpp::Node::SharedPtr& node, const Config& config)
    : node_(node) {
  initializeFromConfig(config);
}

void Transformer::initializeFromConfig(const Config& config) {
  world_frame_ = config.world_frame;
  sensor_frame_ = config.sensor_frame;
  use_tf_transforms_ = config.use_tf_transforms;
  timestamp_tolerance_ns_ = static_cast<int64_t>(config.timestamp_tolerance_sec * 1.0e9);

  // TF2 setup
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if (!use_tf_transforms_) {
    transform_sub_ = node_->create_subscription<geometry_msgs::msg::TransformStamped>(
        "transform", 40, std::bind(&Transformer::transformCallback, this, std::placeholders::_1));

    // Process T_B_D transform matrix
    if (!config.T_B_D_vector.empty()) {
      if (config.T_B_D_vector.size() != 16) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Parameter T_B_D must be a 4x4 matrix (16 doubles), but has %zu values.",
                     config.T_B_D_vector.size());
      } else {
        Eigen::Matrix4f T_B_D_matrix;
        for (int i = 0; i < 4; ++i) {
          for (int j = 0; j < 4; ++j) {
            T_B_D_matrix(i, j) = static_cast<float>(config.T_B_D_vector[i * 4 + j]);
          }
        }
        T_B_D_ = Transformation(T_B_D_matrix);
        if (config.invert_T_B_D) {
          T_B_D_ = T_B_D_.inverse();
        }
      }
    }

    // Process T_B_C transform matrix
    if (!config.T_B_C_vector.empty()) {
      if (config.T_B_C_vector.size() != 16) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Parameter T_B_C must be a 4x4 matrix (16 doubles), but has %zu values.",
                     config.T_B_C_vector.size());
      } else {
        Eigen::Matrix4f T_B_C_matrix;
        for (int i = 0; i < 4; ++i) {
          for (int j = 0; j < 4; ++j) {
            T_B_C_matrix(i, j) = static_cast<float>(config.T_B_C_vector[i * 4 + j]);
          }
        }
        T_B_C_ = Transformation(T_B_C_matrix);
        if (config.invert_T_B_C) {
          T_B_C_ = T_B_C_.inverse();
        }
      }
    }
  }
  
  T_D_C_ = T_B_D_.inverse() * T_B_C_;

  // Process T_C_CH transform matrix (model transformation)
  if (!config.T_C_CH_vector.empty()) {
    if (config.T_C_CH_vector.size() != 16) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Parameter T_C_CH must be a 4x4 matrix (16 doubles), but has %zu values.",
                   config.T_C_CH_vector.size());
    } else {
      Eigen::Matrix4f T_C_CH_matrix;
      for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
          T_C_CH_matrix(i, j) = static_cast<float>(config.T_C_CH_vector[i * 4 + j]);
        }
      }
      T_C_CH_ = Transformation(T_C_CH_matrix);
      if (config.invert_T_C_CH) {
        T_C_CH_ = T_C_CH_.inverse();
      }
    }
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
