#ifndef VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_DISPLAY_H_
#define VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_DISPLAY_H_

#include <memory>

#include <voxblox_msgs/msg/mesh.hpp>

#include "voxblox_rviz_plugin/voxblox_mesh_visual.h"

#include <rviz_common/message_filter_display.hpp>
#include <rclcpp/time.hpp>

namespace rviz_common {
namespace properties {
class BoolProperty;
}
}  // namespace rviz_common

namespace voxblox_rviz_plugin {

class VoxbloxMeshVisual;

class VoxbloxMeshDisplay
    : public rviz_common::MessageFilterDisplay<voxblox_msgs::msg::Mesh> {
  Q_OBJECT
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VoxbloxMeshDisplay();
  ~VoxbloxMeshDisplay() override = default;

 protected:
  void onInitialize() override;
  void reset() override;

 private Q_SLOTS:
  void visibleSLOT();
  void onFixedFrameChanged();

 private:
  void processMessage(voxblox_msgs::msg::Mesh::ConstSharedPtr msg) override;
  virtual bool updateTransformation(voxblox_msgs::msg::Mesh::ConstSharedPtr msg);

  std::unique_ptr<VoxbloxMeshVisual> visual_;

  rviz_common::properties::BoolProperty* visible_property_;
};

}  // namespace voxblox_rviz_plugin

#endif  // VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_DISPLAY_H_
