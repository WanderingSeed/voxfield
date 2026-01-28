#ifndef VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MULTI_MESH_DISPLAY_H_
#define VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MULTI_MESH_DISPLAY_H_

#include <map>
#include <memory>
#include <string>
#include <unordered_map>

#include <voxblox_msgs/msg/multi_mesh.hpp>

#include "voxblox_rviz_plugin/voxblox_mesh_visual.h"

#include <rviz_common/message_filter_display.hpp>
#include <rclcpp/time.hpp>
#include <rviz_common/properties/bool_property.hpp>

namespace voxblox_rviz_plugin {

class VoxbloxMeshVisual;
class VisibilityField;

class VoxbloxMultiMeshDisplay
    : public rviz_common::MessageFilterDisplay<voxblox_msgs::msg::MultiMesh> {
  Q_OBJECT

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VoxbloxMultiMeshDisplay();
  ~VoxbloxMultiMeshDisplay() override = default;

 protected:
  void onInitialize() override;
  void reset() override;

 private Q_SLOTS:
  void visibilityChanged();
  void allSubmapsVisibleChanged();

 private:
  void processMessage(voxblox_msgs::msg::MultiMesh::ConstSharedPtr msg) override;
  void createVisibilityProperty(const std::string& submap_namespace);

  // Map of all submap visuals, key is the submap namespace.
  std::map<std::string, std::unique_ptr<VoxbloxMeshVisual>> visuals_;

  // The root of the visibility tree.
  rviz_common::properties::Property* submap_visibility_property_;
  rviz_common::properties::BoolProperty* all_submaps_visible_property_;

  // Map of all visibility properties, identified by submap namespace.
  // Map of all visibility properties, identified by submap namespace.
  std::map<std::string, rviz_common::properties::BoolProperty*> visibility_fields_;
};

// Allow the user to show hide sets of submaps based on the name spaces.
class VisibilityField : public rviz_common::properties::BoolProperty {
  Q_OBJECT
 public:
  VisibilityField(
      const std::string& name, rviz_common::properties::BoolProperty* parent,
      VoxbloxMultiMeshDisplay* master);
  void addField(const std::string& field_name);
  void removeField(const std::string& field_name);
  bool isEnabled(const std::string& field_name);
  void setEnabledForAll(bool enabled);

 private Q_SLOTS:
  void visibleSlot();

 private:
  VoxbloxMultiMeshDisplay* master_;
  std::unordered_map<std::string, std::unique_ptr<VisibilityField>> children_;
  bool hasNameSpace(
      const std::string& name, std::string* ns, std::string* sub_name);
};

}  // namespace voxblox_rviz_plugin

#endif  // VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MULTI_MESH_DISPLAY_H_
