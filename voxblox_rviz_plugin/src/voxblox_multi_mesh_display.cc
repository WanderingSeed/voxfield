#include "voxblox_rviz_plugin/voxblox_multi_mesh_display.h"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/validate_floats.hpp"

#include "voxblox_rviz_plugin/voxblox_mesh_visual.h"

namespace voxblox_rviz_plugin {

VoxbloxMultiMeshDisplay::VoxbloxMultiMeshDisplay()
    : rviz_common::MessageFilterDisplay<voxblox_msgs::msg::MultiMesh>() {}

VoxbloxMultiMeshDisplay::~VoxbloxMultiMeshDisplay() = default;

void VoxbloxMultiMeshDisplay::onInitialize() {
  MFDClass::onInitialize();

  submap_visibility_property_ = new rviz_common::properties::Property(
      "Submap Visibility", QVariant(),
      "Allows showing and hiding of individual submaps.", this);

  all_submaps_visible_property_ = new rviz_common::properties::BoolProperty(
      "Show All", true,
      "Turn all submaps on or off.",
      this, SLOT(allSubmapsVisibleChanged()));
}


void VoxbloxMultiMeshDisplay::reset() {
  MFDClass::reset();
  visuals_.clear();
  visibility_fields_.clear();
  delete submap_visibility_property_;
  // Re-create the property.
  submap_visibility_property_ = new rviz_common::properties::Property(
      "Submap Visibility", QVariant(),
      "Allows showing and hiding of individual submaps.", this);
}

bool validateFloats(const voxblox_msgs::msg::Mesh& msg) {
  return true;
}

void VoxbloxMultiMeshDisplay::processMessage(
    voxblox_msgs::msg::MultiMesh::ConstSharedPtr msg) {
  // The MultiMesh message in ROS2 does not have a clear field.
  // The rviz plugin is not responsible for clearing the visuals anymore.
  // The user should stop and restart rviz to clear the visuals.

  // Here we laid out the visuals in a grid.
  const auto& mesh = msg->mesh;

  if (!validateFloats(mesh)) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
              "Message contained invalid floating point values (nans or infs)");
    return;
  }

  // Get the submap namespace
  const std::string& submap_namespace = msg->name_space;

  // Check if we have a visual for this id
  auto it = this->visuals_.find(submap_namespace);
  if (it == this->visuals_.end()) {
    // If we don't have a visual for this submap, create one
    auto visual = std::make_unique<VoxbloxMeshVisual>(
        context_, scene_node_);
    // Set the message
    auto mesh_msg = std::make_shared<voxblox_msgs::msg::Mesh>(mesh);
    mesh_msg->header = msg->header;
    visual->setMessage(mesh_msg);

    // Set the pose of the visual
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if (!context_->getFrameManager()->getTransform(mesh_msg->header, position,
                                                  orientation)) {
      RCLCPP_DEBUG(
          this->context_->getRosNodeAbstraction().lock()->get_raw_node()->get_logger(),
          "Error transforming from frame '%s' to frame '%s'",
          mesh_msg->header.frame_id.c_str(), qPrintable(this->fixed_frame_));
      return;
    }
    setTransformOk();
    visual->setFramePosition(position);
    visual->setFrameOrientation(orientation);

    // Add the visual to the map
    this->visuals_[submap_namespace] = std::move(visual);

    // Create a visibility property for this submap
    createVisibilityProperty(submap_namespace);
  } else {
    // If we have a visual, update the message
    auto mesh_msg = std::make_shared<voxblox_msgs::msg::Mesh>(mesh);
    mesh_msg->header = msg->header;
    it->second->setMessage(mesh_msg);

    // Update the pose of the visual
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if (!context_->getFrameManager()->getTransform(mesh_msg->header, position,
                                                  orientation)) {
      setMissingTransformToFixedFrame(mesh_msg->header.frame_id);
      return;
    }
    setTransformOk();
    it->second->setFramePosition(position);
    it->second->setFrameOrientation(orientation);
  }
}

void VoxbloxMultiMeshDisplay::createVisibilityProperty(
    const std::string& submap_namespace) {
  // Check if we have a visibility property for this id
  auto it = this->visibility_fields_.find(submap_namespace);
  if (it == this->visibility_fields_.end()) {
    // If we don't have a visibility property, create one
    auto visibility_property = new rviz_common::properties::BoolProperty(
        submap_namespace.c_str(), this->all_submaps_visible_property_->getBool(),
        "Show or hide this submap.", this->submap_visibility_property_,
        SLOT(visibilityChanged()), this);
    this->visibility_fields_[submap_namespace] = visibility_property;
  }
}

void VoxbloxMultiMeshDisplay::visibilityChanged() {
  // Set the visibility of the submaps
  for (auto const& [submap_namespace, visual] : this->visuals_) {
    // Check if we have a visibility property for this id
    auto it = this->visibility_fields_.find(submap_namespace);
    if (it != this->visibility_fields_.end()) {
      visual->setEnabled(it->second->getBool());
    }
  }
}

void VoxbloxMultiMeshDisplay::allSubmapsVisibleChanged() {
  // Set the visibility of all submaps
  for (auto const& [submap_id, visibility_field] : this->visibility_fields_) {
    visibility_field->setBool(this->all_submaps_visible_property_->getBool());
  }
}

}  // namespace voxblox_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(voxblox_rviz_plugin::VoxbloxMultiMeshDisplay,
                       rviz_common::Display)