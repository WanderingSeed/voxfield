#include "voxblox_rviz_plugin/voxblox_mesh_display.h"

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <voxblox_rviz_plugin/material_loader.h>

namespace voxblox_rviz_plugin {

VoxbloxMeshDisplay::VoxbloxMeshDisplay() {
  voxblox_rviz_plugin::MaterialLoader::loadMaterials();
}

void VoxbloxMeshDisplay::onInitialize() {
  MFDClass::onInitialize();

  visible_property_ = new rviz_common::properties::BoolProperty(
      "Visible", true,
      "Show or hide the mesh. If the mesh is hidden but not disabled, it "
      "will persist and is incrementally built in the background.",
      this, SLOT(visibleSLOT()));

  visual_.reset(
      new VoxbloxMeshVisual(context_, scene_node_));
  visual_->setEnabled(visible_property_->getBool());

  connect(context_->getFrameManager(), &rviz_common::FrameManagerIface::fixedFrameChanged,
          this, &VoxbloxMeshDisplay::onFixedFrameChanged);
}

void VoxbloxMeshDisplay::reset() {
  MFDClass::reset();
  if (visual_) {
    visual_->reset();
  }
}

void VoxbloxMeshDisplay::visibleSLOT() {
  if (visual_) {
    visual_->setEnabled(visible_property_->getBool());
    if (visible_property_->getBool()) {
      // Create an empty mesh message to trigger the transformation update
      auto msg = std::make_shared<voxblox_msgs::msg::Mesh>();
      msg->header.frame_id = fixed_frame_.toStdString();
      msg->header.stamp = context_->getClock()->now();
      updateTransformation(msg);
    }
  }
}

void VoxbloxMeshDisplay::processMessage(
    voxblox_msgs::msg::Mesh::ConstSharedPtr msg) {
  if (!visual_) {
    return;
  }

  if (updateTransformation(msg)) {
    visual_->setMessage(msg);
    setStatus(rviz_common::properties::StatusProperty::Ok, "Transform", "Ok");
  } else {
    std::string error_message = "Could not transform from [" +
                              msg->header.frame_id + "] to Fixed Frame [" +
                              fixed_frame_.toStdString() + "]";
    setStatus(rviz_common::properties::StatusProperty::Error, "Transform",
              QString::fromStdString(error_message));
  }
}

bool VoxbloxMeshDisplay::updateTransformation(voxblox_msgs::msg::Mesh::ConstSharedPtr msg) {
  if (!visual_) {
    return false;
  }
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp,
                                                position, orientation)) {
    RCLCPP_DEBUG(this->context_->getRosNodeAbstraction().lock()->get_raw_node()->get_logger(),
                 "Error transforming from frame '%s' to frame '%s'",
                 msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return false;
  }
  visual_->setFramePosition(position);
  visual_->setFrameOrientation(orientation);
  return true;
}

void VoxbloxMeshDisplay::onFixedFrameChanged() {
  // The transformation will be updated when the next message is processed.
}

}  // namespace voxblox_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(voxblox_rviz_plugin::VoxbloxMeshDisplay,
                       rviz_common::Display)
