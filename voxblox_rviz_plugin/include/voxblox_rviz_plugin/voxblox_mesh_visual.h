#ifndef VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_VISUAL_H_
#define VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_VISUAL_H_

#include <unordered_map>

#include <voxblox/core/block.h>
#include <string>

#include <OgreVector.h>
#include <OgreQuaternion.h>

#include <voxblox/core/block_hash.h>
#include "voxblox_msgs/msg/mesh.hpp"

namespace Ogre
{
class ManualObject;
class SceneManager;
class SceneNode;
}

namespace rviz_common
{
class DisplayContext;
}

namespace voxblox_rviz_plugin
{

typedef std::unordered_map<voxblox::BlockIndex, Ogre::ManualObject*,
                           voxblox::AnyIndexHash>
    BlockIndexObjectMap;

// Visualizes a single voxblox_msgs::Mesh message.
class VoxbloxMeshVisual
{
public:
  VoxbloxMeshVisual(rviz_common::DisplayContext * context, Ogre::SceneNode * parent_node);
  virtual ~VoxbloxMeshVisual();

  void setMessage(voxblox_msgs::msg::Mesh::ConstSharedPtr msg);

  // Set the coordinate frame pose.
  void setFramePosition(const Ogre::Vector3 & position);
  void setFrameOrientation(const Ogre::Quaternion & orientation);

  void setEnabled(bool enabled);
  void reset();

  void setVisible(bool visible);

private:
  // The object implementing the actual visualization.
  voxblox::AnyIndexHashMapType<Ogre::ManualObject*>::type object_map_;


  // A SceneNode whose pose is set to match the coordinate frame of
  // the Mesh message header.
  Ogre::SceneNode * frame_node_;

  // The SceneManager, kept here only so the destructor can ask
  // it to destroy the ManualObject it created.
  Ogre::SceneManager * scene_manager_;

  // Used for identifying the object created by this visual
  unsigned int instance_number_;
  static unsigned int instance_counter_;

  bool is_enabled_;
};

}  // namespace voxblox_rviz_plugin

#endif  // VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_VISUAL_H_
