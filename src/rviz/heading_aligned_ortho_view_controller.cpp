// Copyright (c) 2026 Autonomous Driving Lab, University of Tartu
// SPDX-License-Identifier: MIT

#include "autoware_mini/rviz/heading_aligned_ortho_view_controller.h"

#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>

#include <rviz/display_context.h>
#include <pluginlib/class_list_macros.h>

namespace autoware_mini
{

void HeadingAlignedOrthoViewController::updateTargetSceneNode()
{
  if (!getNewTransform()) {
    return;
  }
  target_scene_node_->setPosition(reference_position_);

  // Extract yaw from the target frame's orientation and apply it to the
  // target scene node so the camera (a child of this node) rotates with
  // the vehicle's heading. Same getRoll(false) pattern that
  // ThirdPersonFollowerViewController uses internally.
  Ogre::Radian yaw = reference_orientation_.getRoll(false);
  Ogre::Quaternion yaw_quat(yaw, Ogre::Vector3::UNIT_Z);
  target_scene_node_->setOrientation(yaw_quat);

  context_->queueRender();
}

} // namespace autoware_mini

PLUGINLIB_EXPORT_CLASS(autoware_mini::HeadingAlignedOrthoViewController, rviz::ViewController)
