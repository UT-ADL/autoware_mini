// Copyright (c) 2025 Autonomous Driving Lab, University of Tartu
// SPDX-License-Identifier: MIT

#include <vector>

#include <autoware_mini/rviz/trajectory_visual.h>
#include <autoware_mini/rviz/rviz_helpers.h>

namespace autoware_mini
{

TrajectoryVisual::TrajectoryVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, uint32_t id)
    : scene_manager_(scene_manager)
    , id_(id)
    , color_r_(1.0f)
    , color_g_(1.0f)
    , color_b_(1.0f)
    , color_a_(1.0f)
{
    // Create scene node as child of parent
    scene_node_ = parent_node->createChildSceneNode();

    // Create manual object for rendering geometry
    manual_object_ = createDynamicManualObject(scene_manager_, scene_node_, "PredictedTrajectoryVisual", id_);
}

TrajectoryVisual::~TrajectoryVisual()
{
    scene_manager_->destroyManualObject(manual_object_);
    scene_manager_->destroySceneNode(scene_node_);
}

void TrajectoryVisual::setTrajectories(const PathArray& paths, double width)
{
    manual_object_->clear();

    if (paths.paths.empty())
    {
        return;
    }

    std::vector<Ogre::Vector3> centerline;
    std::vector<Ogre::Vector3> strip;

    for (const auto& path : paths.paths)
    {
        if (path.waypoints.size() < 2)
        {
            continue;
        }

        centerline.clear();
        centerline.reserve(path.waypoints.size());

        for (const auto& wp : path.waypoints)
        {
            centerline.push_back(Ogre::Vector3(wp.position.x, wp.position.y, wp.position.z));
        }

        triangulateRibbon(centerline, width, strip);
    }

    renderTriangleStrip(manual_object_, strip, color_r_, color_g_, color_b_, color_a_);
}

void TrajectoryVisual::setColor(float r, float g, float b, float a)
{
    color_r_ = r;
    color_g_ = g;
    color_b_ = b;
    color_a_ = a;
}

void TrajectoryVisual::setVisible(bool visible)
{
    // Don't cascade visibility to children - they manage their own visibility
    scene_node_->setVisible(visible, false);
}

void TrajectoryVisual::setFramePosition(const Ogre::Vector3& position)
{
    scene_node_->setPosition(position);
}

void TrajectoryVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
    scene_node_->setOrientation(orientation);
}

}  // namespace autoware_mini
