// Copyright (c) 2025 Autonomous Driving Lab, University of Tartu
// SPDX-License-Identifier: MIT

#ifndef AUTOWARE_MINI_RVIZ_TRAJECTORY_VISUAL_H
#define AUTOWARE_MINI_RVIZ_TRAJECTORY_VISUAL_H

#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <autoware_mini/rviz/rviz_helpers.h>
#include <autoware_mini/PathArray.h>

namespace autoware_mini
{

/**
 * @brief Visual representation of predicted trajectories for a single object
 *
 * Uses OGRE ManualObject to render triangulated trajectory ribbons
 * directly, without going through the ROS marker system.
 */
class TrajectoryVisual
{
public:
    /**
     * @brief Constructor
     *
     * @param scene_manager OGRE scene manager
     * @param parent_node Parent scene node to attach visual to
     * @param id Unique identifier for this visual
     */
    TrajectoryVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, uint32_t id);

    /**
     * @brief Destructor - cleans up OGRE resources
     */
    ~TrajectoryVisual();

    /**
     * @brief Update trajectory geometry from PathArray message
     *
     * @param paths Candidate trajectories to visualize
     * @param width Ribbon width in meters
     */
    void setTrajectories(const PathArray& paths, double width);

    /**
     * @brief Set the color and transparency of the trajectory
     *
     * @param r Red component (0-1)
     * @param g Green component (0-1)
     * @param b Blue component (0-1)
     * @param a Alpha component (0-1)
     */
    void setColor(float r, float g, float b, float a);

    /**
     * @brief Set visibility of this visual
     *
     * @param visible Whether the visual should be rendered
     */
    void setVisible(bool visible);

    /**
     * @brief Set position offset for the visual
     *
     * @param position Position in the frame
     */
    void setFramePosition(const Ogre::Vector3& position);

    /**
     * @brief Set orientation for the visual
     *
     * @param orientation Orientation quaternion
     */
    void setFrameOrientation(const Ogre::Quaternion& orientation);

private:
    Ogre::SceneManager* scene_manager_;
    Ogre::SceneNode* scene_node_;
    Ogre::ManualObject* manual_object_;

    float color_r_, color_g_, color_b_, color_a_;
    uint32_t id_;
};

}  // namespace autoware_mini

#endif  // AUTOWARE_MINI_RVIZ_TRAJECTORY_VISUAL_H
