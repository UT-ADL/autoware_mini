// Copyright (c) 2025 Autonomous Driving Lab, University of Tartu
// SPDX-License-Identifier: MIT

#ifndef AUTOWARE_MINI_RVIZ_LOCAL_PATH_VISUAL_H
#define AUTOWARE_MINI_RVIZ_LOCAL_PATH_VISUAL_H

#include <memory>
#include <vector>

#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/movable_text.h>

#include <autoware_mini/LocalPath.h>
#include <autoware_mini/Waypoint.h>
#include <autoware_mini/rviz/rviz_helpers.h>

namespace autoware_mini
{

/**
 * @brief Visual properties for local path rendering
 */
struct LocalPathVisualProperties
{
    bool show_path_ribbon;
    float path_r, path_g, path_b;
    float alpha;
    float path_width;
    bool show_velocity_labels;
    bool show_stopping_point;
    bool show_lane_boundary_safety_box;
    float lb_safety_box_r, lb_safety_box_g, lb_safety_box_b, lb_safety_box_alpha;
    float stopped_speed_limit;

    // Swerving mode properties
    bool use_swerving;
    float narrow_safety_box_width;
    float wide_safety_box_width;
    float distance_to_car_front;
    float current_x, current_y, current_z;
    bool has_current_pose;
};

/**
 * @brief Visual representation of the local path
 *
 * Renders the planned trajectory as a triangulated ribbon
 * with velocity labels at waypoints. Delegates stopping point
 * rendering to StoppingPointVisual.
 */
class LocalPathVisual
{
public:
    /**
     * @brief Constructor
     *
     * @param scene_manager OGRE scene manager
     * @param parent_node Parent scene node to attach visual to
     */
    LocalPathVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);

    /**
     * @brief Destructor - cleans up OGRE resources
     */
    ~LocalPathVisual();

    /**
     * @brief Update visual from LocalPath message
     *
     * @param msg The local path message
     * @param props Visual properties (colors, visibility, etc.)
     */
    void setPath(const LocalPath& msg, const LocalPathVisualProperties& props);

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

    /**
     * @brief Clear all visual elements
     */
    void clear();

private:
    /**
     * @brief Render the path ribbon
     */
    void renderPathRibbon(const std::vector<Ogre::Vector3>& centerline, const LocalPathVisualProperties& props);

    /**
     * @brief Render stopping point marker
     */
    void renderStoppingPoint(const std::vector<Ogre::Vector3>& centerline, const LocalPath& msg,
                             const LocalPathVisualProperties& props);

    /**
     * @brief Render lane boundary safety box corridor
     */
    void renderLaneBoundarySafetyBox(const std::vector<Ogre::Vector3>& centerline,
                                     const LocalPathVisualProperties& props);

    /**
     * @brief Render velocity labels at waypoint positions
     */
    void renderVelocityLabels(const std::vector<Waypoint>& waypoints, size_t num_labels, bool visible);

    /**
     * @brief Hide all velocity labels
     */
    void hideVelocityLabels();

    Ogre::SceneManager* scene_manager_;
    Ogre::SceneNode* scene_node_;

    // Manual object for path ribbon triangles
    Ogre::ManualObject* path_object_;

    // Manual object for lane boundary safety box
    Ogre::ManualObject* lane_boundary_safety_box_object_;

    // Velocity labels
    std::vector<std::unique_ptr<rviz::MovableText>> velocity_labels_;
    std::vector<Ogre::SceneNode*> velocity_label_nodes_;

    // Stopping point cube shape
    std::unique_ptr<rviz::Shape> stopping_point_shape_;
};

}  // namespace autoware_mini

#endif  // AUTOWARE_MINI_RVIZ_LOCAL_PATH_VISUAL_H
