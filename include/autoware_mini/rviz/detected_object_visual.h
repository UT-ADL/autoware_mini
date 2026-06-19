// Copyright (c) 2025 Autonomous Driving Lab, University of Tartu
// SPDX-License-Identifier: MIT

#ifndef AUTOWARE_MINI_RVIZ_DETECTED_OBJECT_VISUAL_H
#define AUTOWARE_MINI_RVIZ_DETECTED_OBJECT_VISUAL_H

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/movable_text.h>
#include <rviz/ogre_helpers/shape.h>

#include <autoware_mini/DetectedObject.h>

#include <memory>
#include <string>

namespace autoware_mini
{

/**
 * @brief Visual properties for detected object rendering
 */
struct DetectedObjectVisualProperties
{
    bool show_bbox_3d;
    bool show_bbox_2d;
    bool show_convex_hull;
    bool show_velocity;
    bool show_label;
    bool show_centroid;

    float bbox_r, bbox_g, bbox_b;
    float hull_r, hull_g, hull_b;
    float vel_r, vel_g, vel_b;
    float label_r, label_g, label_b;

    float alpha;
    float line_width;
};

/**
 * @brief Visual representation of a single detected object
 *
 * Uses rviz::BillboardLine to render bounding boxes, convex hulls,
 * and velocity arrows with configurable line width.
 * Uses MovableText for labels.
 */
class DetectedObjectVisual
{
public:
    /**
     * @brief Constructor
     *
     * @param scene_manager OGRE scene manager
     * @param parent_node Parent scene node to attach visual to
     * @param id Unique identifier for this visual
     */
    DetectedObjectVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, uint32_t id);

    /**
     * @brief Destructor - cleans up OGRE resources
     */
    ~DetectedObjectVisual();

    /**
     * @brief Update visual from DetectedObject message
     *
     * @param obj The detected object data
     * @param props Visual properties (colors, visibility, etc.)
     */
    void setObject(const DetectedObject& obj, const DetectedObjectVisualProperties& props);

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
    /**
     * @brief Render 3D bounding box as semi-transparent cube
     */
    void renderBoundingBox3D(const DetectedObject& obj, const DetectedObjectVisualProperties& props);

    /**
     * @brief Render 2D bounding box outline (bottom face)
     */
    void renderBoundingBox2D(const DetectedObject& obj, const DetectedObjectVisualProperties& props);

    /**
     * @brief Render convex hull as line loop
     */
    void renderConvexHull(const DetectedObject& obj, const DetectedObjectVisualProperties& props);

    /**
     * @brief Render velocity arrow
     */
    void renderVelocityArrow(const DetectedObject& obj, const DetectedObjectVisualProperties& props);

    /**
     * @brief Render text label
     */
    void renderLabel(const DetectedObject& obj, const DetectedObjectVisualProperties& props);

    /**
     * @brief Render centroid as a small sphere
     */
    void renderCentroid(const DetectedObject& obj, const DetectedObjectVisualProperties& props);

    Ogre::SceneManager* scene_manager_;
    Ogre::SceneNode* scene_node_;

    // 3D bounding box as semi-transparent cube
    std::unique_ptr<rviz::Shape> bbox_shape_;

    // Centroid marker as a sphere
    std::unique_ptr<rviz::Shape> centroid_shape_;

    // 2D bounding box outline (bottom face) and convex hull
    std::unique_ptr<rviz::BillboardLine> bbox_outline_;
    std::unique_ptr<rviz::BillboardLine> hull_lines_;

    // Arrow for velocity (cylindrical shaft with cone head)
    std::unique_ptr<rviz::Arrow> velocity_arrow_;

    // Text label
    std::unique_ptr<rviz::MovableText> label_text_;
    Ogre::SceneNode* label_node_;

    uint32_t id_;
};

}  // namespace autoware_mini

#endif  // AUTOWARE_MINI_RVIZ_DETECTED_OBJECT_VISUAL_H
