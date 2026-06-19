// Copyright (c) 2025 Autonomous Driving Lab, University of Tartu
// SPDX-License-Identifier: MIT

#ifndef AUTOWARE_MINI_RVIZ_DETECTED_OBJECTS_DISPLAY_H
#define AUTOWARE_MINI_RVIZ_DETECTED_OBJECTS_DISPLAY_H

#ifndef Q_MOC_RUN
#include <rviz/message_filter_display.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>

#include <autoware_mini/DetectedObjectArray.h>
#include <autoware_mini/rviz/detected_object_visual.h>
#endif

#include <map>
#include <memory>

namespace autoware_mini
{

/**
 * @brief RViz Display plugin for visualizing detected objects
 *
 * Subscribes to DetectedObjectArray messages and renders bounding boxes,
 * convex hulls, velocity arrows, and text labels for each detected object.
 * Uses OGRE for direct rendering without the ROS marker system.
 *
 * Properties:
 * - Show 3D Bounding Box: Toggle 3D semi-transparent bounding box visualization
 * - Show 2D Bounding Box: Toggle 2D bounding box outline visualization
 * - Show Convex Hull: Toggle convex hull polygon visualization
 * - Show Velocity: Toggle velocity arrow visualization
 * - Show Label: Toggle text label visualization
 * - Show Centroid: Toggle centroid sphere visualization
 * - Bounding Box Color: Color for bounding boxes
 * - Convex Hull Color: Color for convex hulls
 * - Velocity Color: Color for velocity arrows
 * - Label Color: Color for text labels
 * - Alpha: Overall transparency
 * - Line Width: Width of lines for boxes and hulls
 */
class DetectedObjectsDisplay : public rviz::MessageFilterDisplay<DetectedObjectArray>
{
    Q_OBJECT

public:
    DetectedObjectsDisplay();
    virtual ~DetectedObjectsDisplay();

protected:
    /**
     * @brief Called when display is enabled/disabled
     */
    void onInitialize() override;

    /**
     * @brief Called when display is reset (e.g., Fixed Frame changes)
     */
    void reset() override;

private Q_SLOTS:
    /**
     * @brief Update cached visual properties when any property changes
     */
    void updateVisualProperties();

private:
    /**
     * @brief Process incoming DetectedObjectArray message
     *
     * @param msg The message containing detected objects
     */
    void processMessage(const DetectedObjectArray::ConstPtr& msg) override;

    /**
     * @brief Clear all visuals
     */
    void clearVisuals();

    // Visual storage: maps object ID to its visual
    std::map<uint32_t, std::unique_ptr<DetectedObjectVisual>> visuals_;

    // Visibility properties
    rviz::BoolProperty* show_bbox_3d_property_;
    rviz::BoolProperty* show_bbox_2d_property_;
    rviz::BoolProperty* show_convex_hull_property_;
    rviz::BoolProperty* show_velocity_property_;
    rviz::BoolProperty* show_label_property_;
    rviz::BoolProperty* show_centroid_property_;

    // Color properties
    rviz::ColorProperty* bbox_color_property_;
    rviz::ColorProperty* convex_hull_color_property_;
    rviz::ColorProperty* velocity_color_property_;
    rviz::ColorProperty* label_color_property_;

    // Other properties
    rviz::FloatProperty* alpha_property_;
    rviz::FloatProperty* line_width_property_;

    // Cached visual properties (updated when properties change)
    DetectedObjectVisualProperties visual_props_;
};

}  // namespace autoware_mini

#endif  // AUTOWARE_MINI_RVIZ_DETECTED_OBJECTS_DISPLAY_H
