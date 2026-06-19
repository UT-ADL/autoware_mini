// Copyright (c) 2025 Autonomous Driving Lab, University of Tartu
// SPDX-License-Identifier: MIT

#include <autoware_mini/rviz/detected_objects_display.h>

#include <unordered_set>

#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>

#include <pluginlib/class_list_macros.h>

namespace autoware_mini
{

DetectedObjectsDisplay::DetectedObjectsDisplay()
{
    // Visibility properties
    show_bbox_3d_property_ = new rviz::BoolProperty(
        "Show 3D Bounding Box",
        true,
        "Show 3D semi-transparent bounding box.",
        this,
        SLOT(updateVisualProperties()));

    show_bbox_2d_property_ = new rviz::BoolProperty(
        "Show 2D Bounding Box",
        true,
        "Show 2D bounding box outline.",
        this,
        SLOT(updateVisualProperties()));

    show_convex_hull_property_ = new rviz::BoolProperty(
        "Show Convex Hull",
        true,
        "Show convex hull polygon.",
        this,
        SLOT(updateVisualProperties()));

    show_velocity_property_ = new rviz::BoolProperty(
        "Show Velocity",
        true,
        "Show velocity arrow.",
        this,
        SLOT(updateVisualProperties()));

    show_label_property_ = new rviz::BoolProperty(
        "Show Label",
        true,
        "Show text label with object info.",
        this,
        SLOT(updateVisualProperties()));

    show_centroid_property_ = new rviz::BoolProperty(
        "Show Centroid",
        true,
        "Show centroid as a small sphere.",
        this,
        SLOT(updateVisualProperties()));

    // Color properties
    bbox_color_property_ = new rviz::ColorProperty(
        "Bounding Box Color",
        QColor(255, 0, 0),  // Red
        "Color of bounding boxes.",
        this,
        SLOT(updateVisualProperties()));

    convex_hull_color_property_ = new rviz::ColorProperty(
        "Convex Hull Color",
        QColor(0, 255, 0),  // Green
        "Color of convex hulls.",
        this,
        SLOT(updateVisualProperties()));

    velocity_color_property_ = new rviz::ColorProperty(
        "Velocity Color",
        QColor(255, 255, 0),  // Yellow
        "Color of velocity arrows.",
        this,
        SLOT(updateVisualProperties()));

    label_color_property_ = new rviz::ColorProperty(
        "Label Color",
        QColor(255, 255, 255),  // White
        "Color of text labels.",
        this,
        SLOT(updateVisualProperties()));

    // Other properties
    alpha_property_ = new rviz::FloatProperty(
        "Alpha",
        0.8f,
        "Transparency (0 = invisible, 1 = opaque).",
        this,
        SLOT(updateVisualProperties()));
    alpha_property_->setMin(0.0f);
    alpha_property_->setMax(1.0f);

    line_width_property_ = new rviz::FloatProperty(
        "Line Width",
        2.0f,
        "Width of lines for boxes and hulls (in pixels).",
        this,
        SLOT(updateVisualProperties()));
    line_width_property_->setMin(1.0f);
    line_width_property_->setMax(20.0f);
}

DetectedObjectsDisplay::~DetectedObjectsDisplay()
{
    clearVisuals();
}

void DetectedObjectsDisplay::onInitialize()
{
    MFDClass::onInitialize();
    updateVisualProperties();
}

void DetectedObjectsDisplay::reset()
{
    MFDClass::reset();
    clearVisuals();
}

void DetectedObjectsDisplay::clearVisuals()
{
    visuals_.clear();
}

void DetectedObjectsDisplay::updateVisualProperties()
{
    visual_props_.show_bbox_3d = show_bbox_3d_property_->getBool();
    visual_props_.show_bbox_2d = show_bbox_2d_property_->getBool();
    visual_props_.show_convex_hull = show_convex_hull_property_->getBool();
    visual_props_.show_velocity = show_velocity_property_->getBool();
    visual_props_.show_label = show_label_property_->getBool();
    visual_props_.show_centroid = show_centroid_property_->getBool();

    QColor bbox_color = bbox_color_property_->getColor();
    visual_props_.bbox_r = bbox_color.redF();
    visual_props_.bbox_g = bbox_color.greenF();
    visual_props_.bbox_b = bbox_color.blueF();

    QColor hull_color = convex_hull_color_property_->getColor();
    visual_props_.hull_r = hull_color.redF();
    visual_props_.hull_g = hull_color.greenF();
    visual_props_.hull_b = hull_color.blueF();

    QColor vel_color = velocity_color_property_->getColor();
    visual_props_.vel_r = vel_color.redF();
    visual_props_.vel_g = vel_color.greenF();
    visual_props_.vel_b = vel_color.blueF();

    QColor label_color = label_color_property_->getColor();
    visual_props_.label_r = label_color.redF();
    visual_props_.label_g = label_color.greenF();
    visual_props_.label_b = label_color.blueF();

    visual_props_.alpha = alpha_property_->getFloat();
    visual_props_.line_width = line_width_property_->getFloat();
}

void DetectedObjectsDisplay::processMessage(const DetectedObjectArray::ConstPtr& msg)
{
    // Get transform from message frame to fixed frame
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;

    if (!context_->getFrameManager()->getTransform(
            msg->header.frame_id, msg->header.stamp, position, orientation))
    {
        ROS_ERROR_THROTTLE(1.0, "DetectedObjectsDisplay: Error transforming from frame '%s' to frame '%s'",
                  msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
        return;
    }

    // Track which IDs are in this message
    std::unordered_set<uint32_t> current_ids;

    // Process each detected object
    for (const auto& obj : msg->objects)
    {
        current_ids.insert(obj.id);

        // Get or create visual for this object
        auto it = visuals_.find(obj.id);
        if (it == visuals_.end())
        {
            auto visual = std::make_unique<DetectedObjectVisual>(
                context_->getSceneManager(), scene_node_, obj.id);
            it = visuals_.emplace(obj.id, std::move(visual)).first;
        }

        DetectedObjectVisual* visual = it->second.get();

        // Update visual
        visual->setFramePosition(position);
        visual->setFrameOrientation(orientation);
        visual->setObject(obj, visual_props_);
        visual->setVisible(true);
    }

    // Remove visuals for objects no longer present
    for (auto it = visuals_.begin(); it != visuals_.end();)
    {
        if (current_ids.find(it->first) == current_ids.end())
        {
            it = visuals_.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

}  // namespace autoware_mini

// Register this display plugin with pluginlib
PLUGINLIB_EXPORT_CLASS(autoware_mini::DetectedObjectsDisplay, rviz::Display)
