// Copyright (c) 2025 Autonomous Driving Lab, University of Tartu
// SPDX-License-Identifier: MIT

#include <autoware_mini/rviz/predicted_trajectory_display.h>

#include <unordered_set>

#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>

#include <pluginlib/class_list_macros.h>

namespace autoware_mini
{

PredictedTrajectoryDisplay::PredictedTrajectoryDisplay()
{
    color_property_ = new rviz::ColorProperty(
        "Color",
        QColor(255, 255, 0),  // Yellow
        "Color of the trajectory ribbons.",
        this);

    alpha_property_ = new rviz::FloatProperty(
        "Alpha",
        0.5f,
        "Transparency of the trajectory ribbons (0 = invisible, 1 = opaque).",
        this);
    alpha_property_->setMin(0.0f);
    alpha_property_->setMax(1.0f);
}

PredictedTrajectoryDisplay::~PredictedTrajectoryDisplay()
{
    clearVisuals();
}

void PredictedTrajectoryDisplay::onInitialize()
{
    MFDClass::onInitialize();
}

void PredictedTrajectoryDisplay::reset()
{
    MFDClass::reset();
    clearVisuals();
}

void PredictedTrajectoryDisplay::clearVisuals()
{
    visuals_.clear();
}

void PredictedTrajectoryDisplay::processMessage(const DetectedObjectArray::ConstPtr& msg)
{
    // Get transform from message frame to fixed frame
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;

    if (!context_->getFrameManager()->getTransform(
            msg->header.frame_id, msg->header.stamp, position, orientation))
    {
        ROS_ERROR_THROTTLE(1.0, "PredictedTrajectoryDisplay: Error transforming from frame '%s' to frame '%s'",
                  msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
        return;
    }

    // Get current property values
    QColor color = color_property_->getColor();
    float alpha = alpha_property_->getFloat();
    float r = color.redF();
    float g = color.greenF();
    float b = color.blueF();

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
            auto visual = std::make_unique<TrajectoryVisual>(
                context_->getSceneManager(), scene_node_, obj.id);
            it = visuals_.emplace(obj.id, std::move(visual)).first;
        }

        TrajectoryVisual* visual = it->second.get();

        // Update visual using object width
        visual->setFramePosition(position);
        visual->setFrameOrientation(orientation);
        visual->setColor(r, g, b, alpha);
        visual->setTrajectories(obj.candidate_trajectories, obj.dimensions.y);
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
PLUGINLIB_EXPORT_CLASS(autoware_mini::PredictedTrajectoryDisplay, rviz::Display)
