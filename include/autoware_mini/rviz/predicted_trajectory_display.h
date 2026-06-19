// Copyright (c) 2025 Autonomous Driving Lab, University of Tartu
// SPDX-License-Identifier: MIT

#ifndef AUTOWARE_MINI_RVIZ_PREDICTED_TRAJECTORY_DISPLAY_H
#define AUTOWARE_MINI_RVIZ_PREDICTED_TRAJECTORY_DISPLAY_H

#ifndef Q_MOC_RUN
#include <rviz/message_filter_display.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>

#include <autoware_mini/DetectedObjectArray.h>
#include <autoware_mini/rviz/trajectory_visual.h>
#endif

#include <map>
#include <memory>

namespace autoware_mini
{

/**
 * @brief RViz Display plugin for visualizing predicted trajectories
 *
 * Subscribes to DetectedObjectArray messages and renders the
 * candidate_trajectories field for each detected object as colored
 * ribbons. Uses the object's width (dimensions.y) for ribbon width.
 *
 * Properties:
 * - Color: Base color for trajectory ribbons
 * - Alpha: Transparency (0 = invisible, 1 = opaque)
 */
class PredictedTrajectoryDisplay : public rviz::MessageFilterDisplay<DetectedObjectArray>
{
    Q_OBJECT

public:
    PredictedTrajectoryDisplay();
    virtual ~PredictedTrajectoryDisplay();

protected:
    /**
     * @brief Called when display is enabled/disabled
     */
    void onInitialize() override;

    /**
     * @brief Called when display is reset (e.g., Fixed Frame changes)
     */
    void reset() override;

private:
    /**
     * @brief Process incoming DetectedObjectArray message
     *
     * @param msg The message containing detected objects with trajectories
     */
    void processMessage(const DetectedObjectArray::ConstPtr& msg) override;

    /**
     * @brief Clear all visuals
     */
    void clearVisuals();

    // Visual storage: maps object ID to its visual
    std::map<uint32_t, std::unique_ptr<TrajectoryVisual>> visuals_;

    // User-configurable properties
    rviz::ColorProperty* color_property_;
    rviz::FloatProperty* alpha_property_;
};

}  // namespace autoware_mini

#endif  // AUTOWARE_MINI_RVIZ_PREDICTED_TRAJECTORY_DISPLAY_H
