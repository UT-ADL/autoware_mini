// Copyright (c) 2025 Autonomous Driving Lab, University of Tartu
// SPDX-License-Identifier: MIT

#ifndef AUTOWARE_MINI_RVIZ_LOCAL_PATH_DISPLAY_H
#define AUTOWARE_MINI_RVIZ_LOCAL_PATH_DISPLAY_H

#ifndef Q_MOC_RUN
#include <rviz/message_filter_display.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>

#include <ros/subscriber.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <autoware_mini/LocalPath.h>
#include <autoware_mini/rviz/local_path_visual.h>
#endif

#include <memory>
#include <mutex>

namespace autoware_mini
{

/**
 * @brief RViz Display plugin for visualizing the local path
 *
 * Subscribes to LocalPath messages and renders the planned trajectory
 * as a colored ribbon with velocity labels and stopping point marker.
 * Path width comes from ROS parameters (safety_box_width).
 * Supports swerving mode with dual-width visualization.
 *
 * Properties:
 * - Path Color: Color of the path ribbon
 * - Alpha: Transparency (0 = invisible, 1 = opaque)
 * - Show Velocity Labels: Toggle velocity text labels
 * - Show Stopping Point: Toggle stopping point marker
 */
class LocalPathDisplay : public rviz::MessageFilterDisplay<LocalPath>
{
    Q_OBJECT

public:
    LocalPathDisplay();
    virtual ~LocalPathDisplay();

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
     * @brief Process incoming LocalPath message
     *
     * @param msg The message containing the local path
     */
    void processMessage(const LocalPath::ConstPtr& msg) override;

    /**
     * @brief Callback for current pose messages
     */
    void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    /**
     * @brief Clear all visuals
     */
    void clearVisuals();

    // Visual for rendering the local path
    std::unique_ptr<LocalPathVisual> visual_;

    // User-configurable properties
    rviz::BoolProperty* show_path_ribbon_property_;
    rviz::ColorProperty* path_color_property_;
    rviz::FloatProperty* alpha_property_;
    rviz::BoolProperty* show_velocity_labels_property_;
    rviz::BoolProperty* show_stopping_point_property_;
    rviz::BoolProperty* show_lane_boundary_safety_box_property_;
    rviz::ColorProperty* lb_safety_box_color_property_;
    rviz::FloatProperty* lb_safety_box_alpha_property_;

    // Cached visual properties
    LocalPathVisualProperties visual_props_;

    // ROS parameters
    bool use_swerving_;
    float safety_box_width_;
    float narrow_safety_box_width_;
    float wide_safety_box_width_;
    float distance_to_car_front_;
    float stopped_speed_limit_;

    // Current pose subscriber and cached position
    ros::Subscriber current_pose_sub_;
    float current_x_;
    float current_y_;
    float current_z_;
    bool has_current_pose_;
    std::mutex pose_mutex_;

    // TF for looking up distance to car front
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace autoware_mini

#endif  // AUTOWARE_MINI_RVIZ_LOCAL_PATH_DISPLAY_H
