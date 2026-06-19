// Copyright (c) 2025 Autonomous Driving Lab, University of Tartu
// SPDX-License-Identifier: MIT

#include <cmath>
#include <cstdio>

#include <OgreQuaternion.h>

#include <autoware_mini/rviz/local_path_visual.h>
#include <autoware_mini/rviz/rviz_helpers.h>

namespace autoware_mini
{

// Z offset for path ribbon to render above ground
static constexpr float PATH_Z_OFFSET = 0.1f;

// GOAL_POINT = 1 (from CollisionPoints class in collision.py)
static constexpr int GOAL_COLLISION_POINT = 1;

// Velocity label character height
static constexpr float LABEL_CHAR_HEIGHT = 0.5f;

// Stopping point dimensions
static constexpr float STOPPING_POINT_WIDTH = 0.3f;
static constexpr float STOPPING_POINT_DEPTH = 5.0f;
static constexpr float STOPPING_POINT_HEIGHT = 2.5f;
static constexpr float STOPPING_POINT_Z_OFFSET = 1.0f;

LocalPathVisual::LocalPathVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
    : scene_manager_(scene_manager)
{
    // Create scene node as child of parent
    scene_node_ = parent_node->createChildSceneNode();

    // Create manual objects for path ribbon and lane boundary safety box
    path_object_ = createDynamicManualObject(scene_manager_, scene_node_, "LocalPathVisual", this);
    lane_boundary_safety_box_object_ = createDynamicManualObject(
        scene_manager_, scene_node_, "LocalPathVisual_LBSafetyBox", this);

    // Create stopping point cube shape (hidden by default)
    stopping_point_shape_ = std::make_unique<rviz::Shape>(rviz::Shape::Cube, scene_manager_, scene_node_);
    stopping_point_shape_->getRootNode()->setVisible(false);
}

LocalPathVisual::~LocalPathVisual()
{
    // stopping_point_shape_ is destroyed automatically by unique_ptr

    // Destroy velocity label nodes
    for (auto* node : velocity_label_nodes_)
    {
        scene_manager_->destroySceneNode(node);
    }

    // Destroy path object and lane boundary safety box object
    scene_manager_->destroyManualObject(path_object_);
    scene_manager_->destroyManualObject(lane_boundary_safety_box_object_);
    scene_manager_->destroySceneNode(scene_node_);
}

void LocalPathVisual::setPath(const LocalPath& msg, const LocalPathVisualProperties& props)
{
    // Convert waypoints to centerline once
    std::vector<Ogre::Vector3> centerline;
    centerline.reserve(msg.waypoints.size());
    for (const auto& wp : msg.waypoints)
    {
        centerline.push_back(Ogre::Vector3(wp.position.x, wp.position.y, wp.position.z + PATH_Z_OFFSET));
    }

    renderPathRibbon(centerline, props);
    renderLaneBoundarySafetyBox(centerline, props);

    // Render velocity labels - count up to first zero velocity
    size_t num_labels = 0;
    if (props.show_velocity_labels && msg.waypoints.size() >= 2)
    {
        for (size_t i = 0; i < msg.waypoints.size(); ++i)
        {
            num_labels = i + 1;
            if (std::abs(msg.waypoints[i].speed) < 0.001f)
            {
                break;
            }
        }
    }
    renderVelocityLabels(msg.waypoints, num_labels, props.show_velocity_labels);

    renderStoppingPoint(centerline, msg, props);
}

void LocalPathVisual::renderPathRibbon(const std::vector<Ogre::Vector3>& centerline,
                                       const LocalPathVisualProperties& props)
{
    path_object_->clear();

    if (!props.show_path_ribbon || centerline.size() < 2)
    {
        return;
    }

    std::vector<Ogre::Vector3> strip;

    if (props.use_swerving && props.has_current_pose)
    {
        // Swerving mode: narrow width for full path, wide width for front part
        // The union of these two polygons is triangulated together

        // First, find the projection of current position onto the path
        double min_dist_sq = std::numeric_limits<double>::max();
        size_t closest_idx = 0;
        double closest_t = 0.0;

        for (size_t i = 0; i < centerline.size() - 1; ++i)
        {
            // Project current position onto segment i to i+1
            double dx = centerline[i + 1].x - centerline[i].x;
            double dy = centerline[i + 1].y - centerline[i].y;
            double seg_len_sq = dx * dx + dy * dy;

            if (seg_len_sq < 1e-9)
                continue;

            double t = ((props.current_x - centerline[i].x) * dx +
                        (props.current_y - centerline[i].y) * dy) / seg_len_sq;
            t = std::max(0.0, std::min(1.0, t));

            double proj_x = centerline[i].x + t * dx;
            double proj_y = centerline[i].y + t * dy;
            double dist_sq = (props.current_x - proj_x) * (props.current_x - proj_x) +
                             (props.current_y - proj_y) * (props.current_y - proj_y);

            if (dist_sq < min_dist_sq)
            {
                min_dist_sq = dist_sq;
                closest_idx = i;
                closest_t = t;
            }
        }

        // Calculate distance along path to the projection point
        double dist_to_projection = 0.0;
        for (size_t i = 0; i < closest_idx; ++i)
        {
            double dx = centerline[i + 1].x - centerline[i].x;
            double dy = centerline[i + 1].y - centerline[i].y;
            dist_to_projection += std::sqrt(dx * dx + dy * dy);
        }
        // Add partial segment
        double dx = centerline[closest_idx + 1].x - centerline[closest_idx].x;
        double dy = centerline[closest_idx + 1].y - centerline[closest_idx].y;
        dist_to_projection += closest_t * std::sqrt(dx * dx + dy * dy);

        // The trim point is at projection + distance_to_car_front
        double trim_distance = dist_to_projection + props.distance_to_car_front;

        // Split centerline at trim point and triangulate each part with its own width
        std::vector<Ogre::Vector3> narrow_centerline, wide_centerline;
        splitCenterlineAtDistance(centerline, trim_distance, narrow_centerline, wide_centerline);
        triangulateRibbon(narrow_centerline, props.narrow_safety_box_width, strip);
        triangulateRibbon(wide_centerline, props.wide_safety_box_width, strip);
    }
    else
    {
        // Normal mode: single width
        triangulateRibbon(centerline, props.path_width, strip);
    }

    renderTriangleStrip(path_object_, strip, props.path_r, props.path_g, props.path_b, props.alpha);
}

void LocalPathVisual::renderStoppingPoint(const std::vector<Ogre::Vector3>& centerline,
                                          const LocalPath& msg,
                                          const LocalPathVisualProperties& props)
{
    // Hide first - will be shown if conditions are met
    stopping_point_shape_->getRootNode()->setVisible(false);

    if (!props.show_stopping_point || !msg.is_blocked || centerline.size() < 2)
    {
        return;
    }

    // Interpolate point at stopping distance
    float stopping_distance = std::max(msg.stopping_point_distance, 0.0f);
    Ogre::Vector3 point;
    float heading;
    if (!getPointAtDistance(centerline, stopping_distance, point, heading))
    {
        return;
    }

    // Sanity check: don't render at origin (indicates bad data)
    if (std::abs(point.x) < 0.01 && std::abs(point.y) < 0.01)
    {
        return;
    }

    // Determine color based on collision category and object speed
    float r, g, b, alpha;

    // GOAL_POINT = 1 (from CollisionPoints class in collision.py)
    if (msg.collision_point_category == GOAL_COLLISION_POINT)
    {
        // White - goal point
        r = 0.9f; g = 0.9f; b = 0.9f; alpha = 0.2f;
    }
    else if (msg.target_object_speed < props.stopped_speed_limit)
    {
        // Red - stopped obstacle
        r = 1.0f; g = 0.0f; b = 0.0f; alpha = 0.5f;
    }
    else
    {
        // Yellow - following moving obstacle
        r = 1.0f; g = 1.0f; b = 0.0f; alpha = 0.5f;
    }

    // Position the cube at stopping point with Z offset
    float center_z = point.z + STOPPING_POINT_Z_OFFSET;
    stopping_point_shape_->setPosition(Ogre::Vector3(point.x, point.y, center_z));

    // Rotate by heading around Z axis
    Ogre::Quaternion orientation(Ogre::Radian(heading), Ogre::Vector3::UNIT_Z);
    stopping_point_shape_->setOrientation(orientation);

    // Scale to stopping point dimensions
    stopping_point_shape_->setScale(Ogre::Vector3(STOPPING_POINT_WIDTH, STOPPING_POINT_DEPTH, STOPPING_POINT_HEIGHT));

    // Set color with transparency
    stopping_point_shape_->setColor(r, g, b, alpha);

    stopping_point_shape_->getRootNode()->setVisible(true);
}

void LocalPathVisual::renderLaneBoundarySafetyBox(const std::vector<Ogre::Vector3>& centerline,
                                                   const LocalPathVisualProperties& props)
{
    lane_boundary_safety_box_object_->clear();

    if (!props.show_lane_boundary_safety_box || centerline.size() < 2)
    {
        return;
    }

    // Always use narrow_safety_box_width regardless of swerving mode
    // Centerline already has PATH_Z_OFFSET applied in setPath()
    std::vector<Ogre::Vector3> strip;
    triangulateRibbon(centerline, props.narrow_safety_box_width, strip);

    renderTriangleStrip(lane_boundary_safety_box_object_, strip,
                   props.lb_safety_box_r, props.lb_safety_box_g,
                   props.lb_safety_box_b, props.lb_safety_box_alpha);
}

void LocalPathVisual::setVisible(bool visible)
{
    // Don't cascade visibility to children - they manage their own visibility
    scene_node_->setVisible(visible, false);
}

void LocalPathVisual::setFramePosition(const Ogre::Vector3& position)
{
    scene_node_->setPosition(position);
}

void LocalPathVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
    scene_node_->setOrientation(orientation);
}

void LocalPathVisual::clear()
{
    path_object_->clear();
    lane_boundary_safety_box_object_->clear();
    hideVelocityLabels();
    stopping_point_shape_->getRootNode()->setVisible(false);
}

void LocalPathVisual::renderVelocityLabels(const std::vector<Waypoint>& waypoints,
                                           size_t num_labels,
                                           bool visible)
{
    if (!visible || num_labels == 0)
    {
        hideVelocityLabels();
        return;
    }

    num_labels = std::min(num_labels, waypoints.size());

    // Reserve capacity to prevent reallocation during loop
    velocity_labels_.reserve(num_labels);
    velocity_label_nodes_.reserve(num_labels);

    // Update labels (creating new ones if needed)
    for (size_t i = 0; i < num_labels; ++i)
    {
        if (i >= velocity_labels_.size())
        {
            auto* node = scene_node_->createChildSceneNode();
            auto text = std::make_unique<rviz::MovableText>("0.0");
            text->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_CENTER);
            text->setCharacterHeight(LABEL_CHAR_HEIGHT);
            node->attachObject(text.get());

            velocity_label_nodes_.push_back(node);
            velocity_labels_.push_back(std::move(text));
        }

        const auto& wp = waypoints[i];

        // Format speed in km/h with 1 decimal
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%.1f", wp.speed * 3.6f);

        velocity_labels_[i]->setCaption(buf);
        velocity_labels_[i]->setColor(Ogre::ColourValue(1.0f, 1.0f, 1.0f, 1.0f));

        velocity_label_nodes_[i]->setPosition(wp.position.x, wp.position.y, wp.position.z);
        velocity_label_nodes_[i]->setVisible(true);
    }

    // Hide unused labels
    for (size_t i = num_labels; i < velocity_label_nodes_.size(); ++i)
    {
        velocity_label_nodes_[i]->setVisible(false);
    }
}

void LocalPathVisual::hideVelocityLabels()
{
    for (auto* node : velocity_label_nodes_)
    {
        node->setVisible(false);
    }
}

}  // namespace autoware_mini
