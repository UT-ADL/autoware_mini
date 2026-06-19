// Copyright (c) 2025 Autonomous Driving Lab, University of Tartu
// SPDX-License-Identifier: MIT

#include <autoware_mini/rviz/local_path_display.h>

#include <stdexcept>

#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>

#include <pluginlib/class_list_macros.h>

namespace autoware_mini
{

LocalPathDisplay::LocalPathDisplay()
    : distance_to_car_front_(0.0f)
    , current_x_(0.0f)
    , current_y_(0.0f)
    , current_z_(0.0f)
    , has_current_pose_(false)
{
    show_path_ribbon_property_ = new rviz::BoolProperty(
        "Show Local Path",
        true,
        "Show the local path.",
        this,
        SLOT(updateVisualProperties()));

    path_color_property_ = new rviz::ColorProperty(
        "Local Path Color",
        QColor(25, 255, 25),  // Green (0.1, 1.0, 0.1)
        "Color of the local path ribbon.",
        this,
        SLOT(updateVisualProperties()));

    alpha_property_ = new rviz::FloatProperty(
        "Alpha",
        0.6f,
        "Transparency of the local path ribbon (0 = invisible, 1 = opaque).",
        this,
        SLOT(updateVisualProperties()));
    alpha_property_->setMin(0.0f);
    alpha_property_->setMax(1.0f);

    show_velocity_labels_property_ = new rviz::BoolProperty(
        "Show Velocity Labels",
        true,
        "Show velocity labels at waypoints.",
        this,
        SLOT(updateVisualProperties()));

    show_stopping_point_property_ = new rviz::BoolProperty(
        "Show Stopping Point",
        true,
        "Show stopping point marker.",
        this,
        SLOT(updateVisualProperties()));

    show_lane_boundary_safety_box_property_ = new rviz::BoolProperty(
        "Show Narrow Safety Box",
        false,
        "Show the narrow lane boundary safety corridor.",
        this,
        SLOT(updateVisualProperties()));

    lb_safety_box_color_property_ = new rviz::ColorProperty(
        "Narrow Safety Box Color",
        QColor(255, 102, 25),  // Orange (1.0, 0.4, 0.1)
        "Color of the narrow safety box.",
        this,
        SLOT(updateVisualProperties()));

    lb_safety_box_alpha_property_ = new rviz::FloatProperty(
        "Narrow Safety Box Alpha",
        0.6f,
        "Transparency of the narrow safety box (0 = invisible, 1 = opaque).",
        this,
        SLOT(updateVisualProperties()));
    lb_safety_box_alpha_property_->setMin(0.0f);
    lb_safety_box_alpha_property_->setMax(1.0f);
}

LocalPathDisplay::~LocalPathDisplay()
{
    current_pose_sub_.shutdown();
    clearVisuals();
}

void LocalPathDisplay::onInitialize()
{
    MFDClass::onInitialize();

    // Read parameters from ROS parameter server
    ros::NodeHandle nh;

    // Get use_swerving parameter (set based on local_planner in planning.launch)
    if (!nh.getParam("/planning/use_swerving", use_swerving_))
        throw std::runtime_error("Required parameter not found: /planning/use_swerving");

    // Get safety box widths from the planning namespace
    if (!nh.getParam("/planning/safety_box_width", safety_box_width_))
        throw std::runtime_error("Required parameter not found: /planning/safety_box_width");
    if (!nh.getParam("/planning/narrow_safety_box_width", narrow_safety_box_width_))
        throw std::runtime_error("Required parameter not found: /planning/narrow_safety_box_width");
    if (!nh.getParam("/planning/wide_safety_box_width", wide_safety_box_width_))
        throw std::runtime_error("Required parameter not found: /planning/wide_safety_box_width");

    // Initialize TF buffer and listener for looking up distance to car front
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // Get stopped speed limit for stopping point color
    if (!nh.getParam("/planning/stopped_speed_limit", stopped_speed_limit_))
        throw std::runtime_error("Required parameter not found: /planning/stopped_speed_limit");

    // Look up distance to car front via TF (static transform)
    if (use_swerving_)
    {
        try
        {
            auto transform = tf_buffer_->lookupTransform("base_link", "car_front", ros::Time(0),
                                                         ros::Duration(1.0));
            distance_to_car_front_ = transform.transform.translation.x;
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_ERROR("LocalPathDisplay: Could not get transform from base_link to car_front: %s", ex.what());
        }
    }

    // Subscribe to current pose
    current_pose_sub_ = nh.subscribe("/localization/current_pose", 1,
                                     &LocalPathDisplay::currentPoseCallback, this);

    updateVisualProperties();

    // Create the visual
    visual_ = std::make_unique<LocalPathVisual>(
        context_->getSceneManager(), scene_node_);
}

void LocalPathDisplay::reset()
{
    MFDClass::reset();
    clearVisuals();
}

void LocalPathDisplay::clearVisuals()
{
    if (visual_)
    {
        visual_->clear();
    }
}

void LocalPathDisplay::currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_x_ = msg->pose.position.x;
    current_y_ = msg->pose.position.y;
    current_z_ = msg->pose.position.z;
    has_current_pose_ = true;
}

void LocalPathDisplay::updateVisualProperties()
{
    visual_props_.show_path_ribbon = show_path_ribbon_property_->getBool();

    QColor path_color = path_color_property_->getColor();
    visual_props_.path_r = path_color.redF();
    visual_props_.path_g = path_color.greenF();
    visual_props_.path_b = path_color.blueF();

    visual_props_.alpha = alpha_property_->getFloat();
    visual_props_.path_width = safety_box_width_;
    visual_props_.show_velocity_labels = show_velocity_labels_property_->getBool();
    visual_props_.show_stopping_point = show_stopping_point_property_->getBool();
    visual_props_.show_lane_boundary_safety_box = show_lane_boundary_safety_box_property_->getBool();

    QColor lb_color = lb_safety_box_color_property_->getColor();
    visual_props_.lb_safety_box_r = lb_color.redF();
    visual_props_.lb_safety_box_g = lb_color.greenF();
    visual_props_.lb_safety_box_b = lb_color.blueF();
    visual_props_.lb_safety_box_alpha = lb_safety_box_alpha_property_->getFloat();

    visual_props_.stopped_speed_limit = stopped_speed_limit_;

    // Swerving mode properties
    visual_props_.use_swerving = use_swerving_;
    visual_props_.narrow_safety_box_width = narrow_safety_box_width_;
    visual_props_.wide_safety_box_width = wide_safety_box_width_;
    visual_props_.distance_to_car_front = distance_to_car_front_;
}

void LocalPathDisplay::processMessage(const LocalPath::ConstPtr& msg)
{
    if (!visual_)
    {
        return;
    }

    // Get transform from message frame to fixed frame
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;

    if (!context_->getFrameManager()->getTransform(
            msg->header.frame_id, msg->header.stamp, position, orientation))
    {
        ROS_ERROR_THROTTLE(1.0, "LocalPathDisplay: Error transforming from frame '%s' to frame '%s'",
                  msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
        return;
    }

    // Update current pose in visual properties
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        visual_props_.current_x = current_x_;
        visual_props_.current_y = current_y_;
        visual_props_.current_z = current_z_;
        visual_props_.has_current_pose = has_current_pose_;
    }

    // Update visual
    visual_->setFramePosition(position);
    visual_->setFrameOrientation(orientation);
    visual_->setPath(*msg, visual_props_);
    visual_->setVisible(true);
}

}  // namespace autoware_mini

// Register this display plugin with pluginlib
PLUGINLIB_EXPORT_CLASS(autoware_mini::LocalPathDisplay, rviz::Display)
