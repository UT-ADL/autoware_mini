// Copyright (c) 2025 Autonomous Driving Lab, University of Tartu
// SPDX-License-Identifier: MIT

#include <cmath>
#include <cstdio>

#include <autoware_mini/rviz/detected_object_visual.h>

namespace autoware_mini
{

DetectedObjectVisual::DetectedObjectVisual(Ogre::SceneManager* scene_manager,
                                           Ogre::SceneNode* parent_node,
                                           uint32_t id)
    : scene_manager_(scene_manager)
    , id_(id)
{
    // Create scene node as child of parent
    scene_node_ = parent_node->createChildSceneNode();

    // Create 3D bounding box shape and 2D outline
    bbox_shape_ = std::make_unique<rviz::Shape>(rviz::Shape::Cube, scene_manager_, scene_node_);
    bbox_outline_ = std::make_unique<rviz::BillboardLine>(scene_manager_, scene_node_);
    hull_lines_ = std::make_unique<rviz::BillboardLine>(scene_manager_, scene_node_);

    // Create centroid sphere
    centroid_shape_ = std::make_unique<rviz::Shape>(rviz::Shape::Sphere, scene_manager_, scene_node_);

    // Create Arrow for velocity visualization (hidden and zero-sized by default)
    velocity_arrow_ = std::make_unique<rviz::Arrow>(scene_manager_, scene_node_);
    velocity_arrow_->set(0, 0, 0, 0);  // Zero dimensions
    velocity_arrow_->getSceneNode()->setVisible(false);

    // Create label node (separate from main scene node for text positioning)
    label_node_ = scene_node_->createChildSceneNode();
}

DetectedObjectVisual::~DetectedObjectVisual()
{
    label_text_.reset();
    scene_manager_->destroySceneNode(label_node_);
    scene_manager_->destroySceneNode(scene_node_);
}

void DetectedObjectVisual::setObject(const DetectedObject& obj, const DetectedObjectVisualProperties& props)
{
    renderBoundingBox3D(obj, props);
    renderBoundingBox2D(obj, props);
    renderConvexHull(obj, props);
    renderVelocityArrow(obj, props);
    renderLabel(obj, props);
    renderCentroid(obj, props);
}

void DetectedObjectVisual::renderBoundingBox3D(const DetectedObject& obj, const DetectedObjectVisualProperties& props)
{
    if (props.show_bbox_3d)
    {
        bbox_shape_->getRootNode()->setVisible(true);
        bbox_shape_->setPosition(Ogre::Vector3(obj.center.x, obj.center.y, obj.center.z));
        bbox_shape_->setOrientation(Ogre::Quaternion(Ogre::Radian(obj.heading), Ogre::Vector3::UNIT_Z));
        bbox_shape_->setScale(Ogre::Vector3(obj.dimensions.x, obj.dimensions.y, obj.dimensions.z));
        bbox_shape_->setColor(props.bbox_r, props.bbox_g, props.bbox_b, props.alpha * 0.3f);
    }
    else
    {
        bbox_shape_->getRootNode()->setVisible(false);
    }
}

void DetectedObjectVisual::renderBoundingBox2D(const DetectedObject& obj, const DetectedObjectVisualProperties& props)
{
    bbox_outline_->clear();

    if (!props.show_bbox_2d)
    {
        return;
    }

    float half_x = obj.dimensions.x / 2.0f;
    float half_y = obj.dimensions.y / 2.0f;
    float half_z = obj.dimensions.z / 2.0f;
    float cos_h = std::cos(obj.heading);
    float sin_h = std::sin(obj.heading);

    auto transform = [&](float lx, float ly) -> Ogre::Vector3 {
        return Ogre::Vector3(
            obj.center.x + lx * cos_h - ly * sin_h,
            obj.center.y + lx * sin_h + ly * cos_h,
            obj.center.z - half_z);
    };

    Ogre::Vector3 p0 = transform(-half_x, -half_y);
    Ogre::Vector3 p1 = transform(half_x, -half_y);
    Ogre::Vector3 p2 = transform(half_x, half_y);
    Ogre::Vector3 p3 = transform(-half_x, half_y);

    Ogre::Vector3 center = transform(0, 0);
    Ogre::Vector3 front = transform(half_x, 0);

    bbox_outline_->setLineWidth(props.line_width * 0.01f);
    bbox_outline_->setColor(props.bbox_r, props.bbox_g, props.bbox_b, props.alpha);
    bbox_outline_->setMaxPointsPerLine(7);
    bbox_outline_->setNumLines(1);

    bbox_outline_->addPoint(center);
    bbox_outline_->addPoint(front);
    bbox_outline_->addPoint(p2);
    bbox_outline_->addPoint(p3);
    bbox_outline_->addPoint(p0);
    bbox_outline_->addPoint(p1);
    bbox_outline_->addPoint(front);
}

void DetectedObjectVisual::renderConvexHull(const DetectedObject& obj, const DetectedObjectVisualProperties& props)
{
    hull_lines_->clear();

    if (!props.show_convex_hull || obj.convex_hull.empty())
    {
        return;
    }

    // Convex hull is stored as flat array of x, y, z triplets
    size_t num_points = obj.convex_hull.size() / 3;
    if (num_points < 2)
    {
        return;
    }

    // Set line properties
    hull_lines_->setLineWidth(props.line_width * 0.01f);
    hull_lines_->setColor(props.hull_r, props.hull_g, props.hull_b, props.alpha);

    // Draw as a closed loop (num_points + 1 points to close)
    hull_lines_->setMaxPointsPerLine(num_points + 1);
    hull_lines_->setNumLines(1);

    for (size_t i = 0; i < num_points; ++i)
    {
        float x = obj.convex_hull[i * 3];
        float y = obj.convex_hull[i * 3 + 1];
        float z = obj.convex_hull[i * 3 + 2];
        hull_lines_->addPoint(Ogre::Vector3(x, y, z));
    }

    // Close the loop
    hull_lines_->addPoint(Ogre::Vector3(obj.convex_hull[0], obj.convex_hull[1], obj.convex_hull[2]));
}

void DetectedObjectVisual::renderVelocityArrow(const DetectedObject& obj, const DetectedObjectVisualProperties& props)
{
    if (!props.show_velocity)
    {
        velocity_arrow_->getSceneNode()->setVisible(false);
        return;
    }

    float vx = obj.velocity.x;
    float vy = obj.velocity.y;
    float vz = obj.velocity.z;
    float speed = std::sqrt(vx * vx + vy * vy + vz * vz);

    if (speed < 0.01f)
    {
        velocity_arrow_->getSceneNode()->setVisible(false);
        return;
    }

    velocity_arrow_->getSceneNode()->setVisible(true);

    // Arrow position at centroid
    velocity_arrow_->setPosition(Ogre::Vector3(obj.centroid.x, obj.centroid.y, obj.centroid.z));

    // Arrow direction from velocity vector (normalized)
    Ogre::Vector3 direction(vx, vy, vz);
    direction.normalise();
    velocity_arrow_->setDirection(direction);

    // Set arrow dimensions based on speed
    // Parameters: shaft_length, shaft_diameter, head_length, head_diameter
    float shaft_length = std::max(speed - 0.3f, 0.01f);  // Leave room for head
    float shaft_diameter = 0.1f;
    float head_length = std::min(0.3f, speed * 0.5f);
    float head_diameter = 0.2f;

    velocity_arrow_->set(shaft_length, shaft_diameter, head_length, head_diameter);

    // Set color
    velocity_arrow_->setColor(props.vel_r, props.vel_g, props.vel_b, props.alpha);
}

void DetectedObjectVisual::renderLabel(const DetectedObject& obj, const DetectedObjectVisualProperties& props)
{
    if (!props.show_label)
    {
        if (label_text_)
        {
            label_text_->setVisible(false);
        }
        return;
    }

    // Calculate speed in km/h
    float vx = obj.velocity.x;
    float vy = obj.velocity.y;
    float vz = obj.velocity.z;
    float speed_ms = std::sqrt(vx * vx + vy * vy + vz * vz);
    int speed_kmh = static_cast<int>(speed_ms * 3.6f);

    // Build label text
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%s %u (%d km/h)", obj.label.c_str(), obj.id, speed_kmh);
    std::string text = buf;

    // Create or update label
    if (!label_text_)
    {
        label_text_ = std::make_unique<rviz::MovableText>(text);
        label_text_->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_CENTER);
        label_text_->setCharacterHeight(0.5f);
        label_node_->attachObject(label_text_.get());
    }
    else
    {
        label_text_->setCaption(text);
        label_text_->setVisible(true);
    }

    // Position label above centroid
    label_node_->setPosition(obj.centroid.x, obj.centroid.y, obj.centroid.z + 1.0f);

    // Set color
    label_text_->setColor(Ogre::ColourValue(props.label_r, props.label_g, props.label_b, props.alpha));
}

void DetectedObjectVisual::renderCentroid(const DetectedObject& obj, const DetectedObjectVisualProperties& props)
{
    if (!props.show_centroid)
    {
        centroid_shape_->getRootNode()->setVisible(false);
        return;
    }

    centroid_shape_->getRootNode()->setVisible(true);
    centroid_shape_->setPosition(Ogre::Vector3(obj.centroid.x, obj.centroid.y, obj.centroid.z));
    centroid_shape_->setScale(Ogre::Vector3(0.5f, 0.5f, 0.5f));
    centroid_shape_->setColor(obj.color.r, obj.color.g, obj.color.b, props.alpha);
}

void DetectedObjectVisual::setVisible(bool visible)
{
    // Don't cascade visibility to children - they manage their own visibility
    scene_node_->setVisible(visible, false);
}

void DetectedObjectVisual::setFramePosition(const Ogre::Vector3& position)
{
    scene_node_->setPosition(position);
}

void DetectedObjectVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
    scene_node_->setOrientation(orientation);
}

}  // namespace autoware_mini
