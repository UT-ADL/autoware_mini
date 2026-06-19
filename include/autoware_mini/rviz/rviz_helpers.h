// Copyright (c) 2025 Autonomous Driving Lab, University of Tartu
// SPDX-License-Identifier: MIT

#ifndef AUTOWARE_MINI_RVIZ_HELPERS_H
#define AUTOWARE_MINI_RVIZ_HELPERS_H

#include <cmath>
#include <sstream>
#include <vector>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>

namespace autoware_mini
{

/**
 * @brief Triangulate a polyline into a ribbon as triangle strip vertices
 *
 * Outputs vertices in triangle strip order (L0, R0, L1, R1, ...) for the
 * specified width around the centerline. When appending to a non-empty result,
 * inserts degenerate triangles to separate from the previous strip.
 *
 * @param centerline Input 3D polyline points
 * @param width Buffer width (full width)
 * @param result Output vector to append strip vertices to
 */
inline void triangulateRibbon(const std::vector<Ogre::Vector3>& centerline, float width,
                              std::vector<Ogre::Vector3>& result)
{
    if (centerline.size() < 2)
    {
        return;
    }

    float half_width = width / 2.0f;

    // Build left and right offset points
    std::vector<Ogre::Vector3> left_side;
    std::vector<Ogre::Vector3> right_side;

    for (size_t i = 0; i < centerline.size(); ++i)
    {
        // Compute 2D direction at this point
        Ogre::Vector3 dir;

        if (i == 0)
        {
            dir = centerline[1] - centerline[0];
        }
        else if (i == centerline.size() - 1)
        {
            dir = centerline[i] - centerline[i - 1];
        }
        else
        {
            Ogre::Vector3 d1 = centerline[i] - centerline[i - 1];
            Ogre::Vector3 d2 = centerline[i + 1] - centerline[i];
            d1.z = 0;
            d2.z = 0;
            if (d1.squaredLength() > 1e-9f) d1.normalise();
            if (d2.squaredLength() > 1e-9f) d2.normalise();

            dir = d1 + d2;

            // If directions cancel out (e.g., 180 degree turn), use outgoing direction
            if (dir.squaredLength() < 1e-9f)
            {
                dir = d2;
            }
        }

        // Normalize 2D direction
        dir.z = 0;
        if (dir.squaredLength() < 1e-9f) continue;
        dir.normalise();

        // Perpendicular: (-dy, dx) is left
        Ogre::Vector3 perp(-dir.y, dir.x, 0);

        left_side.push_back(centerline[i] + perp * half_width);
        right_side.push_back(centerline[i] - perp * half_width);
    }

    if (left_side.size() < 2)
    {
        return;
    }

    // Insert degenerate triangles to separate from previous strip
    if (!result.empty())
    {
        result.push_back(result.back());
        result.push_back(left_side[0]);
    }

    // Output strip vertices: L0, R0, L1, R1, ...
    for (size_t i = 0; i < left_side.size(); ++i)
    {
        result.push_back(left_side[i]);
        result.push_back(right_side[i]);
    }
}

/**
 * @brief Find the segment index and interpolation parameter for a point at a given distance along a polyline
 *
 * Walks along the polyline accumulating 2D segment lengths. Returns the segment index and
 * interpolation parameter t such that the point is at centerline[index-1] + t * (centerline[index] - centerline[index-1]).
 *
 * @param centerline Input 3D polyline points (must not be empty)
 * @param distance Distance along the polyline
 * @param out_index Output segment endpoint index (1-based, i.e. the point at the end of the segment)
 * @param out_t Output interpolation parameter [0, 1] within the segment
 * @return true if distance falls within the polyline, false if distance exceeds total length
 */
inline bool findSegmentAtDistance(const std::vector<Ogre::Vector3>& centerline,
                                  float distance, size_t& out_index, float& out_t)
{
    float accumulated = 0.0f;

    for (size_t i = 1; i < centerline.size(); ++i)
    {
        Ogre::Vector3 d = centerline[i] - centerline[i - 1];
        float seg_len = d.length();

        if (accumulated + seg_len >= distance)
        {
            out_index = i;
            out_t = std::max(0.0f, std::min(1.0f, (distance - accumulated) / seg_len));
            return true;
        }

        accumulated += seg_len;
    }

    // Distance exceeds total length
    out_index = centerline.size() - 1;
    out_t = 1.0f;
    return false;
}

/**
 * @brief Split a centerline polyline at a given distance, interpolating the split point
 *
 * @param centerline Input 3D polyline points
 * @param distance Distance along the polyline at which to split
 * @param first Output points from start up to and including the interpolated split point
 * @param second Output points from the interpolated split point to end
 */
inline void splitCenterlineAtDistance(const std::vector<Ogre::Vector3>& centerline,
                                      float distance,
                                      std::vector<Ogre::Vector3>& first,
                                      std::vector<Ogre::Vector3>& second)
{
    first.clear();
    second.clear();

    if (centerline.empty())
    {
        return;
    }

    if (distance <= 0.0f)
    {
        second = centerline;
        return;
    }

    size_t index;
    float t;

    if (!findSegmentAtDistance(centerline, distance, index, t))
    {
        // Distance exceeds total length
        first = centerline;
        return;
    }

    Ogre::Vector3 split_pt = centerline[index - 1] + (centerline[index] - centerline[index - 1]) * t;

    // First part: start to split point (including points up to index-1)
    first.insert(first.end(), centerline.begin(), centerline.begin() + index);
    first.push_back(split_pt);

    // Second part: split point to end
    second.push_back(split_pt);
    second.insert(second.end(), centerline.begin() + index, centerline.end());
}

/**
 * @brief Interpolate a point and heading at a given distance along a polyline
 *
 * @param centerline Input 3D polyline points
 * @param distance Distance along the polyline in meters
 * @param out_point Output interpolated point
 * @param out_heading Output heading angle (radians)
 * @return true if point was computed successfully, false if centerline is empty
 */
inline bool getPointAtDistance(const std::vector<Ogre::Vector3>& centerline, float distance,
                                Ogre::Vector3& out_point, float& out_heading)
{
    if (centerline.empty())
    {
        return false;
    }

    if (centerline.size() == 1)
    {
        out_point = centerline[0];
        out_heading = 0.0f;
        return true;
    }

    size_t index;
    float t;

    if (distance <= 0.0f)
    {
        index = 1;
        t = 0.0f;
    }
    else if (!findSegmentAtDistance(centerline, distance, index, t))
    {
        // Distance exceeds path length - clamp to last point
        index = centerline.size() - 1;
        t = 1.0f;
    }

    Ogre::Vector3 d = centerline[index] - centerline[index - 1];
    out_point = centerline[index - 1] + d * t;
    out_heading = std::atan2(d.y, d.x);
    return true;
}

/**
 * @brief Create a dynamic ManualObject with a unique name and attach to scene node
 *
 * @param scene_manager OGRE scene manager
 * @param scene_node Scene node to attach the object to
 * @param name_prefix Prefix for the unique name (e.g., "LocalPathVisual")
 * @param unique_id Integer used to generate unique name suffix
 * @return Pointer to the created ManualObject
 */
inline Ogre::ManualObject* createDynamicManualObject(
    Ogre::SceneManager* scene_manager,
    Ogre::SceneNode* scene_node,
    const std::string& name_prefix,
    uintptr_t unique_id)
{
    std::ostringstream name;
    name << name_prefix << "_" << unique_id;
    auto* obj = scene_manager->createManualObject(name.str());
    obj->setDynamic(true);
    scene_node->attachObject(obj);
    return obj;
}

/**
 * @brief Convenience overload for using 'this' pointer as unique identifier
 */
inline Ogre::ManualObject* createDynamicManualObject(
    Ogre::SceneManager* scene_manager,
    Ogre::SceneNode* scene_node,
    const std::string& name_prefix,
    const void* unique_ptr)
{
    return createDynamicManualObject(scene_manager, scene_node, name_prefix,
                                     reinterpret_cast<uintptr_t>(unique_ptr));
}

/**
 * @brief Get the shared material for overlay rendering with ManualObject
 *
 * Creates a material on first call with settings suitable for rendering
 * transparent geometry with vertex colors:
 * - No shadows
 * - No lighting (vertex colors appear as specified)
 * - Transparent alpha blending
 * - No depth writing (proper transparency ordering)
 * - No backface culling (visible from both sides)
 *
 * @return Material name to use with ManualObject::begin()
 */
inline const std::string& getTransparentMaterial()
{
    static const std::string material_name = "TransparentMaterial";
    static bool initialized = false;

    if (!initialized)
    {
        auto material = Ogre::MaterialManager::getSingleton().create(
            material_name,
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

        material->setReceiveShadows(false);
        material->getTechnique(0)->setLightingEnabled(false);
        material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
        material->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        material->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);

        initialized = true;
    }

    return material_name;
}

/**
 * @brief Get the shared material for opaque rendering with ManualObject
 *
 * Creates a material on first call with settings suitable for rendering
 * solid opaque geometry with vertex colors:
 * - No shadows
 * - No lighting (vertex colors appear as specified)
 * - Depth writing enabled (proper depth ordering)
 * - No backface culling (visible from both sides)
 *
 * @return Material name to use with ManualObject::begin()
 */
inline const std::string& getOpaqueMaterial()
{
    static const std::string material_name = "OpaqueMaterial";
    static bool initialized = false;

    if (!initialized)
    {
        auto material = Ogre::MaterialManager::getSingleton().create(
            material_name,
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

        material->setReceiveShadows(false);
        material->getTechnique(0)->setLightingEnabled(false);
        material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(true);
        material->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_REPLACE);
        material->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);

        initialized = true;
    }

    return material_name;
}

/**
 * @brief Render triangle strip to a ManualObject with uniform color
 *
 * @param obj ManualObject to render to (should be cleared by caller)
 * @param vertices Vector of triangle vertices
 * @param r Red color component (0-1)
 * @param g Green color component (0-1)
 * @param b Blue color component (0-1)
 * @param alpha Alpha component (0-1)
 * @param overlay If true, use transparent overlay material; if false, use opaque material
 */
inline void renderTriangleStrip(Ogre::ManualObject* obj,
                           const std::vector<Ogre::Vector3>& vertices,
                           float r, float g, float b, float alpha,
                           bool overlay = true)
{
    if (vertices.empty())
    {
        return;
    }

    obj->begin(overlay ? getTransparentMaterial() : getOpaqueMaterial(), Ogre::RenderOperation::OT_TRIANGLE_STRIP);

    for (const auto& pt : vertices)
    {
        obj->position(pt);
        obj->colour(r, g, b, alpha);
    }

    obj->end();
}

}  // namespace autoware_mini

#endif  // AUTOWARE_MINI_RVIZ_HELPERS_H
