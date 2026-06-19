// Copyright (c) 2026 Autonomous Driving Lab, University of Tartu
// SPDX-License-Identifier: MIT

#ifndef AUTOWARE_MINI_RVIZ_HEADING_ALIGNED_ORTHO_VIEW_CONTROLLER_H
#define AUTOWARE_MINI_RVIZ_HEADING_ALIGNED_ORTHO_VIEW_CONTROLLER_H

#ifndef Q_MOC_RUN
#include <rviz/default_plugin/view_controllers/fixed_orientation_ortho_view_controller.h>
#endif

namespace autoware_mini
{

class HeadingAlignedOrthoViewController : public rviz::FixedOrientationOrthoViewController
{
  Q_OBJECT
public:
  HeadingAlignedOrthoViewController() = default;
  ~HeadingAlignedOrthoViewController() override = default;

protected:
  void updateTargetSceneNode() override;
};

} // namespace autoware_mini

#endif // AUTOWARE_MINI_RVIZ_HEADING_ALIGNED_ORTHO_VIEW_CONTROLLER_H
