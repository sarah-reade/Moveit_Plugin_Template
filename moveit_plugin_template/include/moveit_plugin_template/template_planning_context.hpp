/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, MoveIt Plugin Template contributors.
 * See the package LICENSE file for the complete license text.
 */

#pragma once

#include <motion_planner_template/motion_planner.hpp>
#include <moveit/planning_interface/planning_interface.h>

namespace moveit_plugin_template
{

class TemplatePlanningContext final : public planning_interface::PlanningContext
{
public:
  TemplatePlanningContext(const std::string& name, const std::string& group,
                          motion_planner_template::MotionPlannerPtr planner);

  bool solve(planning_interface::MotionPlanResponse& response) override;
  bool solve(planning_interface::MotionPlanDetailedResponse& response) override;
  bool terminate() override;
  void clear() override;

private:
  motion_planner_template::MotionPlannerPtr planner_;
};

}  // namespace moveit_plugin_template
