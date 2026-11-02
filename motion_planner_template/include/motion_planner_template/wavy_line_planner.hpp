/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, MoveIt Plugin Template contributors.
 * See the package LICENSE file for the complete license text.
 */

#pragma once

#include <motion_planner_template/motion_planner.hpp>

namespace motion_planner_template
{

class WavyLinePlanner final : public MotionPlanner
{
public:
  explicit WavyLinePlanner(moveit::core::RobotModelConstPtr robot_model);

  std::string getName() const override;
  bool configure(const Parameters& parameters, std::string& error_message) override;
  bool plan(const planning_scene::PlanningSceneConstPtr& planning_scene,
            const planning_interface::MotionPlanRequest& request,
            robot_trajectory::RobotTrajectoryPtr& trajectory,
            std::string& error_message) const override;

private:
  std::size_t num_steps_{100};
  double amplitude_{0.05};
  double frequency_{1.0};
};

}  // namespace motion_planner_template
