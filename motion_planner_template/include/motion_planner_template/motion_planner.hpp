/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, MoveIt Plugin Template contributors.
 * See the package LICENSE file for the complete license text.
 */

#pragma once

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>

#include <map>
#include <memory>
#include <string>

namespace motion_planner_template
{

class MotionPlanner
{
public:
  using Parameters = std::map<std::string, std::string>;

  explicit MotionPlanner(moveit::core::RobotModelConstPtr robot_model);
  virtual ~MotionPlanner() = default;

  virtual std::string getName() const = 0;

  // To Change: Add planner-specific configuration parameters here.
  // To Change: Extend this method when your planner needs additional settings.
  virtual bool configure(const Parameters& parameters, std::string& error_message);

  virtual bool plan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& request,
                    robot_trajectory::RobotTrajectoryPtr& trajectory,
                    std::string& error_message) const = 0;

protected:
  bool makeStartAndGoalStates(const planning_scene::PlanningSceneConstPtr& planning_scene,
                              const planning_interface::MotionPlanRequest& request,
                              moveit::core::RobotState& start_state, moveit::core::RobotState& goal_state,
                              std::string& error_message) const;

  bool validateTrajectory(const planning_scene::PlanningSceneConstPtr& planning_scene,
                          const planning_interface::MotionPlanRequest& request,
                          const robot_trajectory::RobotTrajectory& trajectory,
                          std::string& error_message) const;

  moveit::core::RobotModelConstPtr robot_model_;
};

using MotionPlannerPtr = std::shared_ptr<MotionPlanner>;

}  // namespace motion_planner_template
