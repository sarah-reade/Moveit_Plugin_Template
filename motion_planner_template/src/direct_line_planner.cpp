/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, MoveIt Plugin Template contributors.
 * See the package LICENSE file for the complete license text.
 */

#include <motion_planner_template/direct_line_planner.hpp>

#include <cmath>
#include <stdexcept>

namespace motion_planner_template
{

DirectLinePlanner::DirectLinePlanner(moveit::core::RobotModelConstPtr robot_model) : MotionPlanner(std::move(robot_model))
{
}

std::string DirectLinePlanner::getName() const
{
  return "DirectLinePlanner";
}

bool DirectLinePlanner::configure(const Parameters& parameters, std::string& error_message)
{
  const auto iterator = parameters.find("num_steps");
  if (iterator == parameters.end())
    return true;

  try
  {
    const auto value = std::stoul(iterator->second);
    if (value == 0)
    {
      error_message = "num_steps must be greater than zero.";
      return false;
    }
    num_steps_ = value;
  }
  catch (const std::invalid_argument&)
  {
    error_message = "num_steps must be a positive integer.";
    return false;
  }
  catch (const std::out_of_range&)
  {
    error_message = "num_steps must be a positive integer.";
    return false;
  }
  return true;
}

bool DirectLinePlanner::plan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                             const planning_interface::MotionPlanRequest& request,
                             robot_trajectory::RobotTrajectoryPtr& trajectory,
                             std::string& error_message) const
{
  moveit::core::RobotState start_state(robot_model_);
  moveit::core::RobotState goal_state(robot_model_);
  if (!makeStartAndGoalStates(planning_scene, request, start_state, goal_state, error_message))
    return false;

  trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, request.group_name);
  const auto* group = robot_model_->getJointModelGroup(request.group_name);
  for (std::size_t step = 0; step <= num_steps_; ++step)
  {
    auto waypoint = std::make_shared<moveit::core::RobotState>(robot_model_);
    start_state.interpolate(goal_state, static_cast<double>(step) / num_steps_, *waypoint, group);
    waypoint->update();
    trajectory->addSuffixWayPoint(waypoint, step == 0 ? 0.0 : 0.1);
  }

  if (!validateTrajectory(planning_scene, request, *trajectory, error_message))
  {
    trajectory.reset();
    return false;
  }
  return true;
}

}  // namespace motion_planner_template
