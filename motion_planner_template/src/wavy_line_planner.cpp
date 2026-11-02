/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, MoveIt Plugin Template contributors.
 * See the package LICENSE file for the complete license text.
 */

#include <motion_planner_template/wavy_line_planner.hpp>

#include <cmath>
#include <stdexcept>

namespace motion_planner_template
{

WavyLinePlanner::WavyLinePlanner(moveit::core::RobotModelConstPtr robot_model) : MotionPlanner(std::move(robot_model))
{
}

std::string WavyLinePlanner::getName() const
{
  return "WavyLinePlanner";
}

bool WavyLinePlanner::configure(const Parameters& parameters, std::string& error_message)
{
  try
  {
    if (const auto iterator = parameters.find("num_steps"); iterator != parameters.end())
      num_steps_ = std::stoul(iterator->second);
    if (const auto iterator = parameters.find("amplitude"); iterator != parameters.end())
      amplitude_ = std::stod(iterator->second);
    if (const auto iterator = parameters.find("frequency"); iterator != parameters.end())
      frequency_ = std::stod(iterator->second);
  }
  catch (const std::invalid_argument&)
  {
    error_message = "WavyLinePlanner parameters must be numeric.";
    return false;
  }
  catch (const std::out_of_range&)
  {
    error_message = "WavyLinePlanner parameters must be numeric.";
    return false;
  }
  if (num_steps_ == 0 || frequency_ < 0.0)
  {
    error_message = "num_steps must be greater than zero and frequency cannot be negative.";
    return false;
  }
  return true;
}

bool WavyLinePlanner::plan(const planning_scene::PlanningSceneConstPtr& planning_scene,
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
  const auto& variables = group->getVariableNames();
  for (std::size_t step = 0; step <= num_steps_; ++step)
  {
    const double progress = static_cast<double>(step) / num_steps_;
    auto waypoint = std::make_shared<moveit::core::RobotState>(robot_model_);
    start_state.interpolate(goal_state, progress, *waypoint, group);

    // The sin term is zero at both endpoints, so the exact start and goal are preserved.
    // To Change: Implement your own trajectory generation algorithm here.
    if (step != 0 && step != num_steps_)
    {
      const double offset = amplitude_ * std::sin(2.0 * M_PI * frequency_ * progress);
      for (const auto& variable : variables)
        waypoint->setVariablePosition(variable, waypoint->getVariablePosition(variable) + offset);
    }
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
