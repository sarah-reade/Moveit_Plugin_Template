/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, MoveIt Plugin Template contributors.
 * See the package LICENSE file for the complete license text.
 */

#include <motion_planner_template/motion_planner.hpp>

#include <moveit/robot_state/conversions.h>

#include <sstream>

namespace motion_planner_template
{

MotionPlanner::MotionPlanner(moveit::core::RobotModelConstPtr robot_model) : robot_model_(std::move(robot_model))
{
}

bool MotionPlanner::configure(const Parameters&, std::string&)
{
  return true;
}

bool MotionPlanner::makeStartAndGoalStates(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                           const planning_interface::MotionPlanRequest& request,
                                           moveit::core::RobotState& start_state,
                                           moveit::core::RobotState& goal_state,
                                           std::string& error_message) const
{
  if (!planning_scene || !robot_model_)
  {
    error_message = "A planning scene and robot model are required.";
    return false;
  }

  const auto* joint_model_group = robot_model_->getJointModelGroup(request.group_name);
  if (!joint_model_group)
  {
    error_message = "Unknown planning group: " + request.group_name;
    return false;
  }

  start_state = planning_scene->getCurrentState();
  if (!request.start_state.joint_state.name.empty() &&
      !moveit::core::robotStateMsgToRobotState(request.start_state, start_state))
  {
    error_message = "The request start state could not be applied to the robot model.";
    return false;
  }
  start_state.update();
  goal_state = start_state;

  if (request.goal_constraints.empty())
  {
    error_message = "At least one goal constraint is required.";
    return false;
  }

  // This template uses joint goals so the trajectory algorithm stays easy to replace.
  // To Change: Extend goal extraction here if your planner supports pose or other goals.
  for (const auto& joint_constraint : request.goal_constraints.front().joint_constraints)
  {
    if (!robot_model_->getJointModel(joint_constraint.joint_name))
    {
      error_message = "Unknown joint in goal constraint: " + joint_constraint.joint_name;
      return false;
    }
    goal_state.setVariablePosition(joint_constraint.joint_name, joint_constraint.position);
  }
  goal_state.update();

  if (!goal_state.satisfiesBounds(joint_model_group))
  {
    error_message = "The requested goal is outside the joint bounds.";
    return false;
  }
  return true;
}

bool MotionPlanner::validateTrajectory(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                       const planning_interface::MotionPlanRequest& request,
                                       const robot_trajectory::RobotTrajectory& trajectory,
                                       std::string& error_message) const
{
  if (trajectory.getWayPointCount() < 2)
  {
    error_message = "A trajectory must contain at least a start and goal waypoint.";
    return false;
  }

  std::vector<std::size_t> invalid_indices;
  if (!planning_scene->isPathValid(trajectory, request.path_constraints, request.goal_constraints,
                                   request.group_name, false, &invalid_indices))
  {
    std::ostringstream stream;
    stream << "The generated trajectory is invalid";
    if (!invalid_indices.empty())
      stream << " at waypoint " << invalid_indices.front();
    stream << ".";
    error_message = stream.str();
    return false;
  }
  return true;
}

}  // namespace motion_planner_template
