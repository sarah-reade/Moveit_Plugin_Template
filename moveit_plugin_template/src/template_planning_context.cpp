/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, MoveIt Plugin Template contributors.
 * See the package LICENSE file for the complete license text.
 */

#include <moveit_plugin_template/template_planning_context.hpp>

#include <chrono>
#include <rclcpp/rclcpp.hpp>

namespace moveit_plugin_template
{

TemplatePlanningContext::TemplatePlanningContext(const std::string& name, const std::string& group,
                                                 motion_planner_template::MotionPlannerPtr planner)
  : PlanningContext(name, group), planner_(std::move(planner))
{
}

bool TemplatePlanningContext::solve(planning_interface::MotionPlanResponse& response)
{
  response = planning_interface::MotionPlanResponse();
  const auto start = std::chrono::steady_clock::now();
  std::string error_message;
  if (!planner_ || !planner_->plan(getPlanningScene(), getMotionPlanRequest(), response.trajectory_, error_message))
  {
    RCLCPP_ERROR(rclcpp::get_logger("moveit_plugin_template"), "Planner failed: %s", error_message.c_str());
    response.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    response.planning_time_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    return false;
  }
  response.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  response.planning_time_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  return true;
}

bool TemplatePlanningContext::solve(planning_interface::MotionPlanDetailedResponse& response)
{
  response = planning_interface::MotionPlanDetailedResponse();
  const auto start = std::chrono::steady_clock::now();
  robot_trajectory::RobotTrajectoryPtr trajectory;
  std::string error_message;
  if (!planner_ || !planner_->plan(getPlanningScene(), getMotionPlanRequest(), trajectory, error_message))
  {
    RCLCPP_ERROR(rclcpp::get_logger("moveit_plugin_template"), "Planner failed: %s", error_message.c_str());
    response.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    response.processing_time_.push_back(
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
    return false;
  }
  response.trajectory_.push_back(std::move(trajectory));
  response.description_.push_back(planner_->getName());
  response.processing_time_.push_back(
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
  response.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return true;
}

bool TemplatePlanningContext::terminate()
{
  return true;
}

void TemplatePlanningContext::clear()
{
  // Planners are configured before the context is created, so no persistent solve state is retained here.
}

}  // namespace moveit_plugin_template
