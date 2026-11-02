/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, MoveIt Plugin Template contributors.
 * See the package LICENSE file for the complete license text.
 */

#include <moveit_plugin_template/template_planner_manager.hpp>
#include <moveit_plugin_template/template_planning_context.hpp>

#include <motion_planner_template/direct_line_planner.hpp>
#include <motion_planner_template/wavy_line_planner.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace moveit_plugin_template
{

bool TemplatePlannerManager::initialize(const moveit::core::RobotModelConstPtr& model,
                                        const rclcpp::Node::SharedPtr&, const std::string&)
{
  robot_model_ = model;
  return static_cast<bool>(robot_model_);
}

std::string TemplatePlannerManager::getDescription() const
{
  return "MoveIt 2 motion planner template";
}

void TemplatePlannerManager::getPlanningAlgorithms(std::vector<std::string>& algorithms) const
{
  algorithms = {"DirectLinePlanner", "WavyLinePlanner"};
  // To Change: Add your new planner class to this list.
  // Search pattern: DirectLinePlanner, WavyLinePlanner
}

const planning_interface::PlannerConfigurationSettings* TemplatePlannerManager::findConfiguration(
    const std::string& planner_id, const std::string& group) const
{
  const std::string group_specific_name = group + "[" + planner_id + "]";
  for (const auto& name : {planner_id, group_specific_name, group})
  {
    const auto iterator = config_settings_.find(name);
    if (iterator != config_settings_.end())
      return &iterator->second;
  }
  return nullptr;
}

motion_planner_template::MotionPlannerPtr TemplatePlannerManager::makePlanner(const std::string& planner_id,
                                                                                const std::string& group,
                                                                                std::string& error_message) const
{
  motion_planner_template::MotionPlannerPtr planner;
  if (planner_id == "DirectLinePlanner" || planner_id.empty())
    planner = std::make_shared<motion_planner_template::DirectLinePlanner>(robot_model_);
  else if (planner_id == "WavyLinePlanner")
    planner = std::make_shared<motion_planner_template::WavyLinePlanner>(robot_model_);
  else
  {
    error_message = "Unknown planner_id: " + planner_id;
    return nullptr;
  }

  if (const auto* configuration = findConfiguration(planner_id.empty() ? "DirectLinePlanner" : planner_id, group))
  {
    if (!planner->configure(configuration->config, error_message))
      return nullptr;
  }
  return planner;
  // To Change: Register your additional motion planner class here.
  // Search pattern: DirectLinePlanner, WavyLinePlanner
}

planning_interface::PlanningContextPtr TemplatePlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene, const planning_interface::MotionPlanRequest& request,
    moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  if (!robot_model_ || !planning_scene || !robot_model_->getJointModelGroup(request.group_name))
    return nullptr;

  std::string error_message;
  auto planner = makePlanner(request.planner_id, request.group_name, error_message);
  if (!planner)
    return nullptr;

  auto context = std::make_shared<TemplatePlanningContext>(
      request.planner_id.empty() ? "DirectLinePlanner" : request.planner_id, request.group_name, std::move(planner));
  context->setPlanningScene(planning_scene);
  context->setMotionPlanRequest(request);
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return context;
}

bool TemplatePlannerManager::canServiceRequest(const planning_interface::MotionPlanRequest& request) const
{
  if (!robot_model_ || !robot_model_->getJointModelGroup(request.group_name))
    return false;
  return request.planner_id.empty() || request.planner_id == "DirectLinePlanner" ||
         request.planner_id == "WavyLinePlanner";
}

void TemplatePlannerManager::setPlannerConfigurations(
    const planning_interface::PlannerConfigurationMap& configurations)
{
  config_settings_ = configurations;
}

}  // namespace moveit_plugin_template

PLUGINLIB_EXPORT_CLASS(moveit_plugin_template::TemplatePlannerManager, planning_interface::PlannerManager)
