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

class TemplatePlannerManager final : public planning_interface::PlannerManager
{
public:
  bool initialize(const moveit::core::RobotModelConstPtr& model, const rclcpp::Node::SharedPtr& node,
                  const std::string& parameter_namespace) override;
  std::string getDescription() const override;
  void getPlanningAlgorithms(std::vector<std::string>& algorithms) const override;
  planning_interface::PlanningContextPtr getPlanningContext(
      const planning_scene::PlanningSceneConstPtr& planning_scene, const planning_interface::MotionPlanRequest& request,
      moveit_msgs::msg::MoveItErrorCodes& error_code) const override;
  bool canServiceRequest(const planning_interface::MotionPlanRequest& request) const override;
  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& configurations) override;

private:
  motion_planner_template::MotionPlannerPtr makePlanner(const std::string& planner_id,
                                                        const std::string& group,
                                                        std::string& error_message) const;
  const planning_interface::PlannerConfigurationSettings* findConfiguration(const std::string& planner_id,
                                                                            const std::string& group) const;

  moveit::core::RobotModelConstPtr robot_model_;
};

}  // namespace moveit_plugin_template
