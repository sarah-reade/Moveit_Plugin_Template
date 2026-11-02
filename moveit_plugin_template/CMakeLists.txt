cmake_minimum_required(VERSION 3.22)
project(moveit_plugin_template)

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(motion_planner_template REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/template_planning_context.cpp
  src/template_planner_manager.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  moveit_core
  motion_planner_template
  pluginlib
  rclcpp
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES template_planner_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_core motion_planner_template pluginlib rclcpp)
pluginlib_export_plugin_description_file(moveit_core template_planner_plugin.xml)
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})
ament_package()
