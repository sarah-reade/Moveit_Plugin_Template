cmake_minimum_required(VERSION 3.22)
project(moveit_planners_ompl)

# Common CMake code applied to all MoveIt packages
find_package(moveit_common REQUIRED)
moveit_package()

# Dependencies
find_package(Boost REQUIRED system filesystem date_time thread serialization)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenMP REQUIRED)
find_package(motion_planner_template REQUIRED)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wno-overloaded-virtual)
endif()

include_directories(SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

set(MOVEIT_LIB_NAME moveit_template_plugin)

# OMPL interface library
add_library(${MOVEIT_LIB_NAME} SHARED
  src/template_interface.cpp
  src/planning_context_manager.cpp
  src/model_based_planning_context.cpp
  src/parameterization/model_based_state_space.cpp
  src/parameterization/model_based_state_space_factory.cpp
  src/parameterization/joint_space/constrained_planning_state_space.cpp
  src/parameterization/joint_space/constrained_planning_state_space_factory.cpp
  src/parameterization/joint_space/joint_model_state_space.cpp
  src/parameterization/joint_space/joint_model_state_space_factory.cpp
  src/parameterization/work_space/pose_model_state_space.cpp
  src/parameterization/work_space/pose_model_state_space_factory.cpp
  src/detail/ompl_constraints.cpp
  src/detail/threadsafe_state_storage.cpp
  src/detail/state_validity_checker.cpp
  src/detail/projection_evaluators.cpp
  src/detail/goal_union.cpp
  src/detail/constraints_library.cpp
  src/detail/constrained_sampler.cpp
  src/detail/constrained_valid_state_sampler.cpp
  src/detail/constrained_goal_sampler.cpp
)

set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES
  VERSION "${${PROJECT_NAME}_VERSION}"
)

# Used to link in ODE, an OMPL dependency, on macOS
if(APPLE)
  target_link_directories(${MOVEIT_LIB_NAME} PUBLIC ${OMPL_LIBRARY_DIRS})
endif()

ament_target_dependencies(${MOVEIT_LIB_NAME}
  moveit_core
  moveit_msgs
  moveit_ros_planning
  rclcpp
  pluginlib
  tf2_eigen
  tf2_ros
  motion_planner_template
  Boost
)

set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES
  COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}"
  LINK_FLAGS "${OpenMP_CXX_FLAGS}"
)

# State database generator
add_executable(moveit_generate_state_database
  scripts/generate_state_database.cpp
)

target_link_libraries(moveit_generate_state_database
  ${MOVEIT_LIB_NAME}
)

set_target_properties(moveit_generate_state_database PROPERTIES
  LINK_FLAGS "${OpenMP_CXX_FLAGS}"
  OUTPUT_NAME "generate_state_database"
)

# OMPL planner plugin
add_library(moveit_ompl_planner_plugin SHARED
  src/template_planner_manager.cpp
)

set_target_properties(moveit_ompl_planner_plugin PROPERTIES
  VERSION "${${PROJECT_NAME}_VERSION}"
)

ament_target_dependencies(moveit_ompl_planner_plugin
  moveit_core
  moveit_ros_planning
  rclcpp
  pluginlib
  tf2_ros
  OMPL
  Boost
)

target_link_libraries(moveit_ompl_planner_plugin
  ${MOVEIT_LIB_NAME}
)

# Install targets
install(TARGETS
  ${MOVEIT_LIB_NAME}
  moveit_ompl_planner_plugin
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS moveit_generate_state_database
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_core ompl)

pluginlib_export_plugin_description_file(
  moveit_core template_interface_plugin_description.xml
)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Eigen3 REQUIRED)

  # These lint checks don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_state_space
    test/test_state_space.cpp
  )
  ament_target_dependencies(test_state_space moveit_core OMPL Boost Eigen3)
  target_link_libraries(test_state_space ${MOVEIT_LIB_NAME})
  set_target_properties(test_state_space PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")

  ament_add_gtest(test_state_validity_checker
    test/test_state_validity_checker.cpp
  )
  ament_target_dependencies(test_state_validity_checker moveit_core OMPL Boost Eigen3)
  target_link_libraries(test_state_validity_checker ${MOVEIT_LIB_NAME})
  set_target_properties(test_state_validity_checker PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")

  ament_add_gtest(test_planning_context_manager
    test/test_planning_context_manager.cpp
  )
  ament_target_dependencies(test_planning_context_manager moveit_core tf2_eigen OMPL Boost Eigen3)
  target_link_libraries(test_planning_context_manager ${MOVEIT_LIB_NAME})

  # Disabling flaky test
  # TODO (vatanaksoytezer): Uncomment once this is fixed
  # ament_add_gtest(test_ompl_constraints
  #   test/test_ompl_constraints.cpp
  # )
  # ament_target_dependencies(test_ompl_constraints moveit_core OMPL Boost Eigen3)
  # target_link_libraries(test_ompl_constraints ${MOVEIT_LIB_NAME})

  ament_add_gtest(test_constrained_planning_state_space
    test/test_constrained_planning_state_space.cpp
  )
  ament_target_dependencies(test_constrained_planning_state_space moveit_core OMPL Boost Eigen3)
  target_link_libraries(test_constrained_planning_state_space ${MOVEIT_LIB_NAME})
  set_target_properties(test_constrained_planning_state_space PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")

  ament_add_gtest(test_constrained_state_validity_checker
    test/test_constrained_state_validity_checker.cpp
  )
  ament_target_dependencies(test_constrained_state_validity_checker moveit_core OMPL Boost Eigen3)
  target_link_libraries(test_constrained_state_validity_checker ${MOVEIT_LIB_NAME})
  set_target_properties(test_constrained_state_validity_checker PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")

  ament_add_gtest(test_threadsafe_state_storage
    test/test_threadsafe_state_storage.cpp
  )
  ament_target_dependencies(test_threadsafe_state_storage moveit_core OMPL Boost Eigen3)
  target_link_libraries(test_threadsafe_state_storage ${MOVEIT_LIB_NAME})
  set_target_properties(test_threadsafe_state_storage PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")
endif()

ament_package()
