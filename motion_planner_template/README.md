# motion_planner_template

This ROS 2 Humble package contains two small, independent motion-planning
algorithms used by the companion [`moveit_plugin_template`](../moveit_plugin_template)
package. It is intentionally straightforward: a new algorithm can be added by
deriving from `MotionPlanner` and implementing `plan()`.

## Included planners

* **DirectLinePlanner** interpolates the start and goal `RobotState` in the
  selected joint group.
* **WavyLinePlanner** uses the same interpolation and adds
  `amplitude * sin(2*pi*frequency*t)` to each group variable for interior
  points. The sine term is zero at `t = 0` and `t = 1`, so endpoints remain
  exact.

Every generated waypoint is checked with `PlanningScene::isPathValid`, which
checks bounds, collisions, path constraints, and goal constraints.

## Adding a planner

1. Add `my_planner.hpp` and `my_planner.cpp`, deriving from `MotionPlanner`.
2. Implement `getName()`, `configure()`, and `plan()`.
3. Add the source file to `CMakeLists.txt`.
4. Register the class in the manager package at the marked
   `DirectLinePlanner, WavyLinePlanner` search points.

Planner-specific settings are received as string key/value pairs through
`configure()`. Add their names to the planner's implementation and expose
them in the MoveIt planning configuration.

## Build

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to moveit_plugin_template motion_planner_template \
  --event-handlers console_direct+
```

The package depends on `moveit_core`, `moveit_msgs`, and `rclcpp`.

## License and attribution

The original template code is BSD 3-Clause licensed; see [LICENSE](LICENSE).
The implementation uses public MoveIt 2 APIs and is not copied from OMPL or
CHOMP. MoveIt 2 and its dependencies retain their own copyrights and licenses.
