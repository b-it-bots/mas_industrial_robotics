# mir_planning_core

## Dependencies

- `mir_planning_msgs`
- `mir_states`
- `mir_pddl_problem_generator`
- `mir_task_planning`
- `mir_planner_executor`
- Optionally (with real robot)
  - `mir_actions`

## Setup

This is not a standalone package and thus it should not be running without its
dependencies.

To test with mockup action servers without a robot
```
roscore
roslaunch mir_planning_core task_planning_components.launch
rosrun mir_task_executor task_executor_mockup
roslaunch mir_task_planning upload_problem.launch
roslaunch mir_planning_core task_planning_sm.launch
```


