# mir_task_planning

Generate task plan using classical planner.

This module provides ros independent wrapping for planners (currently `mercury`
and `lama`) in `/common/planner_wrapper/`.
This ros wrapper on top of the previously mentioned module is available in
`/ros` folder. The ros wrapper is implemented as an `ActionServer`.

Upon request from a `ActionClient` containing `domain_file`, `problem_file`, `planner`
 and `mode`, the server tries to
plan with those provided configuration and returns a `CompletePlan`.

- `domain_file` - file path of domain file (it should be a `.pddl` file)
- `problem_file` - file path of problem file (it should be a `.pddl` file)
- `planner` - name of planner to use (`lama` or `mercury`)
- `mode` - `PlanGoal.NORMAL` or `PlanGoal.FAST` (fast mode implies that first
  plan found will be returned. No further optimisation will be performed by the
  planner. **Note**: This should only be use when a small number of goals are
  provided as it will produce a very bad and non optimal plan.)



## Requirements
- [mercury_planner](https://github.com/b-it-bots/mercury_planner)
- [lama_planner](https://github.com/b-it-bots/lama_planner)

## Test

- Without ROS
  ```
  python planner_wrapper.py
  ```

- With ROS
  ```
  roscore
  roslaunch mir_task_planning task_planner.launch
  roslaunch mir_task_planning task_planner_client_test.launch
  ```

## Additional files

- `ros/launch/upload_problem.launch` and `ros/test/upload_problem` can be used
  for testing whole planning pipeline without Refbox. This launch file will act
  as *refbox*, *refbox client* and *refbox parser*. It reads a `.pddl` problem file
  and uploads instances and facts to *knowledge base*. To test the whole
  planning pipeline without refbox, following things need to be executed
  ```
  roscore
  roslaunch mir_planning_core task_planning_components.launch
  rosrun mir_planner_executor planner_executor_mockup
  roslaunch mir_task_planning upload_problem.launch
  roslaunch mir_planning_core task_planning_sm.launch
  ```

- `common/pddl` contains robocup's domain file and a bunch of problem files for
  testing. **Note**: This folder is needed by default if
  `common/planner_wrapper/planner_wrapper.py` is running standalone.

