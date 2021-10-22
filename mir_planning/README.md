# mir_planning

![Planning-Visualisation](mir_planning_visualisation/docs/kb_and_plan_vis.gif)

## Test

```
roscore
roslaunch mir_planning_core task_planning_components.launch
roslaunch mir_task_planning upload_problem.launch
rosrun mir_planner_executor planner_executor_mockup
roslaunch mir_planning_visualisation test_planning_visualiser.launch
roslaunch mir_planning_core task_planning_sm.launch
```

# Generate plan manually

1. Run ros master
   ```
   roscore
   ```

2. Launch all the necessary planning components
   ```
   roslaunch mir_planning_core task_planning_components.launch
   ```

3. Visualise knowledge base and plan (Optional)
   ```
   roslaunch mir_planning_visualisation test_planning_visualiser.launch
   ```

4. Get the facts, instances and goals into the knowledge base. There are 2 ways
   to achieve this

   1. With `atwork_commander`
      - See instructions of how to launch `atwork_commander` in their
        [github repo](https://github.com/robocup-at-work/atwork-commander#howto)
      - Or alternatively run a rosbag file containing a task

   2. With a problem file
      ```
      roslaunch mir_task_planning upload_problem.launch
      ```

5. Launch planner executor mockup to create dummy action servers
   ```
   rosrun mir_planner_executor planner_executor_mockup
   ```

6. See the output of pddl problem generator (Optional)
   ```
   rostopic echo /mir_planning/pddl_problem_generator/event_out
   ```

7. Generate pddl problem
   ```
   rostopic pub /mir_planning/pddl_problem_generator/event_in std_msgs/String "data: 'e_trigger'" -1
   ```
   You should see `e_success` in the output of step 6. You can see the pddl
   problem file `mir_knowledge/common/pddl/general_domain/problems/p1.pddl`

8. Plan for the generated problem
   ```
   roslaunch mir_task_planning task_planner_client_test.launch
   ```
   You would see the output as a list of actions.
   You can also see the raw plan file at `~/.ros/task_plan.plan`.
