.. _mir_planning_core:

Planning core
=============

Handles plan generation, execution, monitoring and replanning (if needed)


Dependencies
------------

- ``mir_planning_msgs``
- ``mir_states``
- ``mir_pddl_problem_generator`` (:ref:`mir_pddl_problem_generator`)
- ``mir_task_planning`` (:ref:`mir_task_planning_pkg`)
- ``mir_planner_executor`` (:ref:`mir_planner_executor`)
- Optionally (with real robot) ``mir_actions`` (:ref:`mir_actions`)


Usage
-----

This is not a standalone package (but it acts like a coordinator) and thus it
should not be executed without its dependencies.

To test with mockup action servers without a robot

.. code-block:: bash

    roscore
    roslaunch mir_planning_core task_planning_components.launch
    rosrun mir_task_executor task_executor_mockup
    roslaunch mir_task_planning upload_problem.launch
    roslaunch mir_planning_core task_planning_sm.launch


