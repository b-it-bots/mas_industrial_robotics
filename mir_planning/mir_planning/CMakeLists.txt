cmake_minimum_required(VERSION 2.8.3)
project(mir_planning)

find_package(catkin REQUIRED
  COMPONENTS
  mir_task_planning
  mir_planner_executor
  mir_planning_bringup
  mir_planning_core
  mir_planning_msgs
  mir_knowledge_base_analyzer
  mir_refbox_parser
  mir_actions
  mir_knowledge
  mir_pddl_problem_generator
)

catkin_metapackage()
