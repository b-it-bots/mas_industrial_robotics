/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/executor_action.h>
#include <ros/console.h>

void ExecutorAction::initialize(KnowledgeUpdater *knowledge_updater)
{
  knowledge_updater_ = knowledge_updater;
}

bool ExecutorAction::execute(std::string &name, std::vector<diagnostic_msgs::KeyValue> &params)
{
  updateParamsBasedOnContext(params);
  bool success = run(params);
  update_knowledge_base(success, params);
  return success;
}

bool ExecutorAction::run(std::vector<diagnostic_msgs::KeyValue> &params)
{
  mir_planning_msgs::GenericExecuteGoal goal;
  goal.parameters = params;
  actionlib::SimpleClientGoalState state =
      client_.sendGoalAndWait(goal, ros::Duration(150.0), ros::Duration(5.0));
  if (state == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) {
    /* auto res = client_.getResult(); */
    /* ROS_INFO_STREAM(*res); */
    return true;
  } else
    return false;
}
