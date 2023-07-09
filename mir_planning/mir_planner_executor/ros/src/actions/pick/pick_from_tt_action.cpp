/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Vivek Mannava
 *
 */

#include <mir_planner_executor/actions/pick/pick_from_tt_action.h>

PickFromTurnTableAction::PickFromTurnTableAction() : BasePickAction("/rtt_server")
{
  // client_.waitForServer();
}

bool PickFromTurnTableAction::run(std::vector<diagnostic_msgs::KeyValue> &params)
{
  mir_planning_msgs::GenericExecuteGoal goal;
  goal.parameters = params;
  actionlib::SimpleClientGoalState state =
      client_.sendGoalAndWait(goal, ros::Duration(250.0), ros::Duration(5.0));
  if (state == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) {
    /* auto res = client_.getResult(); */
    /* ROS_INFO_STREAM(*res); */
    return true;
  } else
    return false;
}
