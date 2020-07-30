/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/move/move_action.h>

MoveAction::MoveAction() : ExecutorAction("/move_base_safe_server")
{
  // client_.waitForServer();
}

void MoveAction::update_knowledge_base(bool success, std::vector<diagnostic_msgs::KeyValue> &params)
{
  std::string robot = getValueOf(params, "robot_name");
  std::string from = getValueOf(params, "source_location");
  std::string to = getValueOf(params, "destination_location");
  int N = 0;
  if (success) {
    knowledge_updater_->remKnowledge("at", {{"r", robot}, {"l", from}});
    knowledge_updater_->remGoal("at", {{"r", robot}, {"l", to}});
    knowledge_updater_->addKnowledge("at", {{"r", robot}, {"l", to}});
    knowledge_updater_->remKnowledge("perceived", {{"l", from}});
    knowledge_updater_->remKnowledge("perceived", {{"l", to}});
  } else {
    int count = 1;
    if (failure_count_.find(to) != failure_count_.end()) {
      count = failure_count_[to] + 1;
    }
    ROS_WARN("Move to location \"%s\" failed %d times", to.c_str(), count);
    if (count > N) {
      knowledge_updater_->remGoalsRelatedToLocation(to);
      ROS_WARN("Move failed %d times, remove goals with object from location \"%s\"", count,
               to.c_str());
      failure_count_[to] = 0;
    } else {
      failure_count_[to] = count;
    }
  }
}

void MoveAction::updateParamsBasedOnContext(std::vector<diagnostic_msgs::KeyValue> &params)
{
  int from_index = getIndexOf(params, "param_1");
  int to_index = getIndexOf(params, "param_2");
  if (from_index > 0) {
    params[from_index].key = "source_location";
  }
  if (to_index > 0) {
    params[to_index].key = "destination_location";
  }
}
