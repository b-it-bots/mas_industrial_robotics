/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/perceive/base_perceive_action.h>
#include <utility>

void BasePerceiveAction::update_knowledge_base(bool success,
                                               std::vector<diagnostic_msgs::KeyValue> &params)
{
  std::string robot = getValueOf(params, "robot_name");
  std::string location = getValueOf(params, "location");
  int N = 1;
  if (success) {
    knowledge_updater_->addKnowledge("perceived", {{"l", location}});
    knowledge_updater_->remGoal("perceived", {{"l", location}});
  } else {
        // get the next action and check if it is PICK
    std::string next_action = getValueOf(params, "next_action");
    if (next_action == "PICK" || next_action == "pick") {
      int count = 1;
      std::string object = getValueOf(params, "next_object");
      if (failure_count_.find(object) != failure_count_.end()) {
        count = failure_count_[object] + 1;
        knowledge_updater_->remKnowledge("at", {{"r", robot}, {"l", location}});
        knowledge_updater_->addKnowledge("at", {{"r", robot}, {"l", "START"}});
      }
      ROS_WARN("Perceive for object \"%s\" failed %d times", object.c_str(), count);
      if (count > N) {
        knowledge_updater_->remGoalsWithObject(object);
        knowledge_updater_->remKnowledge("perceived", {{"l", location}});
        knowledge_updater_->remKnowledge("at", {{"r", robot}, {"l", location}});
        knowledge_updater_->addKnowledge("at", {{"r", robot}, {"l", "START"}});
        ROS_WARN("Pick failed %d times, remove goals with object \"%s\"", count, object.c_str());
        failure_count_[object] = 0;
      } else {
        knowledge_updater_->remKnowledge("perceived", {{"l", location}});
        knowledge_updater_->remKnowledge("at", {{"r", robot}, {"l", location}});
        knowledge_updater_->addKnowledge("at", {{"r", robot}, {"l", "START"}});
        failure_count_[object] = count;
      }
    } else {
      return;
    }
  }
}

void BasePerceiveAction::updateParamsBasedOnContext(std::vector<diagnostic_msgs::KeyValue> &params)
{
  int location_index = getIndexOf(params, "param_1");
  if (location_index > 0) params[location_index].key = "location";
}
