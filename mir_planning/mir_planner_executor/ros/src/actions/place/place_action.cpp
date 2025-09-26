/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/place/place_action.h>
#include <utility>

PlaceAction::PlaceAction() : ExecutorAction("/place_object_server")
{
  // client_.waitForServer();
}

void PlaceAction::update_knowledge_base(bool success,
                                        std::vector<diagnostic_msgs::KeyValue> &params)
{
  std::string robot = getValueOf(params, "robot_name");
  std::string location = getValueOf(params, "location");
  std::string object = getValueOf(params, "object");
  if (success) {
    knowledge_updater_->addKnowledge("on", {{"o", object}, {"l", location}});
    knowledge_updater_->remGoal("on", {{"o", object}, {"l", location}});
    knowledge_updater_->remKnowledge("holding", {{"r", robot}, {"o", object}});
    knowledge_updater_->addKnowledge("gripper_is_free", {{"r", robot}});
    knowledge_updater_->remGoal("gripper_is_free", {{"r", robot}});
    knowledge_updater_->remGoal("perceived", {{"l", location}});
  } else {
    return;
  }
}

void PlaceAction::updateParamsBasedOnContext(std::vector<diagnostic_msgs::KeyValue> &params)
{
  int location_index = getIndexOf(params, "param_1");
  int object_index = getIndexOf(params, "param_2");
  if (location_index > 0) {
    params[location_index].key = "location";
  }
  if (object_index > 0) {
    params[object_index].key = "object";
  }
}
