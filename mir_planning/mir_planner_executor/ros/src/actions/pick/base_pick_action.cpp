/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/pick/base_pick_action.h>
#include <utility>

void BasePickAction::update_knowledge_base(bool success,
                                           std::vector<diagnostic_msgs::KeyValue> &params)
{
  std::string robot = getValueOf(params, "robot_name");
  std::string location = getValueOf(params, "location");
  std::string object = getValueOf(params, "object");
  if (success) {
    knowledge_updater_->addKnowledge("holding", {{"r", robot}, {"o", object}});
    knowledge_updater_->remGoal("holding", {{"r", robot}, {"o", object}});
    knowledge_updater_->remKnowledge("on", {{"o", object}, {"l", location}});
    knowledge_updater_->remKnowledge("gripper_is_free", {{"r", robot}});
  } else {
    int count = 1;
    if (failure_count_.find(object) != failure_count_.end()) {
      count = failure_count_[object] + 1;
    }
    ROS_WARN("Pick for object \"%s\" failed %d times", object.c_str(), count);
    if (count >= 3) {  // if failed to pick 3 times, remove goal and add to end of list
      knowledge_updater_->remGoalsWithObject(object);
      knowledge_updater_->remKnowledge("perceived", {{"l", location}});
      ROS_WARN("Pick failed %d times, remove goals with object \"%s\"", count, object.c_str());
      failure_count_[object] = 0;
    } else if (count == 2) {  // if failed to pick 2 times, move base to same location and retry
      knowledge_updater_->remKnowledge("perceived", {{"l", location}});
      knowledge_updater_->remKnowledge("at", {{"r", robot}, {"l", location}});
      knowledge_updater_->addKnowledge("at", {{"r", robot}, {"l", "START"}});
      failure_count_[object] = count;
    } else {  // reperceive and try to pick
      knowledge_updater_->remKnowledge("perceived", {{"l", location}});
      failure_count_[object] = count;
    }
  }
}

void BasePickAction::updateParamsBasedOnContext(std::vector<diagnostic_msgs::KeyValue> &params)
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
