/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/unstage/unstage_action.h>
#include <utility>

UnstageAction::UnstageAction() : ExecutorAction("/unstage_object_server")
{
  // client_.waitForServer();
}

void UnstageAction::update_knowledge_base(bool success,
                                          std::vector<diagnostic_msgs::KeyValue> &params)
{
  std::string robot = getValueOf(params, "robot_name");
  std::string platform = getValueOf(params, "platform");
  std::string object = getValueOf(params, "object");
  if (success) {
    knowledge_updater_->remKnowledge("gripper_is_free", {{"r", robot}});
    knowledge_updater_->remKnowledge("stored", {{"rp", platform}, {"o", object}});
    knowledge_updater_->remKnowledge("occupied", {{"rp", platform}});

    knowledge_updater_->addKnowledge("holding", {{"r", robot}, {"o", object}});
    knowledge_updater_->remGoal("holding", {{"r", robot}, {"o", object}});
  } else {
    return;
  }
}

void UnstageAction::updateParamsBasedOnContext(std::vector<diagnostic_msgs::KeyValue> &params)
{
  int platform_index = getIndexOf(params, "param_1");
  int object_index = getIndexOf(params, "param_2");
  if (platform_index > 0) {
    params[platform_index].key = "platform";
  }
  if (object_index > 0) {
    params[object_index].key = "object";
  }
}
