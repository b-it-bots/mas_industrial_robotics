/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/stage/stage_action.h>
#include <utility>

StageAction::StageAction() : ExecutorAction("/stage_object_server") {
  // client_.waitForServer();
}

void StageAction::update_knowledge_base(
    bool success, std::vector<diagnostic_msgs::KeyValue> &params) {
  std::string robot = getValueOf(params, "robot_name");
  std::string platform = getValueOf(params, "platform");
  std::string object = getValueOf(params, "object");
  if (success) {
    knowledge_updater_->remKnowledge("holding", {{"r", robot}, {"o", object}});

    knowledge_updater_->addKnowledge("gripper_is_free", {{"r", robot}});
    knowledge_updater_->remGoal("gripper_is_free", {{"r", robot}});

    knowledge_updater_->addKnowledge("stored",
                                     {{"rp", platform}, {"o", object}});
    knowledge_updater_->remGoal("stored", {{"rp", platform}, {"o", object}});

    knowledge_updater_->addKnowledge("occupied", {{"rp", platform}});
    knowledge_updater_->remGoal("occupied", {{"rp", platform}});
  } else {
    return;
  }
}

void StageAction::updateParamsBasedOnContext(
    std::vector<diagnostic_msgs::KeyValue> &params) {
  int platform_index = getIndexOf(params, "param_1");
  int object_index = getIndexOf(params, "param_2");
  if (platform_index > 0) {
    params[platform_index].key = "platform";
  }
  if (object_index > 0) {
    params[object_index].key = "object";
  }
}
