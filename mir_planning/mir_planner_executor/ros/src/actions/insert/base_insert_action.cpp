/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/insert/base_insert_action.h>
#include <utility>

void BaseInsertAction::update_knowledge_base(bool success,
                                             std::vector<diagnostic_msgs::KeyValue> &params)
{
  std::string robot = getValueOf(params, "robot_name");
  std::string platform = getValueOf(params, "platform");
  std::string location = getValueOf(params, "location");
  std::string peg = getValueOf(params, "peg");
  std::string hole = getValueOf(params, "hole");
  if (success) {
    knowledge_updater_->addKnowledge("in", {{"peg", peg}, {"hole", hole}});
    knowledge_updater_->remGoal("in", {{"peg", peg}, {"hole", hole}});

    knowledge_updater_->addKnowledge("on", {{"peg", peg}, {"l", location}});
    knowledge_updater_->remGoal("on", {{"peg", peg}, {"l", location}});

    knowledge_updater_->addKnowledge("heavy", {{"peg", peg}});
    knowledge_updater_->remGoal("heavy", {{"peg", peg}});

    knowledge_updater_->addKnowledge("heavy", {{"hole", hole}});
    knowledge_updater_->remGoal("heavy", {{"hole", hole}});

    knowledge_updater_->remKnowledge("stored", {{"o", peg}, {"rp", platform}});

    knowledge_updater_->remKnowledge("occupied", {{"rp", platform}});
  } else {
    // knowledge_updater_->remKnowledge("perceived", {{"l", location}});
    knowledge_updater_->remGoalsWithObject(peg);
    ROS_WARN("Insert failed, remove goals with object \"%s\"", peg.c_str());
  }
}

void BaseInsertAction::updateParamsBasedOnContext(std::vector<diagnostic_msgs::KeyValue> &params)
{
  int platform_index = getIndexOf(params, "param_1");
  int location_index = getIndexOf(params, "param_2");
  int peg_index = getIndexOf(params, "param_3");
  int hole_index = getIndexOf(params, "param_4");
  if (platform_index > 0) params[platform_index].key = "platform";
  if (location_index > 0) params[location_index].key = "location";
  if (peg_index > 0) params[peg_index].key = "peg";
  if (hole_index > 0) params[hole_index].key = "hole";
}
