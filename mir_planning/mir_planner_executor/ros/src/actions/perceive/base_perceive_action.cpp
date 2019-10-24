/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/perceive/base_perceive_action.h>
#include <utility>

void BasePerceiveAction::update_knowledge_base(bool success, std::vector<diagnostic_msgs::KeyValue>& params)
{
    std::string robot = getValueOf(params, "robot_name");
    std::string location = getValueOf(params, "location");
    if(success) {
        knowledge_updater_->addKnowledge("perceived", {{"l", location}});
        knowledge_updater_->remGoal("perceived", {{"l", location}});
    } else {
        //TODO move base?
        return;
    }
}

void BasePerceiveAction::updateParamsBasedOnContext(std::vector<diagnostic_msgs::KeyValue>& params)
{
    int location_index = getIndexOf(params, "param_1");
    if (location_index > 0)
        params[location_index].key = "location";
}
