/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/perceive/combined_perceive_action.h>
#include <mir_planner_executor/actions/perceive/perceive_action.h>
#include <mir_planner_executor/actions/perceive/perceive_cavity_action.h>
#include <mir_planner_executor/to_be_removed_constants.h>
#include <utility>

CombinedPerceiveAction::CombinedPerceiveAction()
{
  default_perceive_ = new PerceiveAction();
  cavity_perceive_ = new PerceiveCavityAction();
}

void CombinedPerceiveAction::initialize(KnowledgeUpdater *knowledge_updater)
{
  default_perceive_->initialize(knowledge_updater);
  cavity_perceive_->initialize(knowledge_updater);
}

bool CombinedPerceiveAction::execute(std::string &name,
                                     std::vector<diagnostic_msgs::KeyValue> &params)
{
  std::string location = getValueOf(params, "param_1");
  std::transform(location.begin(), location.end(), location.begin(), ::toupper);
  if (location == CAVITY_LOCATION_NAME) {
    ROS_INFO("CombinedPerceiveAction: CAVITY");
    return cavity_perceive_->execute(name, params);
  } else {
    ROS_INFO("CombinedPerceiveAction: DEFAULT");
    return default_perceive_->execute(name, params);
  }
}
