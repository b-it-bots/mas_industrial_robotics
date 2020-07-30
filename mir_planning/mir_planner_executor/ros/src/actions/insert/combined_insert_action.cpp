/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/insert/combined_insert_action.h>
#include <mir_planner_executor/actions/insert/insert_action.h>
#include <mir_planner_executor/actions/insert/insert_cavity_action.h>
#include <mir_planner_executor/to_be_removed_constants.h>
#include <utility>

CombinedInsertAction::CombinedInsertAction()
{
  default_insert_ = new InsertAction();
  cavity_insert_ = new InsertCavityAction();
}

void CombinedInsertAction::initialize(KnowledgeUpdater *knowledge_updater)
{
  default_insert_->initialize(knowledge_updater);
  cavity_insert_->initialize(knowledge_updater);
}

bool CombinedInsertAction::execute(std::string &name,
                                   std::vector<diagnostic_msgs::KeyValue> &params)
{
  std::string hole = getValueOf(params, "param_4");
  std::transform(hole.begin(), hole.end(), hole.begin(), ::toupper);
  if (hole == CAVITY_OBJECT_NAME) {
    ROS_INFO("CombinedInsertAction: CAVITY");
    return cavity_insert_->execute(name, params);
  } else {
    ROS_INFO("CombinedInsertAction: DEFAULT");
    return default_insert_->execute(name, params);
  }
}
