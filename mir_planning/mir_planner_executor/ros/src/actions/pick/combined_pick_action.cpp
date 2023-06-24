/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/pick/combined_pick_action.h>
#include <mir_planner_executor/actions/pick/pick_action.h>
#include <mir_planner_executor/actions/pick/pick_from_shelf_action.h>
#include <mir_planner_executor/actions/pick/pick_from_tt_action.h>
#include <mir_planner_executor/to_be_removed_constants.h>
#include <utility>

CombinedPickAction::CombinedPickAction()
{
  default_pick_ = new PickAction();
  pick_from_shelf_ = new PickFromShelfAction();
  pick_from_tt_ = new PickFromTurnTableAction();
}

void CombinedPickAction::initialize(KnowledgeUpdater *knowledge_updater)
{
  default_pick_->initialize(knowledge_updater);
  pick_from_shelf_->initialize(knowledge_updater);
  pick_from_tt_->initialize(knowledge_updater);
}

bool CombinedPickAction::execute(std::string &name, std::vector<diagnostic_msgs::KeyValue> &params)
{
  std::string location = getValueOf(params, "param_1");
  std::transform(location.begin(), location.end(), location.begin(), ::toupper);
  std::string location_type = location.substr(0, 2);
  if (location_type == SHELF_LOCATION_TYPE) {
    ROS_INFO("CombinedPickAction: SHELF");
    return pick_from_shelf_->execute(name, params);
  } else if (location_type == TURN_TABLE_TYPE) {
    ROS_INFO("CombinedPickAction: TurnTable");
    return pick_from_tt_->execute(name, params);
  }
   else {
    ROS_INFO("CombinedPickAction: DEFAULT");
    return default_pick_->execute(name, params);
  }
}
