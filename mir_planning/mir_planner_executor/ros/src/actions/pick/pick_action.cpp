/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/pick/pick_action.h>

PickAction::PickAction() : BasePickAction("/wbc_pick_object_server") {
  // client_.waitForServer();
}
