/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/perceive/perceive_action.h>

PerceiveAction::PerceiveAction() : BasePerceiveAction("/perceive_location_server")
{
  // client_.waitForServer();
}
