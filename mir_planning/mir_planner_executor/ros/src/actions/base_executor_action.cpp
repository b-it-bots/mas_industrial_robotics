/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/actions/base_executor_action.h>

std::string BaseExecutorAction::getValueOf(std::vector<diagnostic_msgs::KeyValue> &dict,
                                           std::string key)
{
  for (diagnostic_msgs::KeyValue kv : dict) {
    if (key.compare(kv.key) == 0) {
      return kv.value;
    }
  }
  return "";
}

int BaseExecutorAction::getIndexOf(std::vector<diagnostic_msgs::KeyValue> &dict, std::string key)
{
  for (int i = 0; i < dict.size(); i++) {
    if (key.compare(dict[i].key) == 0) {
      return i;
    }
  }
  return -1;
}
