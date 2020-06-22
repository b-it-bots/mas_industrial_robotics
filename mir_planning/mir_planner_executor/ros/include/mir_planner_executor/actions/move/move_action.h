/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#pragma once

#include <mir_planner_executor/actions/executor_action.h>

class MoveAction : public ExecutorAction {
 public:
  MoveAction();

 protected:
  void updateParamsBasedOnContext(
      std::vector<diagnostic_msgs::KeyValue> &params) override;
  void update_knowledge_base(
      bool success, std::vector<diagnostic_msgs::KeyValue> &params) override;
  std::map<std::string, int> failure_count_;
};
