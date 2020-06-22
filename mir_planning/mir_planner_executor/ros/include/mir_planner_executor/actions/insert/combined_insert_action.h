/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#pragma once

#include <mir_planner_executor/actions/base_executor_action.h>
#include <mir_planner_executor/actions/executor_action.h>

class CombinedInsertAction : public BaseExecutorAction {
 private:
  ExecutorAction *default_insert_;
  ExecutorAction *cavity_insert_;

 public:
  CombinedInsertAction();
  void initialize(KnowledgeUpdater *knowledge_updater) override;
  bool execute(std::string &name,
               std::vector<diagnostic_msgs::KeyValue> &params) override;
};
