/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#pragma once

#include <mir_planner_executor/actions/base_executor_action.h>
#include <mir_planner_executor/actions/executor_action.h>

class CombinedPerceiveAction : public BaseExecutorAction
{
 private:
  ExecutorAction *default_perceive_;
  ExecutorAction *cavity_perceive_;

 public:
  CombinedPerceiveAction();
  void initialize(KnowledgeUpdater *knowledge_updater) override;
  bool execute(std::string &name, std::vector<diagnostic_msgs::KeyValue> &params) override;
};
