/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#pragma once
#include <mir_planner_executor/knowledge_updater.h>
#include <vector>

class BaseExecutorAction
{
 private:
 protected:
 public:
  virtual bool execute(std::string &name, std::vector<diagnostic_msgs::KeyValue> &params) = 0;
  virtual void initialize(KnowledgeUpdater *knowledge_updater) = 0;

  /* helper functions to work with vector of KeyValue objs */
  std::string getValueOf(std::vector<diagnostic_msgs::KeyValue> &dict, std::string key);
  int getIndexOf(std::vector<diagnostic_msgs::KeyValue> &dict, std::string key);
};
