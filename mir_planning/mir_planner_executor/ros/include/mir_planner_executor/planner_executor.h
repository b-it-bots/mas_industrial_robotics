/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#pragma once

#include <actionlib/server/simple_action_server.h>
#include <mir_planner_executor/actions/base_executor_action.h>
#include <mir_planner_executor/knowledge_updater.h>
#include <mir_planning_msgs/ExecutePlanAction.h>
#include <ros/ros.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>
#include <map>
#include <string>
#include <vector>

class PlannerExecutor
{
 private:
  actionlib::SimpleActionServer<mir_planning_msgs::ExecutePlanAction> server_;

  KnowledgeUpdater *knowledge_updater_;

  ros::Publisher audio_publisher_;

  std::map<std::string, BaseExecutorAction *> actions_;

  void announceAction(std::string action_name, std::vector<diagnostic_msgs::KeyValue> params);

  BaseExecutorAction *getActionExecutor(std::string &name);

  bool checkPlan(const rosplan_dispatch_msgs::CompletePlan &plan);
  std::string toUpper(std::string str);
  void addActionExecutor(std::string name, BaseExecutorAction *action);

 public:
  PlannerExecutor(ros::NodeHandle &nh);
  ~PlannerExecutor();
  void executeCallback();
};
