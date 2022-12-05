#pragma once

#include<mir_planner_executor/actions/executor_action.h>

class PlaceUnstageAction : public ExecutorAction
{
 public:
  PlaceUnstageAction();

 protected:
  void updateParamsBasedOnContext(std::vector<diagnostic_msgs::KeyValue> &params) override;
  void update_knowledge_base(bool success, std::vector<diagnostic_msgs::KeyValue> &params) override;
};
