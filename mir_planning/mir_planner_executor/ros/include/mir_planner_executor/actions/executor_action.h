/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#pragma once
#include <mir_planner_executor/actions/base_executor_action.h>
#include <vector>
#include <mir_planner_executor/knowledge_updater.h>
#include <diagnostic_msgs/KeyValue.h>
#include <mir_planning_msgs/GenericExecuteAction.h>
#include <actionlib/client/simple_action_client.h>

class ExecutorAction : public BaseExecutorAction {
private:
protected:
    actionlib::SimpleActionClient<mir_planning_msgs::GenericExecuteAction> client_;
    KnowledgeUpdater* knowledge_updater_;

    ExecutorAction(std::string server_topic) : client_(server_topic){};

    virtual bool run(std::vector<diagnostic_msgs::KeyValue>& params);
    virtual void update_knowledge_base(bool success, std::vector<diagnostic_msgs::KeyValue>& params) = 0;
    virtual void updateParamsBasedOnContext(std::vector<diagnostic_msgs::KeyValue>& params) = 0;
public:

    virtual bool execute(std::string& name, std::vector<diagnostic_msgs::KeyValue>& params);
    virtual void initialize(KnowledgeUpdater* knowledge_updater);

};
