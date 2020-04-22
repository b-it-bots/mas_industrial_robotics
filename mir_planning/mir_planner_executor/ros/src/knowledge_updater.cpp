/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/knowledge_updater.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <ros/console.h>

KnowledgeUpdater::KnowledgeUpdater(ros::NodeHandle &nh) {
    rosplan_update_client_ = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/rosplan_knowledge_base/update");
    rosplan_get_goals_client_ = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/rosplan_knowledge_base/state/goals");
    rosplan_get_knowledge_client_ = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/rosplan_knowledge_base/state/propositions");
    re_add_goals_server_ = nh.advertiseService("re_add_goals", &KnowledgeUpdater::re_add_goals, this);
}
KnowledgeUpdater::~KnowledgeUpdater() {

}

bool KnowledgeUpdater::update_knowledge(uint8_t type, std::string name, std::vector<std::pair<std::string, std::string>> values) {
    rosplan_knowledge_msgs::KnowledgeItem msg;
    msg.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
    msg.instance_type = "";
    msg.instance_name = "";
    msg.attribute_name = name;
    msg.function_value = 0.0;
    for(auto& p: values) {
        diagnostic_msgs::KeyValue kv;
        kv.key = p.first;
        kv.value = toUpper(p.second);
        msg.values.push_back(kv);
    }
    rosplan_knowledge_msgs::KnowledgeUpdateService srv;
    srv.request.update_type = type;
    srv.request.knowledge = msg;
    return rosplan_update_client_.call(srv);
}

bool KnowledgeUpdater::remKnowledge(std::string name, std::vector<std::pair<std::string, std::string>> values) {
    return update_knowledge(rosplan_knowledge_msgs::KnowledgeUpdateServiceRequest::REMOVE_KNOWLEDGE, name, values);
}

bool KnowledgeUpdater::addKnowledge(std::string name, std::vector<std::pair<std::string, std::string>> values) {
    return update_knowledge(rosplan_knowledge_msgs::KnowledgeUpdateServiceRequest::ADD_KNOWLEDGE, name, values);
}

bool KnowledgeUpdater::remGoal(std::string name, std::vector<std::pair<std::string, std::string>> values) {
    return update_knowledge(rosplan_knowledge_msgs::KnowledgeUpdateServiceRequest::REMOVE_GOAL, name, values);
}

bool KnowledgeUpdater::addGoal(std::string name, std::vector<std::pair<std::string, std::string>> values) {
    return update_knowledge(rosplan_knowledge_msgs::KnowledgeUpdateServiceRequest::ADD_GOAL, name, values);
}

bool KnowledgeUpdater::remGoalsWithObject(std::string object_name) {
    rosplan_knowledge_msgs::GetAttributeService srv;
    if (!rosplan_get_goals_client_.call(srv)) {
        ROS_ERROR("Failed to call rosplan GetAttributeService");
        return false;
    }
    const std::vector<rosplan_knowledge_msgs::KnowledgeItem>& goals = srv.response.attributes;
    for (auto const& goal : goals) {
        for (auto const& item : goal.values) {
            if ((toUpper(item.key) == toUpper("o") || toUpper(item.key) == toUpper("peg")) && (toUpper(item.value) == toUpper(object_name))) {
                rosplan_knowledge_msgs::KnowledgeUpdateService srv_del;
                srv_del.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateServiceRequest::REMOVE_GOAL;
                srv_del.request.knowledge = goal;
                if (!rosplan_update_client_.call(srv_del)) {
                    return false;
                }
                removed_goals_.push_back(goal);
                break;
            }
        }
    }
    return true;
}

bool KnowledgeUpdater::remGoalsWithLocation(std::string location) {
    rosplan_knowledge_msgs::GetAttributeService srv;
    if (!rosplan_get_goals_client_.call(srv)) {
        ROS_ERROR("Failed to call rosplan GetAttributeService");
        return false;
    }
    const std::vector<rosplan_knowledge_msgs::KnowledgeItem>& goals = srv.response.attributes;
    for (auto const& goal : goals) {
        for (auto const& item : goal.values) {
            if (toUpper(item.key) == toUpper("l") && (toUpper(item.value) == toUpper(location))) {
                rosplan_knowledge_msgs::KnowledgeUpdateService srv_del;
                srv_del.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateServiceRequest::REMOVE_GOAL;
                srv_del.request.knowledge = goal;
                if (!rosplan_update_client_.call(srv_del)) {
                    return false;
                }
                removed_goals_.push_back(goal);
                break;
            }
        }
    }
    return true;
}


bool KnowledgeUpdater::remGoalsRelatedToLocation(std::string location) {
    remGoalsWithLocation(location);
    rosplan_knowledge_msgs::GetAttributeService srv;
    if (!rosplan_get_knowledge_client_.call(srv)) {
        ROS_ERROR("Failed to call rosplan GetAttributeService");
        return false;
    }
    const std::vector<rosplan_knowledge_msgs::KnowledgeItem>& facts = srv.response.attributes;
    for (auto const& fact : facts) {
        std::string name = fact.attribute_name;
        if (toUpper(name) == toUpper("on")) {
            std::string o = fact.values[0].value;
            std::string l = fact.values[1].value;
            if(toUpper(location) == toUpper(l)){
                remGoalsWithObject(o);
            }
        }
    }
    return true;
}

bool KnowledgeUpdater::re_add_goals(mir_planning_msgs::ReAddGoals::Request &req, mir_planning_msgs::ReAddGoals::Response &res) {
    ROS_INFO("Going to re-add removed goals!");
    while (!removed_goals_.empty()) {
        auto goal = removed_goals_[0];
        rosplan_knowledge_msgs::KnowledgeUpdateService srv_add;
        srv_add.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateServiceRequest::ADD_GOAL;
        srv_add.request.knowledge = goal;
        if (!rosplan_update_client_.call(srv_add)) {
            res.success = false;
            return false;
        }
        removed_goals_.erase(removed_goals_.begin());
    }
    res.success = true;
    return true;
}

std::string KnowledgeUpdater::toUpper(std::string str) {
    std::transform(str.begin(), str.end(), str.begin(), ::toupper);
    return str;
}
