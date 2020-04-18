
#ifndef PDDL_PROBLEM_GENERATOR_H
#define PDDL_PROBLEM_GENERATOR_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <utility>
#include <tuple>

#include "rosplan_knowledge_msgs/GetDomainNameService.h"
#include "rosplan_knowledge_msgs/GetDomainTypeService.h"
#include "rosplan_knowledge_msgs/GetDomainAttributeService.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "rosplan_knowledge_msgs/GetMetricService.h"

class PDDLProblemGenerator
{
    public:
        PDDLProblemGenerator();
        ~PDDLProblemGenerator();

        bool generatePDDLProblemFile(const std::string &problem_path);

    private:
        ros::NodeHandle nh_;

		std::string domain_name_service_;
		std::string domain_type_service_;
		std::string domain_predicate_service_;

		std::string state_instance_service_;
		std::string state_proposition_service_;
		std::string state_goal_service_;

        std::map<std::string, float> points_map_;
        int max_goals_;
        bool prefer_goals_with_same_source_ws_;

        float getPoints(const std::string &key);
        float getPointsObject(const std::string &obj);
        float getPointsLocation(const std::string &loc);
        float getGoalScore(const std::map<std::string, std::string> &goal);
        std::string getSourceLocation(const rosplan_knowledge_msgs::KnowledgeItem &ki);

        bool makeHeader(std::ofstream &pFile);
        bool makeObjects(std::ofstream &pFile);
        bool makeInitialState(std::ofstream &pFile);
        bool makeGoals(std::ofstream &pFile);
        bool makeMetric(std::ofstream& pFile);

        struct goal_sort_ {
            inline bool operator()(const std::tuple <std::map <std::string, std::string> , float>& a,
                                   const std::tuple <std::map <std::string, std::string> , float>& b)
            {
                return std::get<1>(a) >= std::get<1>(b);
            }
        };

};
#endif  // PDDL_PROBLEM_GENERATOR_H

