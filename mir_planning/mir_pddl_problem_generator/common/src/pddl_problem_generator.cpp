/*
 * Copyright [2016] <Bonn-Rhein-Sieg University>
 *
 * Author: Oscar Lima (olima_84@yahoo.com)
 * Based on: https://github.com/KCL-Planning/ROSPlan/blob/master/rosplan_planning_system/src/generatePDDLProblem.cpp
 *
 * Generic class used to generate a PDDL problem file with or without cost information
 *
 */

#include <mir_pddl_problem_generator/pddl_problem_generator.h>
#include <map>
#include <string>
#include <vector>
#include <algorithm>
#include <utility>
#include <boost/filesystem.hpp>

PDDLProbGenCost::PDDLProbGenCost(std::string& problem_path, std::string& metric)
: problem_path_(problem_path), metric_(metric), ready_to_generate_(true) {

}

void PDDLProbGenCost::addPointsObject(std::string str, float f) {
    points_objects_[str] = f;
    for(int i = 0;i<100;i++) {
        std::ostringstream stringStream;
        stringStream << str << "-" << std::setfill('0') << std::setw(2) << i;
        std::string id = stringStream.str();
        points_objects_[id] = f;
    }
}

bool PDDLProbGenCost::generatePDDLProblemFile(KCL_rosplan::PlanningEnvironment& environment)
{
    points_actions_["in"] = 100.0f;

    points_locations_["sh01"] = 150.0f;
    points_locations_["sh02"] = 150.0f;
    points_locations_["sh03"] = 150.0f;
    points_locations_["sh04"] = 150.0f;
    points_locations_["sh05"] = 150.0f;

    //addPointsObject("bearing_box", 0.5);

    // check if configure method was called, if not then exit without doing anything
    if (!ready_to_generate_) return false;

    boost::filesystem::path boost_problem_file((problem_path_).c_str());
    boost::filesystem::path boost_problem_dir = boost_problem_file.parent_path();

    if(!(boost::filesystem::exists(boost_problem_dir))){
        if (!(boost::filesystem::create_directory(boost_problem_dir)))
            return false;
    }

    std::ofstream pFile;
    pFile.open((problem_path_).c_str());

    if (!makeHeader(environment, pFile))
    {
        std::cerr << "Error : Could not make header" << std::endl;
        pFile.close();
        return false;
    }

    if (!makeObjects(environment, pFile))
    {
        std::cerr << "Error : Could not make objects" << std::endl;
        pFile.close();
        return false;
    }

    if (!makeInitialStateHeader(pFile))
    {
        std::cerr << "Error : Could not make initial state header" << std::endl;
        pFile.close();
        return false;
    }

    if (!makeInitialStateCost(environment, pFile))
    {
        std::cerr << "Error : Could not make initial state costs" << std::endl;
        pFile.close();
        return false;
    }

    if (!makeInitialStateFacts(environment, pFile))
    {
        std::cerr << "Error : Could not make state facts" << std::endl;
        pFile.close();
        return false;
    }

    if (!makeGoals(environment, pFile))
    {
        std::cerr << "Error : Could not make goals" << std::endl;
        pFile.close();
        return false;
    }

    if (!makeMetric(pFile))
    {
        std::cerr << "Error : Could not make metric" << std::endl;
        pFile.close();
        return false;
    }

    if (!finalizePDDLFile(pFile))
    {
        std::cerr << "Error : Could not finalize PDDL file" << std::endl;
        pFile.close();
        return false;
    }

    pFile.close();
    return true;
}

bool PDDLProbGenCost::makeHeader(KCL_rosplan::PlanningEnvironment& environment, std::ofstream &pFile)
{
    // check if configure method was called, if not then exit without doing anything
    if (!ready_to_generate_) return false;

    pFile << ";This PDDL problem definition was made automatically from a KB snapshot" << std::endl;
    pFile << "(define (problem " << environment.domainName << "_task)" << std::endl;
    pFile << "(:domain " << environment.domainName << ")" << std::endl;
    pFile << "" << std::endl;

    return true;
}

bool PDDLProbGenCost::makeObjects(KCL_rosplan::PlanningEnvironment& environment, std::ofstream &pFile)
{
    // check if configure method was called, if not then exit without doing anything
    if (!ready_to_generate_) return false;

    bool is_there_objects = false;

    // objects
    pFile << "(:objects" << std::endl;

    for (std::map<std::string, std::vector<std::string> >::iterator iit=environment.type_object_map.begin();
         iit != environment.type_object_map.end(); ++iit)
    {
        if (iit->second.size() > 0)
        {
            pFile << "    ";

            for (size_t i = 0; i < iit->second.size(); i++)
            {
                pFile << iit->second[i] << " ";
            }

            pFile << "- " << iit->first << std::endl;
            if (!is_there_objects) is_there_objects = true;
        }
    }
    pFile << ")" << std::endl;
    pFile << "" << std::endl;

    return is_there_objects;
}

bool PDDLProbGenCost::makeInitialStateHeader(std::ofstream &pFile)
{
    // check if configure method was called, if not then exit without doing anything
    if (!ready_to_generate_) return false;

    pFile << "(:init" << std::endl;

    return true;
}

bool PDDLProbGenCost::makeInitialStateCost(KCL_rosplan::PlanningEnvironment& environment, std::ofstream &pFile)
{
    // check if configure method was called, if not then exit without doing anything
    if (!ready_to_generate_) return false;

    pFile << "    ;Cost information starts" << std::endl;
    pFile << "    (= (total-cost) 0)" << std::endl;
    pFile << "    ;Cost information ends" << std::endl;
    pFile << "" << std::endl;

    return true;
}

bool PDDLProbGenCost::makeInitialStateFacts(KCL_rosplan::PlanningEnvironment& environment, std::ofstream &pFile)
{
    // check if configure method was called, if not then exit without doing anything
    if (!ready_to_generate_) return false;

    bool is_there_facts = false;

    // add knowledge to the initial state
    for (size_t i = 0; i < environment.domain_attributes.size(); i++)
    {
        std::stringstream ss;
        ss << "    (";

        if (environment.domain_attributes[i].knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FUNCTION)
        {
            ss << "= (";
        }

        ss << environment.domain_attributes[i].attribute_name;

        // fetch the corresponding symbols from domain
        std::map<std::string, std::vector<std::string> >::iterator ait;
        ait = environment.domain_predicates.find(environment.domain_attributes[i].attribute_name);

        if (ait == environment.domain_predicates.end())
        {
            ait = environment.domain_functions.find(environment.domain_attributes[i].attribute_name);
        }

        if (ait == environment.domain_functions.end())
        {
            continue;
        }

        // find the PDDL parameters in the KnowledgeItem
        bool writeAttribute = true;

        for (size_t j=0; j < ait->second.size(); j++)
        {
            bool found = false;
            for (size_t k = 0; k < environment.domain_attributes[i].values.size(); k++)
            {
                if (0 == environment.domain_attributes[i].values[k].key.compare(ait->second[j]))
                {
                    ss << " " << environment.domain_attributes[i].values[k].value;
                    found = true;
                }
            }
            if (!found) writeAttribute = false;
        }

        ss << ")";

        if (!is_there_facts) is_there_facts = true;

        // output function value
        if (environment.domain_attributes[i].knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FUNCTION)
        {
            ss << " " << environment.domain_attributes[i].function_value << ")";
        }

        if (writeAttribute) pFile << ss.str() << std::endl;
    }

    // add knowledge to the initial state
    for (size_t i = 0; i < environment.instance_attributes.size(); i++)
    {
        std::stringstream ss;
        bool writeAttribute = false;

        // check if attribute is a PDDL predicate
        std::map<std::string, std::vector<std::string> >::iterator ait;

        ait = environment.domain_predicates.find(environment.instance_attributes[i].attribute_name);

        if (ait != environment.domain_predicates.end())
        {
            writeAttribute = true;

            ss << "    (" + environment.instance_attributes[i].attribute_name;

            // find the PDDL parameters in the KnowledgeItem
            for (size_t j = 0; j < ait->second.size(); j++)
            {
                bool found = false;

                for (size_t k = 0; k < environment.instance_attributes[i].values.size(); k++)
                {
                    if (0 == environment.instance_attributes[i].values[k].key.compare(ait->second[j]))
                    {
                        ss << " " << environment.instance_attributes[i].values[k].value;
                        found = true;
                    }
                }
                if (!found) writeAttribute = false;
            };
            ss << ")";
        }
        if (writeAttribute) pFile << ss.str() << std::endl;
    }
    pFile << ")" << std::endl;

    // blank space between facts and goals
    pFile << "" << std::endl;

    return is_there_facts;
}

float PDDLProbGenCost::getPoints(std::string str, std::map<std::string, float>& points) {
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    auto it = points.find(str);
    if(it == points.end()) {
        return 0.0f;
    } else {
        return it->second;
    }
}
float PDDLProbGenCost::getPointsAction(std::string str) {
    return getPoints(str, points_actions_);
}
float PDDLProbGenCost::getPointsObject(std::string str) {
    return getPoints(str, points_objects_);
}
float PDDLProbGenCost::getPointsLocation(std::string str) {
    return getPoints(str, points_locations_);
}

std::vector<std::tuple<float, std::string, std::string>> PDDLProbGenCost::genGoalsWithPoints(KCL_rosplan::PlanningEnvironment& environment) {
    // points, goal, origin ws
    std::vector<std::tuple<float, std::string, std::string>> goals;
    for (size_t i = 0; i < environment.goal_attributes.size(); i++)
    {
        auto goal = environment.goal_attributes[i];
        std::string object_name;
        std::stringstream ss;
        bool writeAttribute = true;

        std::string action_name = goal.attribute_name;
        float points = getPointsAction(action_name);

        // check if attribute belongs in the PDDL model
        auto ait = environment.domain_predicates.find(action_name);
        if (ait != environment.domain_predicates.end())
        {
            ss << "    (" + action_name;

            // find the PDDL parameters in the KnowledgeItem
            bool found = false;
            for (size_t j = 0; j < ait->second.size(); j++) {
                for (size_t k = 0; k < goal.values.size(); k++)
                {
                    if (0 == goal.values[k].key.compare(ait->second[j]))
                    {
                        std::string name = goal.values[k].value;
                        ss << " " << goal.values[k].value;
                        found = true;

                        if(action_name.compare("on") == 0) {
                            if(k == 0) {
                                points += getPointsObject(name);
                                object_name = name;
                            } else if(k == 1) {
                                points += getPointsLocation(name);
                            }
                        } else if(action_name.compare("in") == 0) {
                            if(k == 0) {
                                points += getPointsObject(name);
                                object_name = name;
                            }
                        }
                    }
                }
            }
            if (!found)
                writeAttribute = false;

            ss << ")";
        } else {
            writeAttribute = false;
        }
        if (writeAttribute) {
            std:string location = getObjectLocation(object_name, environment);
            goals.push_back(std::make_tuple(points, ss.str(), location));
            //std::cout << ss.str() << std::endl;
        }
    }
    return goals;
}

std::string PDDLProbGenCost::getObjectLocation(std::string obj, KCL_rosplan::PlanningEnvironment& environment) {
    for(auto it = environment.domain_attributes.begin(); it != environment.domain_attributes.end(); ++it) {
        if ((*it).attribute_name.compare("on") != 0) {
            continue;
        }
        bool sameObject = false;
        for(auto xt = (*it).values.begin(); xt != (*it).values.end(); ++xt) {
            if((*xt).key.compare("o") != 0) {
                continue;
            }
            if((*xt).value.compare(obj) == 0) {
                sameObject = true;
                break;
            }
        }
        if (!sameObject) {
            continue;
        }
        for(auto xt = (*it).values.begin(); xt != (*it).values.end(); ++xt) {
            if((*xt).key.compare("l") != 0) {
                continue;
            }
            return (*xt).value;
        }
    }
    return "";
}

bool PDDLProbGenCost::makeGoals(KCL_rosplan::PlanningEnvironment& environment, std::ofstream &pFile)
{
    // check if configure method was called, if not then exit without doing anything
    if (!ready_to_generate_) return false;

    std::vector<std::tuple<float, std::string, std::string>> goals = genGoalsWithPoints(environment);
    if(goals.size() <= 0) return false;

    pFile << "(:goal (and" << std::endl;

    std::sort(goals.begin(), goals.end(), goal_sort_());

    for(auto it = goals.begin(); it != goals.end();++it) {
        std::cout << std::get<0>(*it) << " " << std::get<1>(*it) << " " << std::get<2>(*it) << std::endl;
    }

    uint count = 0;
    while(true) {
        if(goals.size() <= 0) {
            break;
        }
        if(count >= (max_goals_)) {
            break;
        }
        auto mainGoal = goals[0];
        goals.erase(goals.begin());
        pFile << std::get<1>(mainGoal) << std::endl;
        count++;
        auto loc = std::get<2>(mainGoal);
        while(true) {
            bool found = false;
            for(auto it = goals.begin(); it != goals.end();++it) {
                auto goal = *it;
                if(loc.compare(std::get<2>(goal)) == 0) {
                    found = true;
                    pFile << std::get<1>(goal) << std::endl;
                    count++;
                    goals.erase(it);
                    break;
                }
            }
            if(!found) {
                break;
            }
        }
    }

    pFile << "    )" << std::endl;
    pFile << ")" << std::endl;
    pFile << "" << std::endl;

    return count > 0;
}

bool PDDLProbGenCost::makeMetric(std::ofstream& pFile)
{
    // check if configure method was called, if not then exit without doing anything
    if (!ready_to_generate_) return false;

    // metric specification
    pFile << metric_ << std::endl;
    pFile << "" << std::endl;

    return true;
}

bool PDDLProbGenCost::finalizePDDLFile(std::ofstream& pFile)
{
    // check if configure method was called, if not then exit without doing anything
    if (!ready_to_generate_) return false;

    // end of problem
    pFile << ")" << std::endl;

    return true;
}

void PDDLProbGenCost::setMaxGoals(int max_goals)
{
    max_goals_ = max_goals;
}
