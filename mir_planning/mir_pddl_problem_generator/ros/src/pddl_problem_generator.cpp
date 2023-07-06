#include <mir_pddl_generator_node/pddl_problem_generator.h>
#include <boost/filesystem.hpp>

PDDLProblemGenerator::PDDLProblemGenerator() : nh_("~")
{
    std::string knowledge_base;
    nh_.param<std::string>("knowledge_base", knowledge_base, "rosplan_knowledge_base");

    nh_.param<int>("max_goals", max_goals_, 3);
    nh_.param<bool>("prefer_goals_with_same_source_ws", prefer_goals_with_same_source_ws_, true);

    nh_.getParam("points", points_map_);
    if (points_map_.size() == 0) ROS_ERROR("Points for objects and location not defined.");

    std::stringstream ss;

    ss << "/" << knowledge_base << "/domain/name";
    domain_name_service_ = ss.str();
    ss.str("");

    ss << "/" << knowledge_base << "/domain/types";
    domain_type_service_ = ss.str();
    ss.str("");

    ss << "/" << knowledge_base << "/domain/predicates";
    domain_predicate_service_ = ss.str();
    ss.str("");

    ss << "/" << knowledge_base << "/state/instances";
    state_instance_service_ = ss.str();
    ss.str("");

    ss << "/" << knowledge_base << "/state/propositions";
    state_proposition_service_ = ss.str();
    ss.str("");

    ss << "/" << knowledge_base << "/state/goals";
    state_goal_service_ = ss.str();
    ss.str("");

    ROS_INFO("[mir_pddl_problem_generator] Initialised");
}

PDDLProblemGenerator::~PDDLProblemGenerator() {}
bool PDDLProblemGenerator::generatePDDLProblemFile(const std::string &problem_path)
{
    boost::filesystem::path boost_problem_file((problem_path).c_str());
    boost::filesystem::path boost_problem_dir = boost_problem_file.parent_path();

    if (!(boost::filesystem::exists(boost_problem_dir))) {
        if (!(boost::filesystem::create_directory(boost_problem_dir))) {
            ROS_ERROR_STREAM("Could not create dir " << boost_problem_dir);
            return false;
        }
    }

    std::ofstream pFile;
    pFile.open((problem_path).c_str());

    if (!makeHeader(pFile)) {
        ROS_ERROR("Could not make header");
        pFile.close();
        return false;
    }

    if (!makeObjects(pFile)) {
        ROS_ERROR("Could not make objects");
        pFile.close();
        return false;
    }

    if (!makeInitialState(pFile)) {
        ROS_ERROR("Could not make initial state");
        pFile.close();
        return false;
    }

    if (!makeGoals(pFile)) {
        ROS_ERROR("Could not make goals");
        pFile.close();
        return false;
    }

    if (!makeMetric(pFile)) {
        ROS_ERROR("Could not make metric");
        pFile.close();
        return false;
    }

    pFile << ")" << std::endl;
    pFile.close();

    ROS_INFO("[mir_pddl_problem_generator] Problem file created successfully.");

    return true;
}

bool PDDLProblemGenerator::makeHeader(std::ofstream &pFile)
{
    // get domain name
    ros::ServiceClient getNameClient =
        nh_.serviceClient<rosplan_knowledge_msgs::GetDomainNameService>(domain_name_service_);
    rosplan_knowledge_msgs::GetDomainNameService name_srv;
    if (!getNameClient.call(name_srv)) {
        ROS_ERROR("[mir_pddl_problem_generator] Failed to call service %s",
                domain_name_service_.c_str());
        return false;
    }

    pFile << ";This PDDL problem definition was made automatically from a KB "
        "snapshot"
        << std::endl;
    pFile << "(define (problem " << name_srv.response.domain_name << "_task)" << std::endl;
    pFile << "(:domain " << name_srv.response.domain_name << ")" << std::endl;
    pFile << "" << std::endl;
    return true;
}

bool PDDLProblemGenerator::makeObjects(std::ofstream &pFile)
{
    ros::ServiceClient getTypesClient =
        nh_.serviceClient<rosplan_knowledge_msgs::GetDomainTypeService>(domain_type_service_);
    ros::ServiceClient getInstancesClient =
        nh_.serviceClient<rosplan_knowledge_msgs::GetInstanceService>(state_instance_service_);

    bool is_there_objects = false;

    // objects
    pFile << "(:objects" << std::endl;

    // get types
    rosplan_knowledge_msgs::GetDomainTypeService type_srv;
    if (!getTypesClient.call(type_srv)) {
        ROS_ERROR("[mir_pddl_problem_generator] Failed to call service %s",
                domain_type_service_.c_str());
    }

    // get instances of each type
    for (size_t t = 0; t < type_srv.response.types.size(); t++) {
        rosplan_knowledge_msgs::GetInstanceService instance_srv;
        instance_srv.request.type_name = type_srv.response.types[t];

        if (!getInstancesClient.call(instance_srv)) {
            ROS_ERROR("[mir_pddl_problem_generator] Failed to call service %s: %s",
                    state_instance_service_.c_str(), instance_srv.request.type_name.c_str());
        } else {
            if (instance_srv.response.instances.size() == 0) continue;
            pFile << "    ";
            for (size_t i = 0; i < instance_srv.response.instances.size(); i++) {
                pFile << instance_srv.response.instances[i] << " ";
            }
            pFile << "- " << type_srv.response.types[t] << std::endl;
            if (!is_there_objects) is_there_objects = true;
        }
    }
    pFile << ")" << std::endl;
    pFile << "" << std::endl;

    return is_there_objects;
}

bool PDDLProblemGenerator::makeInitialState(std::ofstream &pFile)
{
    ros::ServiceClient getDomainPropsClient =
        nh_.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>(
                domain_predicate_service_);
    ros::ServiceClient getPropsClient =
        nh_.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(state_proposition_service_);

    // note the time now for TILs
    ros::Time time = ros::Time::now() + ros::Duration(1);

    pFile << "(:init" << std::endl;
    pFile << "    ;Cost information starts" << std::endl;
    pFile << "    (= (total-cost) 0)" << std::endl;
    pFile << "    ;Cost information ends" << std::endl;
    pFile << "" << std::endl;

    bool is_there_facts = false;

    // get propositions
    rosplan_knowledge_msgs::GetDomainAttributeService domain_attr_srv;
    if (!getDomainPropsClient.call(domain_attr_srv)) {
        ROS_ERROR("[mir_pddl_problem_generator] Failed to call service %s",
                domain_predicate_service_.c_str());
    } else {
        std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator ait =
            domain_attr_srv.response.items.begin();
        for (; ait != domain_attr_srv.response.items.end(); ait++) {
            rosplan_knowledge_msgs::GetAttributeService attr_srv;
            attr_srv.request.predicate_name = ait->name;
            if (!getPropsClient.call(attr_srv)) {
                ROS_ERROR("[mir_pddl_problem_generator] Failed to call service %s: %s",
                        state_proposition_service_.c_str(), attr_srv.request.predicate_name.c_str());
            } else {
                for (size_t i = 0; i < attr_srv.response.attributes.size(); i++) {
                    rosplan_knowledge_msgs::KnowledgeItem attr = attr_srv.response.attributes[i];

                    pFile << "    (";

                    // Check if the attribute is negated
                    if (attr.is_negative) pFile << "not (";

                    pFile << attr.attribute_name;
                    for (auto j : ait->typed_parameters) {
                        for (auto k : attr.values) {
                            if (j.key.compare(k.key) == 0) {
                                pFile << " " << k.value;
                            }
                        }
                    }
                    pFile << ")";

                    if (attr.is_negative) pFile << ")";

                    pFile << std::endl;
                    is_there_facts = true;
                }
            }
        }
    }

    pFile << ")" << std::endl;

    // blank space between facts and goals
    pFile << std::endl;

    return is_there_facts;
}

bool PDDLProblemGenerator::makeGoals(std::ofstream &pFile)
{
    ros::ServiceClient getCurrentGoalsClient =
        nh_.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(state_goal_service_);

    pFile << "(:goal" << std::endl;
    pFile << "    (and" << std::endl;

    std::vector<std::tuple<std::map<std::string, std::string>, float>> goals;
    bool has_goals = false;

    // get current goals
    rosplan_knowledge_msgs::GetAttributeService current_goal_srv;
    if (!getCurrentGoalsClient.call(current_goal_srv)) {
        ROS_ERROR("[mir_pddl_problem_generator] Failed to call service %s",
                state_goal_service_.c_str());
    } else {
        for (size_t i = 0; i < current_goal_srv.response.attributes.size(); i++) {
            rosplan_knowledge_msgs::KnowledgeItem attr = current_goal_srv.response.attributes[i];

            if (attr.knowledge_type != rosplan_knowledge_msgs::KnowledgeItem::FACT) {
                continue;
            }

            if (attr.is_negative) {
                continue;
            }

            std::string source = getSourceLocation(attr);
            if (source.size() == 0) {
                continue;
            }

            std::map<std::string, std::string> goal;
            goal["src"] = source;
            goal["dest"] = attr.values[1].value;
            goal["obj"] = attr.values[0].value;
            goal["attr"] = attr.attribute_name;
            float score = getGoalScore(goal);
            goals.push_back(std::make_tuple(goal, score));
        }

        std::sort(goals.begin(), goals.end(), goal_sort_());

        /* for debugging */
        for (auto goal : goals) {
            std::cout << std::get<1>(goal) << " "
                      << std::get<0>(goal)["obj"] << " "
                      << std::get<0>(goal)["src"] << " -> "
                      << std::get<0>(goal)["dest"] << std::endl;
        }
        std::cout << std::endl;
        has_goals = goals.size() > 0;

        unsigned int goal_counter = 0;
        while (true) {
            if (goals.size() == 0) {
                break;
            }

            if (goal_counter >= max_goals_) {
                break;
            }

            std::cout << "Selected goals" << std::endl;
            auto main_goal = goals[0];
            std::cout << std::get<1>(main_goal) << " "
                      << std::get<0>(main_goal)["obj"] << " "
                      << std::get<0>(main_goal)["src"] << " -> "
                      << std::get<0>(main_goal)["dest"] << std::endl;
            goals.erase(goals.begin());
            pFile << "        (" << std::get<0>(main_goal)["attr"];
            pFile << " " << std::get<0>(main_goal)["obj"];
            pFile << " " << std::get<0>(main_goal)["dest"];
            pFile << ")" << std::endl;
            goal_counter++;
            std::string src_loc = std::get<0>(main_goal)["src"];
            if (prefer_goals_with_same_source_ws_) {
                unsigned int i = 0;
                while (i < goals.size()) {
                    auto goal = goals[i];
                    if (src_loc.compare(std::get<0>(goal)["src"]) == 0) {
                        pFile << "        (" << std::get<0>(goal)["attr"];
                        pFile << " " << std::get<0>(goal)["obj"];
                        pFile << " " << std::get<0>(goal)["dest"];
                        pFile << ")" << std::endl;
                        goal_counter++;
                        goals.erase(goals.begin() + i);
                    } else {
                        i++;
                    }
                }
            }
        }
    }
    pFile << "    )" << std::endl;
    pFile << ")" << std::endl << std::endl;
    return has_goals;
}

bool PDDLProblemGenerator::makeMetric(std::ofstream &pFile)
{
    pFile << "(:metric minimize (total-cost))" << std::endl;
    pFile << "" << std::endl;

    return true;
}

float PDDLProblemGenerator::getGoalScore(const std::map<std::string, std::string> &goal)
{
    float score = 0.0;
    score += getPointsLocation(goal.at("src"));
    score += getPointsLocation(goal.at("dest"));
    score += getPointsObject(goal.at("obj"));
    return score;
}

std::string PDDLProblemGenerator::getSourceLocation(const rosplan_knowledge_msgs::KnowledgeItem &ki)
{
    if (ki.attribute_name.compare("in") != 0 and ki.attribute_name.compare("on") != 0) return "";

    std::string obj = ki.values[0].value;

    ros::ServiceClient getPropsClient =
        nh_.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(state_proposition_service_);

    rosplan_knowledge_msgs::GetAttributeService attr_srv;
    // source location can only be "on"
    // i.e. we want to find the source of the object that is in the selected goal (ki)
    attr_srv.request.predicate_name = "on";
    if (!getPropsClient.call(attr_srv)) {
        ROS_ERROR("[mir_pddl_problem_generator] Failed to call service %s: %s",
                state_proposition_service_.c_str(), attr_srv.request.predicate_name.c_str());
        return "";
    } else {
        for (size_t i = 0; i < attr_srv.response.attributes.size(); i++) {
            rosplan_knowledge_msgs::KnowledgeItem attr = attr_srv.response.attributes[i];
            if (attr.values[0].value.compare(obj) == 0) {
                return attr.values[1].value;
            }
        }
    }
    attr_srv.request.predicate_name = "stored";
    if (!getPropsClient.call(attr_srv)) {
        ROS_ERROR("[mir_pddl_problem_generator] Failed to call service %s: %s",
                state_proposition_service_.c_str(), attr_srv.request.predicate_name.c_str());
        return "";
    } else {
        for (size_t i = 0; i < attr_srv.response.attributes.size(); i++) {
            rosplan_knowledge_msgs::KnowledgeItem attr = attr_srv.response.attributes[i];
            if (attr.values[1].value.compare(obj) == 0) {
                return attr.values[0].value;
            }
        }
    }
    return "XXXX";  // dummy location
}

float PDDLProblemGenerator::getPoints(const std::string &key)
{
    std::string key_lower(key.length(), 'X');
    std::transform(key.begin(), key.end(), key_lower.begin(), ::tolower);
    auto it = points_map_.find(key_lower);
    return (it == points_map_.end()) ? 0.0f : it->second;
}

float PDDLProblemGenerator::getPointsObject(const std::string &obj)
{
    size_t minus_pos = obj.find_first_of("-");
    return (minus_pos == obj.npos) ? getPoints(obj) : getPoints(obj.substr(0, minus_pos));
}

float PDDLProblemGenerator::getPointsLocation(const std::string &loc)
{
    return (loc.size() > 2) ? getPoints(loc.substr(0, 2)) : 0.0f;
}
