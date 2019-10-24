/*
 * Copyright [2016] <Bonn-Rhein-Sieg University>
 *
 * Author: Oscar Lima (olima_84@yahoo.com)
 *
 * Node used to generate PDDL problem definition automatically from
 * Knowledge base snapshot
 *
 */

#ifndef MCR_PDDL_GENERATOR_NODE_PDDL_PROBLEM_GENERATOR_NODE_H
#define MCR_PDDL_GENERATOR_NODE_PDDL_PROBLEM_GENERATOR_NODE_H

#include <ros/ros.h>
#include <rosplan_planning_system/PlanningEnvironment.h>
#include <mcr_pddl_problem_generator/pddl_problem_generator.h>
#include <std_msgs/String.h>
#include <string>
#include <boost/filesystem.hpp>

class PDDLProblemGeneratorNode
{
    public:
        PDDLProblemGeneratorNode();
        ~PDDLProblemGeneratorNode();

        // get parameters from param server
        void getSetParams();

        // std_msgs/String node event_in callback to trigger PDDL generation process
        void eventInCallback(const std_msgs::String::ConstPtr& msg);

        // ros node main loop
        void update();

    private:
        // flag used to know when we have received a callback
        bool is_event_in_received_;

        // ros related variables
        ros::NodeHandle nh_;
        ros::Publisher pub_event_out_;
        ros::Subscriber sub_event_in_;

        // for receiving event in msg
        std_msgs::String event_in_msg_;

        // for publishing event_out string msg
        std_msgs::String even_out_msg_;

        // Common class pddl problem generator
        PDDLProbGenCost* pddl_problem_generator_;

        // Parses the domain and gets its attributes
        KCL_rosplan::PlanningEnvironment environment_;
};
#endif  // MCR_PDDL_GENERATOR_NODE_PDDL_PROBLEM_GENERATOR_NODE_H

