/*
 * Copyright [2016] <Bonn-Rhein-Sieg University>
 *
 * Author: Oscar Lima (olima_84@yahoo.com)
 *
 * Node used to generate PDDL problem definition automatically from
 * Knowledge base snapshot
 *
 */

#ifndef MIR_PDDL_GENERATOR_NODE_PDDL_PROBLEM_GENERATOR_NODE_H
#define MIR_PDDL_GENERATOR_NODE_PDDL_PROBLEM_GENERATOR_NODE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>

#include <mir_pddl_generator_node/pddl_problem_generator.h>

class PDDLProblemGeneratorNode {
 public:
  PDDLProblemGeneratorNode();
  ~PDDLProblemGeneratorNode();

  // std_msgs/String node event_in callback to trigger PDDL generation process
  void eventInCallback(const std_msgs::String::ConstPtr &msg);

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
  std_msgs::String event_out_msg_;

  std::string problem_path_;

  // Common class pddl problem generator
  PDDLProblemGenerator *pddl_problem_generator_;
};
#endif  // MIR_PDDL_GENERATOR_NODE_PDDL_PROBLEM_GENERATOR_NODE_H
