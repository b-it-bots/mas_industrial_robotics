/*
 * Copyright [2016] <Bonn-Rhein-Sieg University>
 *
 * Author: Oscar Lima (olima_84@yahoo.com)
 *
 * Creates PDDL problem definition automatically from knowledge base snapshot
 *
 */

#include <mir_pddl_generator_node/pddl_problem_generator_node.h>
#include <string>

PDDLProblemGeneratorNode::PDDLProblemGeneratorNode() : nh_("~"), is_event_in_received_(false)
{
  // subscriptions
  sub_event_in_ = nh_.subscribe("event_in", 1, &PDDLProblemGeneratorNode::eventInCallback, this);

  // publications
  pub_event_out_ = nh_.advertise<std_msgs::String>("event_out", 2);

  nh_.param<std::string>("problem_path", problem_path_, "/tmp/problem.pddl");
  ROS_INFO("PDDL problem path : %s", problem_path_.c_str());
  pddl_problem_generator_ = new PDDLProblemGenerator();
}

PDDLProblemGeneratorNode::~PDDLProblemGeneratorNode()
{
  delete pddl_problem_generator_;
  // shut down publishers and subscribers
  sub_event_in_.shutdown();
  pub_event_out_.shutdown();
}

void PDDLProblemGeneratorNode::eventInCallback(const std_msgs::String::ConstPtr &msg)
{
  event_in_msg_ = *msg;
  is_event_in_received_ = true;
}

void PDDLProblemGeneratorNode::update()
{
  // listen to callbacks
  ros::spinOnce();

  if (!is_event_in_received_) return;

  // reset flag
  is_event_in_received_ = false;

  // checking for event in msg content
  if (event_in_msg_.data != "e_trigger") {
    ROS_ERROR("Received unsupported event: %s", event_in_msg_.data.c_str());
    return;
  }

  // generate PDDL file
  if (!pddl_problem_generator_->generatePDDLProblemFile(problem_path_)) {
    ROS_ERROR("An error occurred while generating PDDL file");
    event_out_msg_.data = std::string("e_failure");
    pub_event_out_.publish(event_out_msg_);
    return;
  }

  event_out_msg_.data = std::string("e_success");
  pub_event_out_.publish(event_out_msg_);
  ROS_INFO("Succesfully created PDDL problem from KB snapshot !");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pddl_problem_generator");

  ROS_INFO("Node is going to initialize...");

  // create object of the node class (PDDLProblemGeneratorNode)
  PDDLProblemGeneratorNode pddl_problem_generator_node;

  // setup node frequency
  double node_frequency = 10.0;
  ros::NodeHandle nh("~");
  nh.param("node_frequency", node_frequency, 10.0);
  ROS_INFO("Node will run at : %lf [hz]", node_frequency);
  ros::Rate loop_rate(node_frequency);
  ROS_INFO("Node initialized.");

  while (ros::ok()) {
    // main loop function
    pddl_problem_generator_node.update();

    // sleep to control the node frequency
    loop_rate.sleep();
  }

  return 0;
}
