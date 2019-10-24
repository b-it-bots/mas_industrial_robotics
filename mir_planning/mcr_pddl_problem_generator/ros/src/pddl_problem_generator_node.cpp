/*
 * Copyright [2016] <Bonn-Rhein-Sieg University>
 *
 * Author: Oscar Lima (olima_84@yahoo.com)
 *
 * Creates PDDL problem definition automatically from knowledge base snapshot
 *
 */

#include <mcr_pddl_generator_node/pddl_problem_generator_node.h>
#include <string>

PDDLProblemGeneratorNode::PDDLProblemGeneratorNode() : nh_("~"), is_event_in_received_(false)
{
    // subscriptions
    sub_event_in_ = nh_.subscribe("event_in", 1, &PDDLProblemGeneratorNode::eventInCallback, this);

    // publications
    pub_event_out_ = nh_.advertise<std_msgs::String>("event_out", 2);

    // querying parameters from parameter server
    getSetParams();
}

PDDLProblemGeneratorNode::~PDDLProblemGeneratorNode()
{
    delete pddl_problem_generator_;
    // shut down publishers and subscribers
    sub_event_in_.shutdown();
    pub_event_out_.shutdown();
}

void PDDLProblemGeneratorNode::getSetParams()
{
    std::string domain_path;
    std::string problem_path;


    std::string metric = "(:metric minimize (total-cost))";
    // setup cost file paths default arguments
    std::vector<std::string> default_args;
    default_args.push_back("no_cost");

    std::vector<std::string> cost_file_paths;

    // getting required parameters from parameter server
    nh_.param<std::string>("domain_path", domain_path, "/home/user/my_domain.pddl");
    nh_.param<std::string>("problem_path", problem_path, "/home/user/my_problem.pddl");

    // informing the user about the parameters which will be used
    ROS_INFO("PDDL domain path : %s", domain_path.c_str());
    ROS_INFO("PDDL problem path : %s", problem_path.c_str());

    pddl_problem_generator_ = new PDDLProbGenCost(problem_path, metric);

    int max_goals;
    nh_.param<int>("max_goals", max_goals, 3);
    pddl_problem_generator_->setMaxGoals(max_goals);

    // check domain file existance
    if (boost::filesystem::exists(domain_path.c_str()))
    {
        // parse domain.pddl and setup environment with its information
        environment_.parseDomain(domain_path);
    }
    else
    {
        ROS_ERROR("Error while parsing PDDL domain, file does not exist : %s", domain_path.c_str());
        ROS_WARN("Will exit process now...");
        exit(1);
    }
}

void PDDLProblemGeneratorNode::eventInCallback(const std_msgs::String::ConstPtr& msg)
{
    event_in_msg_ = *msg;
    is_event_in_received_ = true;
}

void PDDLProblemGeneratorNode::update()
{
    // listen to callbacks
    ros::spinOnce();

    if (!is_event_in_received_)
        return;

    // reset flag
    is_event_in_received_ = false;

    // checking for event in msg content
    if (event_in_msg_.data != "e_trigger")
    {
        ROS_ERROR("Received unsupported event: %s", event_in_msg_.data.c_str());
        return;
    }

    try
    {
        // take knowledge base snapshot and store it in environment
        environment_.update(nh_);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("An exception occurred while updating the environment (knowledge base) : %s", e.what());
        even_out_msg_.data = std::string("e_failure");
        pub_event_out_.publish(even_out_msg_);
        return;
    }

    // generate PDDL file
    if (!pddl_problem_generator_->generatePDDLProblemFile(environment_))
    {
        ROS_ERROR("An error occurred while generating PDDL file");
        even_out_msg_.data = std::string("e_failure");
        pub_event_out_.publish(even_out_msg_);

        return;
    }

    // publish even_out : "e_success"
    even_out_msg_.data = std::string("e_success");
    pub_event_out_.publish(even_out_msg_);
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

    while (ros::ok())
    {
        // main loop function
        pddl_problem_generator_node.update();

        // sleep to control the node frequency
        loop_rate.sleep();
    }

    return 0;
}

