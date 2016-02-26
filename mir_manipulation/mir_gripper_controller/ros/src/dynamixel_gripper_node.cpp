/*
 * Copyright [2015] <Bonn-Rhein-Sieg University>
 *
 * dynamixel_gripper_node.cpp
 *
 *  Created on: Apr 14, 2015
 *      Author: Frederik Hegger
 */

#include <string>

#include <mir_gripper_controller/dynamixel_gripper_node.h>

DynamixelGripperNode::DynamixelGripperNode(ros::NodeHandle &nh) :
    gripper_action_server_(nh, "gripper_controller/gripper_command", false), 
    trajectory_action_server_(nh, "gripper_controller/follow_joint_trajectory", false), 
    joint_states_received_(false)
{
    pub_dynamixel_command_ = nh_.advertise < std_msgs::Float64 > ("dynamixel_command", 1);
    sub_dynamixel_motor_states_ = nh_.subscribe("dynamixel_motor_states", 10,
                                  &DynamixelGripperNode::jointStatesCallback, this);

    sub_gripper_command_ = nh_.subscribe("gripper_command", 10,
            &DynamixelGripperNode::gripperCommandCallback, this);

    pub_joint_states_ = nh_.advertise < sensor_msgs::JointState > ("joint_state", 10);

    // read parameters
    ROS_INFO("Parameters:");
    ros::NodeHandle nh_prv("~");
    nh_prv.param<double>("torque_limit", torque_limit_, 1.0);
    nh_prv.param<std::string>("torque_limit_srv_name", torque_limit_srv_name_, "");
    ROS_INFO_STREAM("\tTorque limit: " << torque_limit_); 
    nh_prv.param<int>("compliance_margin", compliance_margin_, 4);
    nh_prv.param<std::string>("compliance_margin_srv_name", compliance_margin_srv_name_, "");
    ROS_INFO_STREAM("\tCompliance margin: " << compliance_margin_); 
    nh_prv.param<int>("compliance_slope", compliance_slope_, 64);
    nh_prv.param<std::string>("compliance_slope_srv_name", compliance_slope_srv_name_, "");
    ROS_INFO_STREAM("\tCompliance slope: " << compliance_slope_);

    nh_prv.param<double>("gripper_configuration_open", gripper_configuration_open_, 0.0);
    ROS_INFO_STREAM("\tgripper configuration <open>: " << gripper_configuration_open_);

    nh_prv.param<double>("gripper_configuration_close", gripper_configuration_close_, 1.0);
    ROS_INFO_STREAM("\tgripper configuration <close>: " << gripper_configuration_close_);

    // set the torque limit
    dynamixel_controllers::SetTorqueLimit torque_srv;
    ros::ServiceClient srv_client_torque = nh_.serviceClient<dynamixel_controllers::SetTorqueLimit>(torque_limit_srv_name_);

    torque_srv.request.torque_limit = torque_limit_;

    ros::service::waitForService(torque_limit_srv_name_, ros::Duration(10.0));
    while (!srv_client_torque.call(torque_srv))
    {
        ROS_ERROR_STREAM("Failed to call service: " << torque_limit_srv_name_ << "! Will try again ...");
        sleep(2);
    }

    // set the compliance margin
    dynamixel_controllers::SetComplianceMargin compliance_margin_srv;
    ros::ServiceClient srv_client_compliance_margin = nh_.serviceClient<dynamixel_controllers::SetComplianceMargin>(compliance_margin_srv_name_);

    compliance_margin_srv.request.margin = compliance_margin_;

    ros::service::waitForService(compliance_margin_srv_name_, ros::Duration(10.0));
    while (!srv_client_compliance_margin.call(compliance_margin_srv))
    {
        ROS_ERROR_STREAM("Failed to call service: " << compliance_margin_srv_name_ << "! Will try again ...");
        sleep(2);
    }

    // set the torque limit
    dynamixel_controllers::SetComplianceSlope compliance_slope_srv;
    ros::ServiceClient srv_client_compliance_slope = nh_.serviceClient<dynamixel_controllers::SetComplianceSlope>(compliance_slope_srv_name_);

    compliance_slope_srv.request.slope = compliance_slope_;

    ros::service::waitForService(compliance_slope_srv_name_, ros::Duration(10.0));
    while (!srv_client_compliance_slope.call(compliance_slope_srv))
    {
        ROS_ERROR_STREAM("Failed to call service: " << compliance_slope_srv_name_ << "! Will try again ...");
        sleep(2);
    }

    // start action server
    trajectory_action_server_.registerGoalCallback(boost::bind(&DynamixelGripperNode::followJointTrajectoryGoalCallback, this));
    trajectory_action_server_.start();
    gripper_action_server_.registerGoalCallback(boost::bind(&DynamixelGripperNode::gripperCommandGoalCallback, this));
    gripper_action_server_.start();
}

DynamixelGripperNode::~DynamixelGripperNode()
{
    pub_dynamixel_command_.shutdown();
    pub_joint_states_.shutdown();
    sub_dynamixel_motor_states_.shutdown();
}

void DynamixelGripperNode::jointStatesCallback(const dynamixel_msgs::JointState::Ptr &msg)
{
    joint_states_ = msg;
    joint_states_received_ = true;

    // publish dynamixel joint states as sensor_msgs joint states
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = msg->header.stamp;

    joint_state.name.push_back("gripper_motor_right_joint");
    joint_state.position.push_back(msg->current_pos);
    joint_state.velocity.push_back(msg->velocity);
    joint_state.effort.push_back(msg->load);

    joint_state.name.push_back("gripper_motor_left_joint");
    joint_state.position.push_back(msg->current_pos);
    joint_state.velocity.push_back(msg->velocity);
    joint_state.effort.push_back(msg->load);

    pub_joint_states_.publish(joint_state);
}

void DynamixelGripperNode::gripperCommandCallback(const mcr_manipulation_msgs::GripperCommand::Ptr &msg)
{
    double set_pos = 0.0;

    if (msg->command == mcr_manipulation_msgs::GripperCommand::OPEN)
        set_pos = gripper_configuration_open_;
    else if (msg->command == mcr_manipulation_msgs::GripperCommand::CLOSE)
        set_pos = gripper_configuration_close_;
    else
    {
        ROS_ERROR_STREAM("Unsupported gripper command: " << msg->command);
        return;
    }

    std_msgs::Float64 gripper_pos;
    gripper_pos.data = set_pos;
    pub_dynamixel_command_.publish(gripper_pos);
}

void DynamixelGripperNode::moveGripper(double position) 
{
    ros::Rate loop_rate(100);
    // publish goal position
    std_msgs::Float64 gripper_pos;
    gripper_pos.data = position;
    pub_dynamixel_command_.publish(gripper_pos);

    joint_states_received_ = false;
    while (!joint_states_received_ && ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    gripper_result_.position = joint_states_->current_pos;
    gripper_result_.effort = joint_states_->load;
    gripper_result_.stalled = false;
    gripper_result_.reached_goal = true;
    
    trajectory_result_.error_code = trajectory_result_.SUCCESSFUL;

    gripper_action_server_.setSucceeded(gripper_result_);
    trajectory_action_server_.setSucceeded(trajectory_result_);
}

void DynamixelGripperNode::gripperCommandGoalCallback()
{
    double set_pos = gripper_action_server_.acceptNewGoal()->command.position;
    moveGripper(set_pos);
}

void DynamixelGripperNode::followJointTrajectoryGoalCallback()
{
    control_msgs::FollowJointTrajectoryGoal::ConstPtr goal = trajectory_action_server_.acceptNewGoal();
    const std::vector<trajectory_msgs::JointTrajectoryPoint> *points = &goal->trajectory.points;
    const trajectory_msgs::JointTrajectoryPoint *point = &points->back();
    double set_pos = point->positions.back();
    moveGripper(set_pos);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamixel_gripper");
    ros::NodeHandle nh;

    DynamixelGripperNode gripper(nh);

    ros::spin();

    return 0;
}
