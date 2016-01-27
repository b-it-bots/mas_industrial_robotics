/*
 * Copyright [2015] <Bonn-Rhein-Sieg University>
 *
 * dynamixel_gripper_node.cpp
 *
 *  Created on: Apr 14, 2015
 *      Author: Frederik Hegger
 */

#include <deque>
#include <string>

#include <mir_gripper_controller/dynamixel_gripper_node.h>

DynamixelGripperNode::DynamixelGripperNode(ros::NodeHandle &nh) :
    action_server_(nh, "gripper_controller", false), joint_states_received_(false)
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
    nh_prv.param("soft_torque_limit", soft_torque_limit_, 0.5);
    ROS_INFO_STREAM("\tSoft torque limit: " << soft_torque_limit_);

    nh_prv.param<std::string>("hard_torque_limit_srv_name", hard_torque_limit_srv_name_, "set_torque_limit");
    nh_prv.param<double>("hard_torque_limit", hard_torque_limit_, 1.0);
    ROS_INFO_STREAM("\tHard torque limit srv name: " << hard_torque_limit_srv_name_);
    ROS_INFO_STREAM("\tHard torque limit: " << hard_torque_limit_);

    nh_prv.param<double>("position_threshold", position_threshold_, 0.05);
    ROS_INFO_STREAM("\tPosition threshold: " << position_threshold_);

    nh_prv.param<int>("queue_size", queue_size_, 10);
    ROS_INFO_STREAM("\tQueue size: " << queue_size_);

    nh_prv.param<double>("gripper_configuration_open", gripper_configuration_open_, 0.0);
    ROS_INFO_STREAM("\tgripper configuration <open>: " << gripper_configuration_open_);

    nh_prv.param<double>("gripper_configuration_close", gripper_configuration_close_, 1.0);
    ROS_INFO_STREAM("\tgripper configuration <close>: " << gripper_configuration_close_);

    // set the hard torque limit
    dynamixel_controllers::SetTorqueLimit torque_srv;
    ros::ServiceClient srv_client_torque = nh_.serviceClient<dynamixel_controllers::SetTorqueLimit>(
            hard_torque_limit_srv_name_);

    torque_srv.request.torque_limit = hard_torque_limit_;

    ros::service::waitForService(hard_torque_limit_srv_name_, ros::Duration(10.0));
    while (!srv_client_torque.call(torque_srv))
    {
        ROS_ERROR_STREAM("Failed to call service: " << hard_torque_limit_srv_name_ << "! Will try again ...");
        sleep(1);
    }

    // start action server
    action_server_.registerGoalCallback(boost::bind(&DynamixelGripperNode::gripperCommandGoalCallback, this));
    action_server_.start();
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

    joint_state.name.push_back("gripper_finger_joint_r");
    joint_state.position.push_back(msg->current_pos);
    joint_state.velocity.push_back(msg->velocity);
    joint_state.effort.push_back(msg->load);

    joint_state.name.push_back("gripper_finger_joint_l");
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

double DynamixelGripperNode::getAverage(const std::deque<double> &queue)
{
    double average = 0.0;

    for (size_t i = 0; i < queue.size(); i++)
        average += queue.at(i);

    average /= queue.size();

    return average;
}


void DynamixelGripperNode::gripperCommandGoalCallback()
{
    ros::Rate loop_rate(100);

    // accept and get goal position
    double set_pos = action_server_.acceptNewGoal()->command.position;

    // publish goal position
    std_msgs::Float64 gripper_pos;
    gripper_pos.data = set_pos;
    pub_dynamixel_command_.publish(gripper_pos);

    // empty queue
    prev_positions_.clear();
    prev_velocities_.clear();

    // wait until position or max. torque is reached
    while (ros::ok())
    {
        joint_states_received_ = false;
        while (!joint_states_received_ && ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }

        if (joint_states_->load > soft_torque_limit_)
        {
            gripper_pos.data = joint_states_->current_pos;
            pub_dynamixel_command_.publish(gripper_pos);
            ROS_WARN_STREAM("Torque soft limit reached (cur: " << joint_states_->load <<  ", limit: " <<
                            soft_torque_limit_ <<  ")  " << set_pos << " reached");

            break;
        }

        ROS_DEBUG_STREAM("Position difference: " << (joint_states_->current_pos - set_pos));

        if (fabs(joint_states_->current_pos - set_pos) < position_threshold_)
        {
            ROS_INFO_STREAM("Position " << set_pos << " reached");
            break;
        }

        if (prev_positions_.size() < queue_size_)
        {
            prev_positions_.push_back(joint_states_->current_pos);
            prev_velocities_.push_back(joint_states_->velocity);
        }
        else
        {
            prev_positions_.pop_front();
            prev_positions_.push_back(joint_states_->current_pos);

            prev_velocities_.pop_front();
            prev_velocities_.push_back(joint_states_->velocity);

            if ((fabs(getAverage(prev_positions_) - joint_states_->current_pos) < position_threshold_) &&
                    (getAverage(prev_velocities_) == joint_states_->velocity))
            {
                gripper_pos.data = joint_states_->current_pos;
                pub_dynamixel_command_.publish(gripper_pos);
                ROS_WARN_STREAM("Position does not change anymore");

                break;
            }
        }
    }

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

    action_server_.setSucceeded(gripper_result_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamixel_gripper");
    ros::NodeHandle nh;

    DynamixelGripperNode gripper(nh);

    ros::spin();

    return 0;
}
