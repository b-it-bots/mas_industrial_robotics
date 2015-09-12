/*
 * Copyright [2015] <Bonn-Rhein-Sieg University>
 *
 * dynamixel_gripper_node.h
 *
 *  Created on: Apr 14, 2015
 *      Author: fred
 */

#ifndef MIR_GRIPPER_CONTROLLER_DYNAMIXEL_GRIPPER_NODE_H_
#define MIR_GRIPPER_CONTROLLER_DYNAMIXEL_GRIPPER_NODE_H_

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <dynamixel_controllers/SetTorqueLimit.h>
#include <dynamixel_msgs/JointState.h>
#include <limits.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <deque>
#include <string>

class DynamixelGripperNode
{
public:
    explicit DynamixelGripperNode(ros::NodeHandle &nh);
    ~DynamixelGripperNode();

private:
    void jointStatesCallback(const dynamixel_msgs::JointState::Ptr &msg);
    void gripperCommandGoalCallback();

    double getAverage(const std::deque<double> &queue);

    ros::NodeHandle nh_;

    ros::Publisher pub_dynamixel_command_;
    ros::Publisher pub_joint_states_;
    ros::Subscriber sub_dynamixel_motor_states_;

    actionlib::SimpleActionServer<control_msgs::GripperCommandAction> action_server_;
    control_msgs::GripperCommandFeedback gripper_feedback_;
    control_msgs::GripperCommandResult gripper_result_;

    dynamixel_msgs::JointState::Ptr joint_states_;
    bool joint_states_received_;

    std::deque<double> prev_positions_;
    std::deque<double> prev_velocities_;

    // Parameters
    double soft_torque_limit_;

    std::string hard_torque_limit_srv_name_;
    double hard_torque_limit_;

    double position_threshold_;
    int queue_size_;
};

#endif  // MIR_GRIPPER_CONTROLLER_DYNAMIXEL_GRIPPER_NODE_H_

