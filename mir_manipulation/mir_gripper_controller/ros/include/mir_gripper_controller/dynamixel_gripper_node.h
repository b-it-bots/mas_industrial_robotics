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
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <dynamixel_controllers/SetTorqueLimit.h>
#include <dynamixel_controllers/SetComplianceMargin.h>
#include <dynamixel_controllers/SetComplianceSlope.h>
#include <dynamixel_msgs/JointState.h>
#include <mcr_manipulation_msgs/GripperCommand.h>
#include <limits.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <string>

class DynamixelGripperNode
{
public:
    explicit DynamixelGripperNode(ros::NodeHandle &nh);
    ~DynamixelGripperNode();

private:
    void jointStatesCallback(const dynamixel_msgs::JointState::Ptr &msg);
    void gripperCommandCallback(const mcr_manipulation_msgs::GripperCommand::Ptr &msg);
    void followJointTrajectoryGoalCallback();
    void gripperCommandGoalCallback();
    
    void moveGripper(double position);

    ros::NodeHandle nh_;

    ros::Publisher pub_dynamixel_command_;
    ros::Publisher pub_joint_states_;
    ros::Subscriber sub_dynamixel_motor_states_;
    ros::Subscriber sub_gripper_command_;

    actionlib::SimpleActionServer<control_msgs::GripperCommandAction> gripper_action_server_;
    control_msgs::GripperCommandFeedback gripper_feedback_;
    control_msgs::GripperCommandResult gripper_result_;
    
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> trajectory_action_server_;
    control_msgs::FollowJointTrajectoryFeedback trajectory_feedback_;
    control_msgs::FollowJointTrajectoryResult trajectory_result_;

    dynamixel_msgs::JointState::Ptr joint_states_;
    bool joint_states_received_;

    // Parameters
    double torque_limit_;
    std::string torque_limit_srv_name_;
    int compliance_margin_;
    std::string compliance_margin_srv_name_;
    int compliance_slope_;
    std::string compliance_slope_srv_name_;
    double gripper_configuration_open_;
    double gripper_configuration_close_;
};

#endif  // MIR_GRIPPER_CONTROLLER_DYNAMIXEL_GRIPPER_NODE_H_

