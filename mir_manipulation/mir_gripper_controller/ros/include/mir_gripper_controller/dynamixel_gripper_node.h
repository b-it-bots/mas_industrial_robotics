/*
 * dynamixel_gripper_node.h
 *
 *  Created on: Apr 14, 2015
 *      Author: fred
 */

#ifndef DYNAMIXEL_GRIPPER_NODE_H_
#define DYNAMIXEL_GRIPPER_NODE_H_

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <dynamixel_msgs/JointState.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

class DynamixelGripperNode
{
public:
    DynamixelGripperNode(ros::NodeHandle &nh);
    ~DynamixelGripperNode();

private:
    void jointStatesCallback(const dynamixel_msgs::JointState::Ptr &msg);
    void gripperCommandGoalCallback();

    double mapFromRadiansToMeter(const double &radians);

    ros::Publisher pub_dynamixel_command_;
    ros::Publisher pub_joint_states_;
    ros::Subscriber sub_dynamixel_motor_states_;

    actionlib::SimpleActionServer<control_msgs::GripperCommandAction> action_server_;
    control_msgs::GripperCommandFeedback gripper_feedback_;
    control_msgs::GripperCommandResult gripper_result_;


    dynamixel_msgs::JointState::Ptr joint_states_;
    bool joint_states_received_;

    ros::NodeHandle nh_;

    double soft_torque_limit_;
 
};

#endif /* DYNAMIXEL_GRIPPER_NODE_H_ */

