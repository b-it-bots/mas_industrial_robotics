/*
 * dynamixel_gripper_grasp_monitor_node.cpp
 *
 *  Created on: Jul 11, 2015
 *      Author: Frederik Hegger
 */

#include <mir_grasp_monitors/dynamixel_gripper_grasp_monitor_node.h>

DynamixelGripperGraspMonitorNode::DynamixelGripperGraspMonitorNode() :
	joint_states_received_(false),
	event_in_received_(false),
	current_state_(INIT),
	loop_rate_init_state_(ros::Rate(10.0))
{
	ros::NodeHandle nh("~");

	pub_event_ = nh.advertise<std_msgs::String>("event_out", 1);
	sub_event_ = nh.subscribe("event_in", 10, &DynamixelGripperGraspMonitorNode::eventCallback, this);
	sub_dynamixel_motor_states_ = nh.subscribe("dynamixel_motor_states", 10, &DynamixelGripperGraspMonitorNode::jointStatesCallback, this);
}

DynamixelGripperGraspMonitorNode::~DynamixelGripperGraspMonitorNode()
{
    pub_event_.shutdown();
    sub_event_.shutdown();
    sub_dynamixel_motor_states_.shutdown();
}

void DynamixelGripperGraspMonitorNode::jointStatesCallback(const dynamixel_msgs::JointState::Ptr &msg)
{
    joint_states_ = msg;
    joint_states_received_ = true;
}

void DynamixelGripperGraspMonitorNode::eventCallback(const std_msgs::String::ConstPtr &msg)
{
    event_in_ = *msg;
    event_in_received_ = true;
}

void DynamixelGripperGraspMonitorNode::update()
{
	checkForNewEvent();

	switch(current_state_)
	{
		case INIT: init_state(); break;
		case IDLE: idle_state(); break;
		case RUN: run_state(); break;
	}
}

void DynamixelGripperGraspMonitorNode::checkForNewEvent()
{
	if(!event_in_received_)
		return;

	ROS_INFO_STREAM("Received event: " << event_in_.data);

	if(event_in_.data == "e_trigger")
		current_state_ = IDLE;
	else
		ROS_ERROR_STREAM("Event not supported: " << event_in_.data);

	event_in_received_ = false;
}

void DynamixelGripperGraspMonitorNode::init_state()
{
	loop_rate_init_state_.sleep();
}

void DynamixelGripperGraspMonitorNode::idle_state()
{
	// wait for incoming data
	if(joint_states_received_)
		current_state_ = RUN;

	joint_states_received_ = false;
}

void DynamixelGripperGraspMonitorNode::run_state()
{
	std_msgs::String event_out;

	if(isObjectGrasped())
		event_out.data = "e_object_grasped";
	else
		event_out.data = "e_object_not_grasped";

	pub_event_.publish(event_out);

	current_state_ = INIT;
}

bool DynamixelGripperGraspMonitorNode::isObjectGrasped()
{
	if(joint_states_->load > 0.0)
		return true;

	return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grasp_monitor");
    ros::NodeHandle nh("~");

    DynamixelGripperGraspMonitorNode grasp_monitor;

    ros::Rate loop_rate(100);

    while(ros::ok())
    {
    	ros::spinOnce();

    	grasp_monitor.update();

    	loop_rate.sleep();
    }


    return 0;
}
