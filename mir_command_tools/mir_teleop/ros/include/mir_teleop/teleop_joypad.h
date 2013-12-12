/*
 * teleop_joypad.h
 *
 *  Created on: May 27, 2012
 *      Author: Frederik Hegger
 */

#ifndef TELEOP_JOYPAD_H_
#define TELEOP_JOYPAD_H_

#include <string>
#include <vector>

#include <moveit_msgs/JointLimits.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointPositions.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>

#include <boost/units/systems/si.hpp>
#include <boost/units/io.hpp>

#define MAX_JOYPAD						1.0

class TeleOpJoypad
{
 public:
	TeleOpJoypad(ros::NodeHandle &nh);

 private:
	bool getJoypadConfigParameter();
	void getBaseParameter();
	bool getArmParameter();
	bool moveGripper(std::string joint_position_name);
	bool switchMotorsOnOff(std::string component_name, std::string state);
	bool reconnect();

	void cbJoypad(const sensor_msgs::Joy::ConstPtr& command);
	void cbJointStates(const sensor_msgs::JointState::ConstPtr& state_msg);
	void setAllArmJointVel(double motor_vel);
	void setSingleArmJointVel(double motor_vel, std::string joint_name);
	void checkArmJointLimits();
	void printArmJointStates();

	ros::NodeHandle* nh_;

	sensor_msgs::JointState current_joint_states_;
	bool is_in_soft_joint_limits_;
	double soft_joint_limit_threshold_;
	double arm_max_vel_;
	std::vector<std::string> arm_joint_names_;
	std::vector<moveit_msgs::JointLimits> arm_joint_limits_;
	brics_actuator::JointVelocities arm_vel_;
	bool is_joint_space_ctrl_active_;

	geometry_msgs::TwistStamped arm_cart_vel_;
	double arm_cart_factor_;

	double speed_factor_;

	bool button_deadman_pressed_prev_;
	bool button_print_arm_states_prev_;

	bool button_gripper_pressed_prev_;
	bool button_gripper_active_;

	bool button_arm_motors_on_off_pressed_prev_;
	bool button_arm_motors_active_;

	bool button_base_motors_on_off_pressed_prev_;
	bool button_base_motors_active_;

	geometry_msgs::Twist base_cart_vel_;
	geometry_msgs::Twist base_cart_zero_vel_;
	geometry_msgs::Twist base_cart_factor_;

	// Subscriber
	ros::Subscriber sub_joypad_;
	ros::Subscriber sub_joint_states_;

	// Publisher
	ros::Publisher pub_base_cart_vel_;
	ros::Publisher pub_arm_joint_vel_;
	ros::Publisher pub_arm_cart_vel_;
	ros::Publisher pub_gripper_position_;

	// Service clients
	ros::ServiceClient srv_arm_motors_on_;
	ros::ServiceClient srv_arm_motors_off_;
	ros::ServiceClient srv_base_motors_on_;
	ros::ServiceClient srv_base_motors_off_;
	ros::ServiceClient srv_reconnect;

	int button_index_deadman_;
	int button_index_run_;
	int button_index_arm_motors_on_off_;
	int button_index_base_motors_on_off_;
	int button_index_print_arm_joint_states_;
	int button_index_arm_joint_1_2_;
	int button_index_arm_joint_3_4_;
	int button_index_arm_joint_5_;
	int button_index_gripper_;
	int button_index_arm_cart_;

	int axes_index_base_linear_x_;
	int axes_index_base_linear_y_;
	int axes_index_base_angular_z_;
	int axes_index_arm_linear_x_;
	int axes_index_arm_linear_y_;
	int axes_index_arm_linear_z_;
	int axes_index_arm_angular_x_;
	int axes_index_arm_angular_y_;
	int axes_index_arm_angular_z_;
	int axes_index_arm_joint_axes_1_;
	int axes_index_arm_joint_axes_2_;
};

#endif /* TELEOP_JOYPAD_H_ */
