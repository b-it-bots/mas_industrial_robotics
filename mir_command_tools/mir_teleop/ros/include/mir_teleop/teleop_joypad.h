/*
 * Copyright [2012] <Bonn-Rhein-Sieg University>
 *
 * teleop_joypad.h
 *
 *  Created on: May 27, 2012
 *      Author: Frederik Hegger
 */

#ifndef MIR_TELEOP_TELEOP_JOYPAD_H_
#define MIR_TELEOP_TELEOP_JOYPAD_H_

#include <string>
#include <vector>

#include "brics_actuator/msg/joint_velocities.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "moveit_msgs/msg/joint_limits.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/empty.hpp"
#include "mir_interfaces/msg/gripper_command.hpp"

#define MAX_JOYPAD 1.0

class TeleOpJoypad : public rclcpp::Node
{
 public:
  TeleOpJoypad();

 private:
  bool getJoypadConfigParameter();
  void getBaseParameter();
  bool getArmParameter();
  bool moveGripper(int gripper_command);
  bool switchMotorsOnOff(std::string component_name, std::string state);
  bool reconnect();

  void cbJoypad(const sensor_msgs::msg::Joy::SharedPtr command);
  void cbJointStates(const sensor_msgs::msg::JointState::SharedPtr state_msg);
  void setAllArmJointVel(double motor_vel);
  void setSingleArmJointVel(double motor_vel, std::string joint_name);
  void checkArmJointLimits();
  void printArmJointStates();

  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

  sensor_msgs::msg::JointState::SharedPtr current_joint_states_;
  bool is_in_soft_joint_limits_;
  double soft_joint_limit_threshold_;
  double arm_max_vel_;
  std::vector<std::string> arm_joint_names_;
  std::vector<moveit_msgs::msg::JointLimits> arm_joint_limits_;
  brics_actuator::msg::JointVelocities arm_vel_;
  bool is_joint_space_ctrl_active_;

  geometry_msgs::msg::TwistStamped arm_cart_vel_;
  double arm_cart_factor_;
  std::string arm_cartesian_reference_link_;

  double speed_factor_;

  bool button_deadman_pressed_prev_;
  bool button_print_arm_states_prev_;

  bool button_gripper_pressed_prev_;
  bool button_gripper_active_;

  bool button_arm_motors_on_off_pressed_prev_;
  bool button_arm_motors_active_;

  bool button_base_motors_on_off_pressed_prev_;
  bool button_base_motors_active_;

  bool button_reconnect_left_pressed_prev_;
  bool button_reconnect_right_pressed_prev_;

  bool button_arm_cart_pressed_prev_;

  bool button_arm_joint_1_2_pressed_prev_;
  bool button_arm_joint_3_4_pressed_prev_;
  bool button_arm_joint_5_pressed_prev_;

  bool is_one_arm_joint_button_pressed_;

  geometry_msgs::msg::Twist base_cart_vel_;
  geometry_msgs::msg::Twist base_cart_zero_vel_;
  geometry_msgs::msg::Twist base_cart_factor_;

  geometry_msgs::msg::TwistStamped arm_cart_zero_vel_;

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joypad_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_base_cart_vel_;
  rclcpp::Publisher<brics_actuator::msg::JointVelocities>::SharedPtr  pub_arm_joint_vel_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr  pub_arm_cart_vel_;
  rclcpp::Publisher<mir_interfaces::msg::GripperCommand>::SharedPtr  pub_gripper_command_;

  // Service clients
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr srv_arm_motors_on_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr srv_arm_motors_off_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr srv_base_motors_on_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr srv_base_motors_off_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr srv_reconnect;

  int button_index_deadman_;
  int button_index_run_;
  int button_index_arm_motors_on_off_;
  int button_index_base_motors_on_off_;
  int button_index_reconnect_left_;
  int button_index_reconnect_right_;
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

#endif  // MIR_TELEOP_TELEOP_JOYPAD_H_
