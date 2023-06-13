/*
 * Copyright [2011] <Bonn-Rhein-Sieg University>
 *
 * teleop_joypad.cpp
 *
 *  Created on: May 27, 2012
 *      Author: Frederik Hegger
 */

#include <mir_teleop/teleop_joypad.h>
#include <string>

TeleOpJoypad::TeleOpJoypad(ros::NodeHandle &nh)
{
  nh_ = &nh;

  is_in_soft_joint_limits_ = false;
  button_deadman_pressed_prev_ = false;
  button_print_arm_states_prev_ = false;
  button_gripper_active_ = false;
  button_arm_motors_active_ = false;
  button_base_motors_active_ = false;
  button_arm_cart_pressed_prev_ = false;
  button_arm_joint_1_2_pressed_prev_ = false;
  button_arm_joint_3_4_pressed_prev_ = false;
  button_arm_joint_5_pressed_prev_ = false;
  is_one_arm_joint_button_pressed_ = false;

  dyn_recfg_cb = boost::bind(&TeleOpJoypad::cbDynamicReconfigure, this, _1, _2);
  dyn_recfg_server.setCallback(dyn_recfg_cb);

  if (!this->getJoypadConfigParameter()) {
    ROS_ERROR("could not get joypad parameters.");
    exit(0);
  }

  this->getBaseParameter();

  if (this->getArmParameter()) {
    ROS_INFO("Arm joint limit parameters available. Joint space control: ACTIVE");

    sub_joint_states_ = nh_->subscribe<sensor_msgs::JointState>("/joint_states", 1,
                                                                &TeleOpJoypad::cbJointStates, this);
    pub_arm_joint_vel_ = nh_->advertise<brics_actuator::JointVelocities>(
        "/arm_1/arm_controller/velocity_command", 1);
  } else
    ROS_ERROR(
        "No arm joint limit parameters available. Joint space control: "
        "DEACTIVATED.");

  sub_joypad_ = nh_->subscribe<sensor_msgs::Joy>("/joy", 1, &TeleOpJoypad::cbJoypad, this);
  pub_base_cart_vel_ = nh_->advertise<geometry_msgs::Twist>("cmd_vel", 1);
  pub_arm_cart_vel_ = nh_->advertise<geometry_msgs::TwistStamped>(
      "/arm_1/arm_controller/cartesian_velocity_command", 1);
  pub_gripper_command_ =
      nh_->advertise<mir_manipulation_msgs::GripperCommand>("gripper_command", 1);

  srv_base_motors_on_ = nh_->serviceClient<std_srvs::Empty>("/base/switchOnMotors");
  srv_base_motors_off_ = nh_->serviceClient<std_srvs::Empty>("/base/switchOffMotors");
  srv_arm_motors_on_ = nh_->serviceClient<std_srvs::Empty>("/arm_1/switchOnMotors");
  srv_arm_motors_off_ = nh_->serviceClient<std_srvs::Empty>("/arm_1/switchOffMotors");
  srv_reconnect = nh_->serviceClient<std_srvs::Empty>("/reconnect");
}

bool TeleOpJoypad::getJoypadConfigParameter()
{
  XmlRpc::XmlRpcValue name_list, index_list;
  std::string types[] = {"buttons", "axes"};

  for (int i = 0; i < 2; ++i)  // TBD
  {
    if (!nh_->getParam("joypad/" + types[i] + "/name", name_list) ||
        !nh_->getParam("joypad/" + types[i] + "/index", index_list)) {
      return false;
    }

    ROS_ASSERT(name_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(index_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(name_list.size() == index_list.size());

    for (int j = 0; j < name_list.size(); ++j) {
      if (i == 0) {
        if (name_list[j] == "deadman")
          button_index_deadman_ = index_list[j];
        else if (name_list[j] == "run")
          button_index_run_ = index_list[j];
        else if (name_list[j] == "base_motors_on_off")
          button_index_base_motors_on_off_ = index_list[j];
        else if (name_list[j] == "arm_motors_on_off")
          button_index_arm_motors_on_off_ = index_list[j];
        else if (name_list[j] == "reconnect_left")
          button_index_reconnect_left_ = index_list[j];
        else if (name_list[j] == "reconnect_right")
          button_index_reconnect_right_ = index_list[j];
        else if (name_list[j] == "arm_motor_1_2")
          button_index_arm_joint_1_2_ = index_list[j];
        else if (name_list[j] == "arm_motor_3_4")
          button_index_arm_joint_3_4_ = index_list[j];
        else if (name_list[j] == "arm_motor_5")
          button_index_arm_joint_5_ = index_list[j];
        else if (name_list[j] == "gripper")
          button_index_gripper_ = index_list[j];
        else if (name_list[j] == "arm_cart_ctrl")
          button_index_arm_cart_ = index_list[j];
        else if (name_list[j] == "print_arm_joint_states")
          button_index_print_arm_joint_states_ = index_list[j];
        else
          ROS_WARN_STREAM("button name <<" << name_list[j] << ">> in yaml but not used in node");
      }

      else if (i == 1) {
        if (name_list[j] == "base_linear_x")
          axes_index_base_linear_x_ = index_list[j];
        else if (name_list[j] == "base_linear_y")
          axes_index_base_linear_y_ = index_list[j];
        else if (name_list[j] == "base_angular_z")
          axes_index_base_angular_z_ = index_list[j];
        else if (name_list[j] == "arm_linear_x")
          axes_index_arm_linear_x_ = index_list[j];
        else if (name_list[j] == "arm_linear_y")
          axes_index_arm_linear_y_ = index_list[j];
        else if (name_list[j] == "arm_linear_z")
          axes_index_arm_linear_z_ = index_list[j];
        else if (name_list[j] == "arm_angular_x")
          axes_index_arm_angular_x_ = index_list[j];
        else if (name_list[j] == "arm_angular_y")
          axes_index_arm_angular_y_ = index_list[j];
        else if (name_list[j] == "arm_angular_z")
          axes_index_arm_angular_z_ = index_list[j];
        else if (name_list[j] == "arm_joint_axes_1")
          axes_index_arm_joint_axes_1_ = index_list[j];
        else if (name_list[j] == "arm_joint_axes_2")
          axes_index_arm_joint_axes_2_ = index_list[j];
        else
          ROS_WARN_STREAM("axes name <<" << name_list[j] << ">> in yaml but not used in node");
      }
    }
  }

  return true;
}

void TeleOpJoypad::getBaseParameter()
{
  double param = 0;
  ros::param::param<double>("~base_max_linear_x_vel", param, 0.3);
  base_cart_factor_.linear.x = param / MAX_JOYPAD;
  ros::param::param<double>("~base_max_linear_y_vel", param, 0.3);
  base_cart_factor_.linear.y = param / MAX_JOYPAD;
  ros::param::param<double>("~base_max_angular_vel", param, 0.5);
  base_cart_factor_.angular.z = param / MAX_JOYPAD;
}

bool TeleOpJoypad::getArmParameter()
{
  is_joint_space_ctrl_active_ = false;

  double param = 0;
  ros::param::param<double>("~arm_max_vel", param, 0.2);
  arm_max_vel_ = param / MAX_JOYPAD;
  ros::param::param<double>("~soft_joint_limit_threshold", soft_joint_limit_threshold_, 0.05);

  // read joint names
  XmlRpc::XmlRpcValue param_list;
  if (nh_->getParam("/arm_1/arm_controller/joints", param_list)) {
    ROS_ASSERT(param_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < param_list.size(); ++i) {
      ROS_ASSERT(param_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
      arm_joint_names_.push_back(static_cast<std::string>(param_list[i]));
    }

    // read joint limits
    for (unsigned int i = 0; i < arm_joint_names_.size(); ++i) {
      moveit_msgs::JointLimits limit;
      limit.joint_name = arm_joint_names_[i];
      nh_->getParam("/arm_1/arm_controller/limits/" + arm_joint_names_[i] + "/min",
                    limit.min_position);
      nh_->getParam("/arm_1/arm_controller/limits/" + arm_joint_names_[i] + "/max",
                    limit.max_position);
      arm_joint_limits_.push_back(limit);
    }

    arm_vel_.velocities.clear();
    for (unsigned int i = 0; i < arm_joint_names_.size(); ++i) {
      brics_actuator::JointValue joint_value;

      joint_value.timeStamp = ros::Time::now();
      joint_value.joint_uri = arm_joint_names_[i];
      joint_value.unit = "s^-1 rad";
      joint_value.value = 0.0;

      arm_vel_.velocities.push_back(joint_value);
    }

    is_joint_space_ctrl_active_ = true;
  }

  arm_cart_factor_ = 0.1;

  return is_joint_space_ctrl_active_;
}

bool TeleOpJoypad::moveGripper(int gripper_command)
{
  mir_manipulation_msgs::GripperCommand command_msgs;

  command_msgs.command = gripper_command;

  pub_gripper_command_.publish(command_msgs);

  return true;
}

void TeleOpJoypad::cbDynamicReconfigure(mir_teleop::TeleopJoypadConfig &config, uint32_t level)
{
  teleop_config_ = config;
  arm_cart_zero_vel_.header.frame_id = teleop_config_.arm_cartesian_reference_link;
}

void TeleOpJoypad::cbJointStates(const sensor_msgs::JointState::ConstPtr &state_msg)
{
  current_joint_states_ = *state_msg;
}

bool TeleOpJoypad::switchMotorsOnOff(std::string component_name, std::string state)
{
  std_srvs::Empty empty;
  ros::ServiceClient *srv_client;

  if (component_name == "arm" && state == "ON")
    srv_client = &srv_arm_motors_on_;
  else if (component_name == "arm" && state == "OFF")
    srv_client = &srv_arm_motors_off_;
  else if (component_name == "base" && state == "ON")
    srv_client = &srv_base_motors_on_;
  else if (component_name == "base" && state == "OFF")
    srv_client = &srv_base_motors_off_;
  else {
    ROS_ERROR_STREAM("component <<" << component_name << ">> or state <<" << state
                                    << ">> not known.");
    return false;
  }

  if (srv_client->call(empty)) {
    ROS_INFO_STREAM("Turned " << state << " " << component_name << " motors");
    return true;
  } else {
    ROS_ERROR_STREAM("Could not turn " << state << " " << component_name << " motors");
    return false;
  }

  return true;
}

bool TeleOpJoypad::reconnect()
{
  std_srvs::Empty empty;

  if (srv_reconnect.call(empty)) {
    ROS_INFO_STREAM("Call to serivce: " << srv_reconnect.getService() << " successful.");
    return true;
  }
  ROS_ERROR_STREAM("Could not call service: " << srv_reconnect.getService() << ".");
  return false;
}

void TeleOpJoypad::setSingleArmJointVel(double motor_vel, std::string joint_name)
{
  for (unsigned int i = 0; i < arm_vel_.velocities.size(); ++i) {
    if (arm_vel_.velocities[i].joint_uri == joint_name) {
      arm_vel_.velocities[i].timeStamp = ros::Time::now();
      arm_vel_.velocities[i].value = motor_vel;
    }
  }
}

void TeleOpJoypad::setAllArmJointVel(double motor_vel)
{
  for (unsigned int i = 0; i < arm_joint_names_.size(); ++i)
    setSingleArmJointVel(motor_vel, arm_joint_names_[i]);
}

void TeleOpJoypad::printArmJointStates()
{
  std::string joint_name_list = "";

  std::cout << "[";

  for (unsigned int i = 0; i < arm_joint_limits_.size(); i++) {
    for (unsigned int j = 0; j < current_joint_states_.name.size(); ++j) {
      if (current_joint_states_.name[j] == arm_joint_limits_[i].joint_name) {
        std::cout << current_joint_states_.position[j];
        joint_name_list += current_joint_states_.name[j];

        if (i < (arm_joint_limits_.size() - 1)) {
          std::cout << ", ";
          joint_name_list += ", ";
        }
      }
    }
  }
  std::cout << "] \t # current arm joint values (" << joint_name_list << ")" << std::endl;
}

void TeleOpJoypad::checkArmJointLimits()
{
  for (unsigned int i = 0; i < arm_joint_limits_.size(); i++) {
    for (unsigned int j = 0; j < current_joint_states_.name.size(); ++j) {
      if (current_joint_states_.name[j] == arm_joint_limits_[i].joint_name) {
        if (((current_joint_states_.position[j] <
              (arm_joint_limits_[i].min_position + soft_joint_limit_threshold_)) &&
             (arm_vel_.velocities[j].value < 0)) ||
            ((current_joint_states_.position[j] >
              (arm_joint_limits_[i].max_position - soft_joint_limit_threshold_)) &&
             (arm_vel_.velocities[j].value > 0))) {
          arm_vel_.velocities[i].value = 0.0;
        }
      }
    }
  }
}

void TeleOpJoypad::cbJoypad(const sensor_msgs::Joy::ConstPtr &command)
{
  is_one_arm_joint_button_pressed_ =
      static_cast<bool>(command->buttons[button_index_arm_joint_1_2_]) ||
      static_cast<bool>(command->buttons[button_index_arm_joint_3_4_]) ||
      static_cast<bool>(command->buttons[button_index_arm_joint_5_]);

  if (static_cast<bool>(command->buttons[button_index_deadman_])) {
    speed_factor_ = !static_cast<bool>(command->buttons[button_index_run_]) ? 0.5 : 1.0;

    // gripper control
    if (!button_gripper_pressed_prev_ &&
        static_cast<bool>(command->buttons[button_index_gripper_])) {
      button_gripper_active_ = !button_gripper_active_;
      if (button_gripper_active_) {
        ROS_INFO("open gripper");
        this->moveGripper(mir_manipulation_msgs::GripperCommand::OPEN);
      } else {
        ROS_INFO("close gripper");
        this->moveGripper(mir_manipulation_msgs::GripperCommand::CLOSE);
      }
    }

    // arm/base services
    if (!button_arm_motors_on_off_pressed_prev_ &&
        static_cast<bool>(command->buttons[button_index_arm_motors_on_off_])) {
      button_arm_motors_active_ = !button_arm_motors_active_;
      if (button_arm_motors_active_) {
        ROS_INFO("turn arm motors OFF");
        this->switchMotorsOnOff("arm", "OFF");
      } else {
        ROS_INFO("turn arm motors ON");
        this->switchMotorsOnOff("arm", "ON");
      }
    }

    if (!button_base_motors_on_off_pressed_prev_ &&
        static_cast<bool>(command->buttons[button_index_base_motors_on_off_])) {
      button_base_motors_active_ = !button_base_motors_active_;
      if (button_base_motors_active_) {
        ROS_INFO("turn base motors OFF");
        this->switchMotorsOnOff("base", "OFF");
      } else {
        ROS_INFO("turn base motors ON");
        this->switchMotorsOnOff("base", "ON");
      }
    }

    // arm cartesian control mode OR base cartesian control mode
    if (static_cast<bool>(command->buttons[button_index_arm_cart_])) {
      arm_cart_vel_.header.frame_id = teleop_config_.arm_cartesian_reference_link;
      arm_cart_vel_.twist.linear.x =
          command->axes[axes_index_arm_linear_x_] * arm_cart_factor_ * speed_factor_;
      arm_cart_vel_.twist.linear.y =
          command->axes[axes_index_arm_linear_y_] * arm_cart_factor_ * speed_factor_;
      arm_cart_vel_.twist.linear.z =
          command->axes[axes_index_arm_linear_z_] * arm_cart_factor_ * speed_factor_;

      arm_cart_vel_.twist.angular.x =
          command->axes[axes_index_arm_angular_x_] * arm_cart_factor_ * speed_factor_;
      arm_cart_vel_.twist.angular.y =
          command->axes[axes_index_arm_angular_y_] * arm_cart_factor_ * speed_factor_;
      arm_cart_vel_.twist.angular.z =
          command->axes[axes_index_arm_angular_z_] * arm_cart_factor_ * speed_factor_;
    } else {
      base_cart_vel_.linear.x =
          command->axes[axes_index_base_linear_x_] * base_cart_factor_.linear.x * speed_factor_;
      base_cart_vel_.linear.y =
          command->axes[axes_index_base_linear_y_] * base_cart_factor_.linear.y * speed_factor_;
      base_cart_vel_.angular.z =
          command->axes[axes_index_base_angular_z_] * base_cart_factor_.angular.z * speed_factor_;

      if (fabs(base_cart_vel_.linear.x) < 0.01) base_cart_vel_.linear.x = 0.0;
      if (fabs(base_cart_vel_.linear.y) < 0.01) base_cart_vel_.linear.y = 0.0;
      if (fabs(base_cart_vel_.angular.z) < 0.01) base_cart_vel_.angular.z = 0.0;
    }

    // arm joint space control mode
    if (is_joint_space_ctrl_active_) {
      if (button_arm_joint_1_2_pressed_prev_ &&
          !static_cast<bool>(command->buttons[button_index_arm_joint_1_2_])) {
        this->setSingleArmJointVel(0.0, arm_joint_names_[0]);
        this->setSingleArmJointVel(0.0, arm_joint_names_[1]);
        pub_arm_joint_vel_.publish(arm_vel_);
      }
      if (button_arm_joint_3_4_pressed_prev_ &&
          !static_cast<bool>(command->buttons[button_index_arm_joint_3_4_])) {
        this->setSingleArmJointVel(0.0, arm_joint_names_[2]);
        this->setSingleArmJointVel(0.0, arm_joint_names_[3]);
        pub_arm_joint_vel_.publish(arm_vel_);
      }
      if (button_arm_joint_5_pressed_prev_ &&
          !static_cast<bool>(command->buttons[button_index_arm_joint_5_])) {
        this->setSingleArmJointVel(0.0, arm_joint_names_[4]);
        pub_arm_joint_vel_.publish(arm_vel_);
      }

      if (static_cast<bool>(command->buttons[button_index_arm_joint_1_2_])) {
        arm_vel_.velocities[0].value =
            command->axes[axes_index_arm_joint_axes_1_] * arm_max_vel_ * speed_factor_ * (-1.0);
        arm_vel_.velocities[1].value =
            command->axes[axes_index_arm_joint_axes_2_] * arm_max_vel_ * speed_factor_;
      } else if (static_cast<bool>(command->buttons[button_index_arm_joint_3_4_])) {
        arm_vel_.velocities[2].value =
            command->axes[axes_index_arm_joint_axes_1_] * arm_max_vel_ * speed_factor_;
        arm_vel_.velocities[3].value =
            command->axes[axes_index_arm_joint_axes_2_] * arm_max_vel_ * speed_factor_;
      } else if (static_cast<bool>(command->buttons[button_index_arm_joint_5_]))
        arm_vel_.velocities[4].value =
            command->axes[axes_index_arm_joint_axes_1_] * arm_max_vel_ * speed_factor_ * (-1.0);
    }

    if (button_arm_cart_pressed_prev_ &&
        !static_cast<bool>(command->buttons[button_index_arm_cart_]))
      pub_arm_cart_vel_.publish(arm_cart_zero_vel_);
    else if (static_cast<bool>(command->buttons[button_index_arm_cart_]))
      pub_arm_cart_vel_.publish(arm_cart_vel_);
    else if (!static_cast<bool>(command->buttons[button_index_arm_cart_]))
      pub_base_cart_vel_.publish(base_cart_vel_);

    if (is_joint_space_ctrl_active_ && is_one_arm_joint_button_pressed_) {
      this->checkArmJointLimits();
      pub_arm_joint_vel_.publish(arm_vel_);
    }

    if (static_cast<bool>(command->buttons[button_index_reconnect_left_]) &&
        static_cast<bool>(command->buttons[button_index_reconnect_right_])) {
      this->reconnect();
    }
  }

  else {
    if (button_deadman_pressed_prev_) {
      if (is_joint_space_ctrl_active_) {
        this->setAllArmJointVel(0.0);
        pub_arm_joint_vel_.publish(arm_vel_);
      }

      pub_base_cart_vel_.publish(base_cart_zero_vel_);
      pub_arm_cart_vel_.publish(arm_cart_zero_vel_);
    }
  }

  // check if arm is in joint mode (not cc mode)
  if (!button_print_arm_states_prev_ &&
      static_cast<bool>(command->buttons[button_index_print_arm_joint_states_]))
    this->printArmJointStates();

  // remember buttons states
  button_deadman_pressed_prev_ = static_cast<bool>(command->buttons[button_index_deadman_]);
  button_gripper_pressed_prev_ = static_cast<bool>(command->buttons[button_index_gripper_]);
  button_arm_motors_on_off_pressed_prev_ =
      static_cast<bool>(command->buttons[button_index_arm_motors_on_off_]);
  button_base_motors_on_off_pressed_prev_ =
      static_cast<bool>(command->buttons[button_index_base_motors_on_off_]);
  button_print_arm_states_prev_ =
      static_cast<bool>(command->buttons[button_index_print_arm_joint_states_]);
  button_arm_cart_pressed_prev_ = static_cast<bool>(command->buttons[button_index_arm_cart_]);
  button_arm_joint_1_2_pressed_prev_ =
      static_cast<bool>(command->buttons[button_index_arm_joint_1_2_]);
  button_arm_joint_3_4_pressed_prev_ =
      static_cast<bool>(command->buttons[button_index_arm_joint_3_4_]);
  button_arm_joint_5_pressed_prev_ = static_cast<bool>(command->buttons[button_index_arm_joint_5_]);
}
