/*
 * Copyright [2022] <Bonn-Rhein-Sieg University>
 *
 * teleop_joypad.cpp
 *
 *  Created on: May 27, 2012
 *      Author: Frederik Hegger
 *  
 *  ROS2 contributors: Santosh Thoduka, Vamsi Kalagaturu, Vivek Mannava.
 */

#include <mir_teleop/teleop_joypad.h>
#include <string>

TeleOpJoypad::TeleOpJoypad() :
    Node("teleop_joypad", rclcpp::NodeOptions().allow_undeclared_parameters(true))
{
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

  rcl_interfaces::msg::ParameterDescriptor arm_cart_ref_link_descriptor;
  arm_cart_ref_link_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  arm_cart_ref_link_descriptor.description = "Reference link which will be used by the arm Cartesian controller";
  this->declare_parameter("arm_cartesian_reference_link", "/base_link", arm_cart_ref_link_descriptor);

  callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&TeleOpJoypad::parametersCallback, this, std::placeholders::_1));

  if (!this->getJoypadConfigParameter()) {
    RCLCPP_ERROR(this->get_logger(), "could not get joypad parameters.");
    exit(0);
  }

  this->getBaseParameter();

  if (this->getArmParameter()) {
    RCLCPP_INFO(this->get_logger(), "Arm joint limit parameters available. Joint space control: ACTIVE");

    sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
                        "/joint_states", 1, std::bind(&TeleOpJoypad::cbJointStates, this, std::placeholders::_1));
    pub_arm_joint_vel_ = this->create_publisher<brics_actuator::msg::JointVelocities>(
        "/arm_1/arm_controller/velocity_command", 1);
  } else
    RCLCPP_ERROR(this->get_logger(),
        "No arm joint limit parameters available. Joint space control: "
        "DEACTIVATED.");

  sub_joypad_ = this->create_subscription<sensor_msgs::msg::Joy>(
                      "/joy", 1, std::bind(&TeleOpJoypad::cbJoypad, this, std::placeholders::_1));
  pub_base_cart_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
                        "/cmd_vel", 1);
  pub_arm_cart_vel_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
          "/arm_1/arm_controller/cartesian_velocity_command", 1);
  pub_gripper_command_ = this->create_publisher<mir_interfaces::msg::GripperCommand>("/arm_1/gripper_command", 1);

  srv_base_motors_on_ = this->create_client<std_srvs::srv::Empty>("/base/switchOnMotors");
  srv_base_motors_off_ = this->create_client<std_srvs::srv::Empty>("/base/switchOffMotors");
  srv_arm_motors_on_ = this->create_client<std_srvs::srv::Empty>("/arm_1/switchOnMotors");
  srv_arm_motors_off_ = this->create_client<std_srvs::srv::Empty>("/arm_1/switchOffMotors");
  srv_reconnect = this->create_client<std_srvs::srv::Empty>("/reconnect");
}

bool TeleOpJoypad::getJoypadConfigParameter()
{
  std::string types[] = {"buttons", "axes"};

  for (int i = 0; i < 2; ++i)  // TBD
  {
    std::string name_param = "joypad." + types[i] + ".name";
    std::string index_param = "joypad." + types[i] + ".index";
    rcl_interfaces::msg::ParameterDescriptor name_param_descriptor;
    name_param_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
    this->declare_parameter(name_param, std::vector<std::string>({}), name_param_descriptor);
    rcl_interfaces::msg::ParameterDescriptor index_param_descriptor;
    index_param_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
    this->declare_parameter(index_param, std::vector<int>({}), index_param_descriptor);
    rclcpp::Parameter name_list_param(name_param, std::vector<std::string>({}));
    rclcpp::Parameter index_list_param(index_param, std::vector<int>({}));
    this->get_parameter(name_param, name_list_param);
    this->get_parameter(index_param, index_list_param);
    std::vector<std::string> name_list = name_list_param.as_string_array();
    auto index_list = index_list_param.as_integer_array();
    if (name_list.empty() or index_list.empty())
    {
        return false;
    }

    assert(name_list.size() == index_list.size());

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
          RCLCPP_WARN_STREAM(this->get_logger(), "button name <<" << name_list[j] << ">> in yaml but not used in node");
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
          RCLCPP_WARN_STREAM(this->get_logger(), "axes name <<" << name_list[j] << ">> in yaml but not used in node");
      }
    }
  }

  return true;
}

void TeleOpJoypad::getBaseParameter()
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;

  double param = 0;
  param = this->declare_parameter<double>("base_max_linear_x_vel", 0.3, descriptor);
  base_cart_factor_.linear.x = param / MAX_JOYPAD;

  param = this->declare_parameter<double>("base_max_linear_y_vel", 0.3, descriptor);
  base_cart_factor_.linear.y = param / MAX_JOYPAD;

  param = this->declare_parameter<double>("base_max_angular_vel", 0.3, descriptor);
  base_cart_factor_.angular.z = param / MAX_JOYPAD;
}

bool TeleOpJoypad::getArmParameter()
{
  arm_cartesian_reference_link_ = this->get_parameter("arm_cartesian_reference_link").as_string();

  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;

  is_joint_space_ctrl_active_ = false;

  double param = 0;
  param = this->declare_parameter<double>("arm_max_vel", 0.2, descriptor);
  arm_max_vel_ = param / MAX_JOYPAD;

  soft_joint_limit_threshold_ = this->declare_parameter<double>("soft_joint_limit_threshold", 0.05, descriptor);

  // declare arm parameters
  auto arm_joint_names_param = this->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>({}));
  for (auto i : arm_joint_names_param){
    arm_joint_names_.push_back(i);
  }

  // read joint limits
  for (unsigned int i = 0; i < arm_joint_names_.size(); ++i) {
    moveit_msgs::msg::JointLimits limit;
    limit.joint_name = arm_joint_names_[i];
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;

    double param = 0;
    param = this->declare_parameter<double>("limits." + arm_joint_names_[i] + ".min", 0.0, descriptor);
    limit.min_position = param;
    param = this->declare_parameter<double>("limits." + arm_joint_names_[i] + ".max", 0.0, descriptor);
    limit.max_position = param;
    arm_joint_limits_.push_back(limit);
  }

  arm_vel_.velocities.clear();
  for (unsigned int i = 0; i < arm_joint_names_.size(); ++i) {
    brics_actuator::msg::JointValue joint_value;

    joint_value.timestamp = this->now();
    joint_value.joint_uri = arm_joint_names_[i];
    joint_value.unit = "s^-1 rad";
    joint_value.value = 0.0;

    arm_vel_.velocities.push_back(joint_value);
  }

  is_joint_space_ctrl_active_ = true;

  arm_cart_factor_ = 0.1;

  return is_joint_space_ctrl_active_;
}

bool TeleOpJoypad::moveGripper(int gripper_command)
{
  mir_interfaces::msg::GripperCommand command_msg;

  command_msg.command = gripper_command;

  pub_gripper_command_ -> publish(command_msg);

  return true;
}

void TeleOpJoypad::cbJointStates(const sensor_msgs::msg::JointState::SharedPtr state_msg)
{
  current_joint_states_ = state_msg;
}

bool TeleOpJoypad::switchMotorsOnOff(std::string component_name, std::string state)
{
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr srv_client;

  if (component_name == "arm" && state == "ON")
    srv_client = srv_arm_motors_on_;
  else if (component_name == "arm" && state == "OFF")
    srv_client = srv_arm_motors_off_;
  else if (component_name == "base" && state == "ON")
    srv_client = srv_base_motors_on_;
  else if (component_name == "base" && state == "OFF")
    srv_client = srv_base_motors_off_;
  else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "component <<" << component_name << ">> or state <<" << state
                                    << ">> not known.");
    return false;
  }

  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto result = srv_client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Turned " << state << " " << component_name << " motors");
    return true;
  }
  else
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not turn " << state << " " << component_name << " motors");
    return false;
  }

  return true;
}

bool TeleOpJoypad::reconnect()
{
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto result = srv_reconnect->async_send_request(request);

  if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Call to serivce: " << srv_reconnect->get_service_name() << " successful.");
    return true;
  }
  RCLCPP_ERROR_STREAM(this->get_logger(), "Could not call service: " << srv_reconnect->get_service_name() << ".");
  return false;
}

void TeleOpJoypad::setSingleArmJointVel(double motor_vel, std::string joint_name)
{
  for (unsigned int i = 0; i < arm_vel_.velocities.size(); ++i) {
    if (arm_vel_.velocities[i].joint_uri == joint_name) {
      arm_vel_.velocities[i].timestamp = this->now();
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
    for (unsigned int j = 0; j < current_joint_states_->name.size(); ++j) {
      if (current_joint_states_->name[j] == arm_joint_limits_[i].joint_name) {
        std::cout << current_joint_states_->position[j];
        joint_name_list += current_joint_states_->name[j];

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
    for (unsigned int j = 0; j < current_joint_states_->name.size(); ++j) {
      if (current_joint_states_->name[j] == arm_joint_limits_[i].joint_name) {
        if (((current_joint_states_->position[j] <
              (arm_joint_limits_[i].min_position + soft_joint_limit_threshold_)) &&
             (arm_vel_.velocities[j].value < 0)) ||
            ((current_joint_states_->position[j] >
              (arm_joint_limits_[i].max_position - soft_joint_limit_threshold_)) &&
             (arm_vel_.velocities[j].value > 0))) {
          arm_vel_.velocities[i].value = 0.0;
        }
      }
    }
  }
}

void TeleOpJoypad::cbJoypad(const sensor_msgs::msg::Joy::SharedPtr command)
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
        RCLCPP_INFO(this->get_logger(), "open gripper");
        this->moveGripper(mir_interfaces::msg::GripperCommand::OPEN);
      } else {
        RCLCPP_INFO(this->get_logger(), "close gripper");
        this->moveGripper(mir_interfaces::msg::GripperCommand::CLOSE);
      }
    }

    // arm/base services
    if (!button_arm_motors_on_off_pressed_prev_ &&
        static_cast<bool>(command->buttons[button_index_arm_motors_on_off_])) {
      button_arm_motors_active_ = !button_arm_motors_active_;
      if (button_arm_motors_active_) {
        RCLCPP_INFO(this->get_logger(), "turn arm motors OFF");
        this->switchMotorsOnOff("arm", "OFF");
      } else {
        RCLCPP_INFO(this->get_logger(), "turn arm motors ON");
        this->switchMotorsOnOff("arm", "ON");
      }
    }

    if (!button_base_motors_on_off_pressed_prev_ &&
        static_cast<bool>(command->buttons[button_index_base_motors_on_off_])) {
      button_base_motors_active_ = !button_base_motors_active_;
      if (button_base_motors_active_) {
        RCLCPP_INFO(this->get_logger(), "turn base motors OFF");
        this->switchMotorsOnOff("base", "OFF");
      } else {
        RCLCPP_INFO(this->get_logger(), "turn base motors ON");
        this->switchMotorsOnOff("base", "ON");
      }
    }

    // arm cartesian control mode OR base cartesian control mode
    if (static_cast<bool>(command->buttons[button_index_arm_cart_])) {
      arm_cart_vel_.header.frame_id = arm_cartesian_reference_link_;
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
        pub_arm_joint_vel_->publish(arm_vel_);
      }
      if (button_arm_joint_3_4_pressed_prev_ &&
          !static_cast<bool>(command->buttons[button_index_arm_joint_3_4_])) {
        this->setSingleArmJointVel(0.0, arm_joint_names_[2]);
        this->setSingleArmJointVel(0.0, arm_joint_names_[3]);
        pub_arm_joint_vel_->publish(arm_vel_);
      }
      if (button_arm_joint_5_pressed_prev_ &&
          !static_cast<bool>(command->buttons[button_index_arm_joint_5_])) {
        this->setSingleArmJointVel(0.0, arm_joint_names_[4]);
        pub_arm_joint_vel_->publish(arm_vel_);
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
      pub_arm_cart_vel_->publish(arm_cart_zero_vel_);
    else if (static_cast<bool>(command->buttons[button_index_arm_cart_]))
      pub_arm_cart_vel_->publish(arm_cart_vel_);
    else if (!static_cast<bool>(command->buttons[button_index_arm_cart_]))
      pub_base_cart_vel_->publish(base_cart_vel_);

    if (is_joint_space_ctrl_active_ && is_one_arm_joint_button_pressed_) {
      this->checkArmJointLimits();
      pub_arm_joint_vel_->publish(arm_vel_);
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
        pub_arm_joint_vel_->publish(arm_vel_);
      }

      pub_base_cart_vel_->publish(base_cart_zero_vel_);
      pub_arm_cart_vel_->publish(arm_cart_zero_vel_);
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

rcl_interfaces::msg::SetParametersResult
  TeleOpJoypad::parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto &param : parameters)
    {
      RCLCPP_INFO(this->get_logger(), "Value of param %s changed to %s", param.get_name().c_str(), param.value_to_string().c_str());
      if (param.get_name() == "arm_cartesian_reference_link")
      {
        this->arm_cartesian_reference_link_ = param.get_value<std::string>();
      }
    }

    return result;
  }

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleOpJoypad>());
  rclcpp::shutdown();
  return 0;
}
