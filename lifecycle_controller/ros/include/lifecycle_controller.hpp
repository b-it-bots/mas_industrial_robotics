/*
 * Copyright 2022 Bonn-Rhein-Sieg University
 *
 * Author: Hamsa Datta Perur
 *
 */

#ifndef LIFECYCLE_CONTROLLER_HPP
#define LIFECYCLE_CONTROLLER_HPP

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <iostream>
#include <termios.h>
#include <unistd.h>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rcutils/logging_macros.h"


extern char key;
extern const char* display;

// Adapted from  ROS2 lifecycle demos
// 
// Every lifecycle node has various services
// attached to it. By convention, we use the format of
// <node name>/<service name>.
// In this file, we use get_state and change_state
// and thus the two service topics are:
// <node name>/get_state
// <node name>/change_state 


template<typename FutureT, typename WaitTimeT>
std::future_status 
wait_for_result( FutureT & future,  WaitTimeT time_to_wait);


class LifecycleController : public rclcpp::Node
{
public:

  explicit LifecycleController(const std::string & node_name);  

  void init();

  bool get_state(std::chrono::seconds time_out);
  
  bool change_state(std::uint8_t transition, std::chrono::seconds time_out);
 
private:

  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
  std::string lifecycle_node;
  std::string get_state_topic; 
	std::string change_state_topic;
  std::string node_get_state_topic;
  std::string node_change_state_topic; 

};

int getch(void);

void callee_script(std::shared_ptr<LifecycleController> lifecycle_controller);

#endif  
