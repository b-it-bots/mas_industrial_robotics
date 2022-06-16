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


  /// Requests the current state of the node
  /**
   * In this function, we send a service request
   * asking for the current state of the node
   * lc_talker(default).
   * If it does return within the given time_out,
   * we return the current state of the node, if
   * not, we return an unknown state.
   * /param time_out Duration in seconds specifying
   * how long we wait for a response before returning
   * unknown state
   */

  bool get_state(std::chrono::seconds time_out);


  /// Invokes a transition
  /**
   * We send a Service request and indicate
   * that we want to invoke transition with
   * the id "transition".
   * By default, these transitions are
   * - configure
   * - activate
   * - cleanup
   * - shutdown
   * \param transition id specifying which
   * transition to invoke
   * \param time_out Duration in seconds specifying
   * how long we wait for a response before returning
   * unknown state
   */
  bool change_state(std::uint8_t transition, std::chrono::seconds time_out);
 
private:
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
  std::string lifecycle_node;
  std::string node_get_state_topic;
  std::string node_change_state_topic;
 

};




// For non-blocking keyboard inputs
int getch(void);



void callee_script(std::shared_ptr<LifecycleController> lifecycle_controller);



#endif  
