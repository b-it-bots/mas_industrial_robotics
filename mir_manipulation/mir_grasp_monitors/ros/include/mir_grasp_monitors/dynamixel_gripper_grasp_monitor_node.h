/*
 * dynamixel_gripper_grasp_monitor_node.h
 *
 *  Created on: Jul 11, 2015
 *      Author: Frederik Hegger
 */

#ifndef DYNAMIXEL_GRIPPER_GRASP_MONITOR_NODE_H_
#define DYNAMIXEL_GRIPPER_GRASP_MONITOR_NODE_H_

#include <dynamixel_msgs/JointState.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <serial/serial.h>

class DynamixelGripperGraspMonitorNode
{
public:
    DynamixelGripperGraspMonitorNode();
    ~DynamixelGripperGraspMonitorNode();

    void update();

private:
    enum States {INIT, IDLE, RUN};

    void jointStatesCallback(const dynamixel_msgs::JointState::Ptr &msg);
    void eventCallback(const std_msgs::String::ConstPtr &msg);

    void checkForNewEvent();
    void init_state();
    void idle_state();
    void run_state();

    void poll_serial();

    bool isObjectGrasped();

    ros::Publisher pub_event_;
    ros::Subscriber sub_event_;
    ros::Subscriber sub_dynamixel_motor_states_;

    dynamixel_msgs::JointState::Ptr joint_states_;
    bool joint_states_received_;

    std_msgs::String event_in_;
    bool event_in_received_;

    States current_state_;

    ros::Rate loop_rate_init_state_;

    double load_threshold_;
    double error_threshold_;
    double use_serial_threshold_;

    bool serial_enabled_;
    std::string serial_device_;
    int serial_baudrate_;
    int serial_timeout_;;
    int serial_value_count_;
    double serial_threshold_;
    serial::Serial *serial_port_;
    bool serial_available_;
    uint8_t *serial_buffer_;
    double *serial_values_;
};

#endif /* DYNAMIXEL_GRIPPER_GRASP_MONITOR_NODE_H_ */

