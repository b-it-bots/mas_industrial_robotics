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

class DynamixelGripperGraspMonitorNode
{
public:
    DynamixelGripperGraspMonitorNode();
    ~DynamixelGripperGraspMonitorNode();

    void update();

private:
    enum States {INIT, IDLE, RUN};

    std::map<std::string, double> object_threshold_map;

    void jointStatesCallback(const dynamixel_msgs::JointState::Ptr &msg);
    void eventCallback(const std_msgs::String::ConstPtr &msg);
    void objectNameCallback(const std_msgs::String::ConstPtr &msg);

    void checkForNewEvent();
    void init_state();
    void idle_state();
    void run_state();
    bool isObjectGrasped();
    void addThreshold(std::string str, float f);
    ros::Publisher pub_event_;
    ros::Subscriber sub_event_;
    ros::Subscriber sub_dynamixel_motor_states_;
    ros::Subscriber sub_object_name_;

    dynamixel_msgs::JointState::Ptr joint_states_;
    bool joint_states_received_;

    std_msgs::String event_in_;
    bool event_in_received_;
    std::string object_name_;

    States current_state_;

    ros::Rate loop_rate_init_state_;

    double load_threshold_;
    double position_error_threshold_;

};

#endif /* DYNAMIXEL_GRIPPER_GRASP_MONITOR_NODE_H_ */
