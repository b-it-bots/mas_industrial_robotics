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
    loop_rate_init_state_(ros::Rate(100.0)),
    serial_port_(NULL),
    serial_available_(false)
{
    ros::NodeHandle nh("~");

    nh.param("load_threshold", load_threshold_, 0.3);
    nh.param("error_threshold", error_threshold_, 0.1);
    nh.param("use_serial_threshold", use_serial_threshold_, 0.28);
    nh.param("serial_enabled", serial_enabled_, true);
    nh.param("serial_value_count", serial_value_count_, 2);
    nh.param("serial_threshold", serial_threshold_, 0.5);
    nh.param("serial_device", serial_device_, std::string("/dev/youbot/gripper_monitor"));
    nh.param("serial_baudrate", serial_baudrate_, 9600);
    nh.param("serial_timeout", serial_timeout_, 100);

    pub_event_ = nh.advertise<std_msgs::String>("event_out", 1);
    sub_event_ = nh.subscribe("event_in", 10, &DynamixelGripperGraspMonitorNode::eventCallback, this);
    sub_dynamixel_motor_states_ = nh.subscribe("dynamixel_motor_states", 10, &DynamixelGripperGraspMonitorNode::jointStatesCallback, this);
    if(serial_enabled_) {
        serial_buffer_ = (uint8_t *) malloc(sizeof(uint8_t) * serial_value_count_);
        serial_values_ = (double *) malloc(sizeof(double) * serial_value_count_);
    }
}

DynamixelGripperGraspMonitorNode::~DynamixelGripperGraspMonitorNode()
{
    if(serial_port_) {
        if(serial_port_->isOpen())
            serial_port_->close();
        delete serial_port_;
    }
    delete serial_buffer_;
    delete serial_values_;
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

    poll_serial();

    switch (current_state_)
    {
    case INIT:
        init_state();
        break;
    case IDLE:
        idle_state();
        break;
    case RUN:
        run_state();
        break;
    }
}

void DynamixelGripperGraspMonitorNode::poll_serial()
{
    if(!serial_enabled_)
        return;
    try {
        if(!serial_port_)
            serial_port_ = new serial::Serial(serial_device_, serial_baudrate_, serial::Timeout::simpleTimeout(serial_timeout_));
        if(!serial_port_->isOpen())
            serial_port_->open();
        if(serial_port_->available() < serial_value_count_)
            return;
        serial_port_->read(serial_buffer_, serial_value_count_);
        for(size_t i = 0; i < serial_value_count_; i++) {
            serial_values_[i] = (double) serial_buffer_[i] / 255.0;
        }
        serial_available_ = true;
    } catch (serial::IOException e) {
        serial_available_ = false;
        ROS_WARN("Serial communication failed");
        if(serial_port_) {
            delete serial_port_;
        }
        serial_port_ = NULL;
    }
}

void DynamixelGripperGraspMonitorNode::checkForNewEvent()
{
    if (!event_in_received_)
        return;

    ROS_INFO_STREAM("Received event: " << event_in_.data);

    if (event_in_.data == "e_trigger")
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
    if (joint_states_received_)
        current_state_ = RUN;

    joint_states_received_ = false;
}

void DynamixelGripperGraspMonitorNode::run_state()
{
    std_msgs::String event_out;

    if (isObjectGrasped())
        event_out.data = "e_object_grasped";
    else
        event_out.data = "e_object_not_grasped";

    pub_event_.publish(event_out);

    current_state_ = INIT;
}

bool DynamixelGripperGraspMonitorNode::isObjectGrasped()
{
    // ROS_DEBUG_STREAM("cur. load: " << joint_states_->load << " load thresh: " << load_threshold_);

    if (joint_states_->load > load_threshold_) {
        ROS_WARN("LOAD");
        return true;
    }
    if (abs(joint_states_->error) > error_threshold_) {
        ROS_WARN("ERROR");
        return true;
    }
    if(use_serial_threshold_ > joint_states_->current_pos) {
        ROS_WARN("USE_SERIAL");
        for(size_t i = 0; i < serial_value_count_; i++) {
            if(serial_values_[i] < serial_threshold_)
                return true;
        }
    }
    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grasp_monitor");
    ros::NodeHandle nh("~");

    DynamixelGripperGraspMonitorNode grasp_monitor;

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        ros::spinOnce();

        grasp_monitor.update();

        loop_rate.sleep();
    }


    return 0;
}
