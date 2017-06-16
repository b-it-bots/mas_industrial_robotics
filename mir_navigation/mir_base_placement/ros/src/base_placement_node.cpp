#include <iostream>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <mcr_perception_msgs/BaseScanLinearRegression.h>
#include <mir_navigation_msgs/OrientToBaseAction.h>

//#include "mir_base_placement/laser_scan_linear_regression.h"

using namespace mcr_perception_msgs;
using namespace mir_navigation_msgs;

class OrientToLaserReadingAction
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<OrientToBaseAction> as_;
    std::string action_name_;

    std::string service_name;
    std::string cmd_vel_topic;

    float target_distance;
    float max_linear_velocity;
    float max_angular_velocity;

    double error_angle_int;
    double error_lin_int;

    double error_angle_d;
    double error_lin_d;

    double last_error_lin;
    double last_error_angular;

    double error_tolerance;

    ros::Publisher cmd_pub;
    ros::ServiceClient client;



public:

    OrientToLaserReadingAction(ros::NodeHandle nh, std::string name, std::string cmd_vel_topic, std::string linreg_service_name)
        : as_(nh, name, boost::bind(&OrientToLaserReadingAction::executeActionCB, this, _1), false)
    {
        this->action_name_ = name;
        this->service_name = linreg_service_name;
        this->cmd_vel_topic = cmd_vel_topic;

        nh_ = nh;

        target_distance = 0.05;

        ros::param::param<float>("~max_linear_velocity", max_linear_velocity, 0.075);
        ros::param::param<float>("~max_angular_velocity", max_angular_velocity, 0.1);

        ROS_DEBUG("Register publisher");

        cmd_pub = nh_.advertise < geometry_msgs::Twist > (cmd_vel_topic, 1000);

        ROS_DEBUG("Create service client");

        client = nh_.serviceClient < BaseScanLinearRegression > (service_name);

        as_.start();

    }


    geometry_msgs::Twist calculateVelocityCommand(double center, double a, double b, bool &oriented, int &iterator)
    {
        geometry_msgs::Twist cmd;

        double lin_p;
        ros::param::param<double>("~lin_p", lin_p, 0.8);
        double lin_i;
        ros::param::param<double>("~lin_i", lin_i, 0.0);
        double lin_d;
        ros::param::param<double>("~lin_d", lin_d, 0.0);

        double ang_p;
        ros::param::param<double>("~ang_p", ang_p, 0.5);
        double ang_i;
        ros::param::param<double>("~ang_i", ang_i, 0.0);
        double ang_d;
        ros::param::param<double>("~ang_d", ang_d, 0.0);

        double error_angle = -b;
        double error_lin = (a - target_distance);

        //angular velocity calculation
        error_angle_int += error_angle;
        error_angle_int = std::min(std::max(error_angle_int, -0.1), 0.1);
        error_angle_d = error_angle - last_error_angular;
        last_error_angular = error_angle;

        cmd.angular.z = (-1)*(error_angle * ang_p + error_angle_int * ang_i + error_angle_d * ang_d);

        //linear velocity calculation
        error_lin_int += error_lin;
        error_lin_int = std::min(std::max(error_lin_int, -0.1), 0.1);
        error_lin_d = error_lin - last_error_lin;
        last_error_lin = error_lin;

        double calculated_velocity = error_lin * lin_p + error_lin_int * lin_i + error_lin_d * lin_d;
        std::string laser_axis;
        ros::param::param<std::string>("~laser_axis", laser_axis, "+x");
        if (laser_axis == "+x")
        {
            cmd.linear.x = calculated_velocity;
        }
        else if (laser_axis == "-x")
        {
            cmd.linear.x = -calculated_velocity;
        }
        else if (laser_axis == "+y")
        {
            cmd.linear.y = calculated_velocity;
        }
        else if (laser_axis == "-y")
        {
            cmd.linear.y = -calculated_velocity;
        }

        /*
        std::cout << "=======================" << std::endl;

        std::cout << "error_angle: " << error_angle << std::endl;
        std::cout << "error_lin: " << error_lin << std::endl;

        std::cout << "b: " << b << std::endl;
        std::cout << "center: " << fabs(center) << std::endl;
        std::cout << "a: " << a << std::endl;

        std::cout << "a_p: " << error_angle * ang_p << std::endl;
        std::cout << "a_i: " << error_angle_int * ang_i << std::endl;
        std::cout << "a_d: " << error_angle_d * ang_d << std::endl;
        */

        if (cmd.linear.x > max_linear_velocity)
            cmd.linear.x = max_linear_velocity;
        else if (cmd.linear.x < -max_linear_velocity)
            cmd.linear.x = -max_linear_velocity;

        if (cmd.linear.y > max_linear_velocity)
            cmd.linear.y = max_linear_velocity;
        else if (cmd.linear.y < -max_linear_velocity)
            cmd.linear.y = -max_linear_velocity;

        if (cmd.angular.z > max_angular_velocity)
            cmd.angular.z = max_angular_velocity;
        else if (cmd.angular.z < -max_angular_velocity)
            cmd.angular.z = -max_angular_velocity;

        return cmd;
    }

    void executeActionCB(const OrientToBaseGoalConstPtr& goal)
    {

        BaseScanLinearRegression srv;

        srv.request.filter_minAngle = -M_PI_4;
        srv.request.filter_maxAngle = M_PI_4;
        srv.request.filter_minDistance = 0.02;
        srv.request.filter_maxDistance = 0.6;

        target_distance = goal->distance;

        ros::Duration max_time(30.0);
        ros::Time stamp = ros::Time::now();
        OrientToBaseResult result;
        bool oriented = false;
        int iterator = 0;

        error_angle_int = 0.0;
        error_lin_int = 0.0;
        error_angle_d = 0.0;
        error_lin_d = 0.0;
        last_error_angular = 0.0;
        last_error_lin = 0.0;

        double translation_error_tolerance = 0.0;
        double angular_error_tolerance = 0.0;
        ros::param::param<double>("~translation_error_tolerance", translation_error_tolerance, 0.04);
        ros::param::param<double>("~angular_error_tolerance", angular_error_tolerance, 0.04);

        while (true)
        {
            ROS_DEBUG("Call service Client");

            if (client.call(srv))
            {
                ROS_DEBUG("Called service LaserScanLinearRegressionService");
                //std::cout << "result: " << srv.response.center << ", " << srv.response.a << ", " << srv.response.b << std::endl;

                geometry_msgs::Twist cmd = calculateVelocityCommand(srv.response.center, srv.response.a, srv.response.b, oriented, iterator);
                cmd_pub.publish(cmd);

                //std::cout << "cmd x:" << cmd.linear.x << ", y: "  << cmd.linear.y << ", z: " << cmd.angular.z << std::endl;

                double translation_error = fabs(srv.response.a - target_distance);
                double angular_error = fabs(srv.response.b)
                // std::cout << "current error: " << error << std::endl;

                if ((translation_error < translation_error_tolerance) and (angular_error<angular_error_tolerance))
                {

                    ROS_DEBUG("Point reached");
                    result.succeed = true;
                    as_.setSucceeded(result);
                    geometry_msgs::Twist zero_vel;
                    cmd_pub.publish(zero_vel);
                    break;

                }

                if (stamp + max_time < ros::Time::now())
                {
                    result.succeed = false;
                    as_.setAborted(result);
                    geometry_msgs::Twist zero_vel;
                    cmd_pub.publish(zero_vel);
                    break;
                }

            }
            else
            {
                ROS_ERROR("Failed to call service LaserScanLinearRegressionService");

                if (stamp + max_time < ros::Time::now())
                {
                    result.succeed = false;
                    as_.setAborted(result);
                    break;
                }
            }
        }

    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "base_placement");
    ros::NodeHandle n("~");

    std::string cmd_vel_name = "/cmd_vel";

    n.getParam("cmd_vel_topic", cmd_vel_name);
    ROS_DEBUG("Publishing on cmd_vel_topic: %s", cmd_vel_name.c_str());

    std::string service_name = "linear_regression_service-NOT-SET";
    n.getParam("linear_regression_service", service_name);
    ROS_DEBUG("Using linear_regression_service: %s", service_name.c_str());


    OrientToLaserReadingAction orientAction(n, "adjust_to_workspace", cmd_vel_name, service_name);

    ROS_DEBUG("Action Service is ready");

    ros::Rate loop_rate(15);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
