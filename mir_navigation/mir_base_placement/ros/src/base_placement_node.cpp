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
	float max_velocity;
	float min_velocity;
	float min_angular_velocity;
	bool stop_oscillation;
	int oscillation;
	float prev_angvel;
	bool orient_command_counts;

	ros::Publisher cmd_pub;
	ros::ServiceClient client;

 public:

	OrientToLaserReadingAction(ros::NodeHandle nh, std::string name, std::string cmd_vel_topic, std::string linreg_service_name)
			: as_(nh, name, boost::bind(&OrientToLaserReadingAction::executeActionCB, this, _1), false)
	{
		//as_.registerGoalCallback();
		this->action_name_ = name;
		this->service_name = linreg_service_name;
		this->cmd_vel_topic = cmd_vel_topic;

		nh_ = nh;

		target_distance = 0.05;
		max_velocity = 0.15;
		min_velocity = 0.01;
		min_angular_velocity = 0.1;

		stop_oscillation = false;
		oscillation = 0;
		orient_command_counts = 0;

		ROS_DEBUG("Register publisher");

		cmd_pub = nh_.advertise < geometry_msgs::Twist > (cmd_vel_topic, 1000);

		ROS_DEBUG("Create service clinet");

		client = nh_.serviceClient < BaseScanLinearRegression > (service_name);

		as_.start();

	}

	geometry_msgs::Twist calculateVelocityCommand(double center, double a, double b, bool &oriented, int &iterator)
	{
		geometry_msgs::Twist cmd;

		std::cout << "b: " << fabs(b) << std::endl;
		std::cout << "center:: " << fabs(center) << std::endl;
		std::cout << "a" << a << std::endl;
		std::cout << "oscillation" << oscillation << std::endl;
		stop_oscillation = (oscillation > 4) ? true : false;
		//0.0003
		if ((fabs(b) > 0.1) && (fabs(center) > 0.0005) && !stop_oscillation && !oriented)
		{
			//velcoity update
			cmd.angular.z = -b / 2;

		}
		/*
		 else if (fabs(center) > 0.005)
		 {
		 cmd.linear.y = center / 3;
		 //cmd_pub.publish(cmd);
		 //std::cout << "cmd.linear.y:  " << cmd.linear.y << std::endl;
		 }*/

		else if (a > target_distance)
		{
			oriented = true;
			/*
			 if (fabs(a - target_distance) < 0.001)
			 {
			 oriented = false;

			 }
			 else
			 {
			 oriented = true;

			 }*/

			//cmd.linear.x = a / 3;
			cmd.linear.x = (a - target_distance) / 2;
			cmd.angular.z = 0;
			//std::cout << "cmd.linear.x:  " << cmd.linear.x << std::endl;
			//cmd_pub.publish(cmd);

		}
		else if (a < target_distance)
		{

			cmd.linear.x = -(target_distance - a) / 2;
			cmd.angular.z = 0;

			/*
			 if (fabs(a - target_distance) < 0.01)
			 {
			 oriented = false;

			 }
			 else
			 {
			 oriented = true;

			 }*/
			//   std::cout << "cmd.linear.x:  " << cmd.linear.x << std::endl;
			if (iterator < 10)
			{
				//oscillation = 0;
				//stop_oscillation = false;
				iterator++;
			}
			else
			{
				oriented = true;
			}
		}

		if (cmd.linear.x > max_velocity)
			cmd.linear.x = max_velocity;
		else if (cmd.linear.x < -max_velocity)
			cmd.linear.x = -max_velocity;

		if (cmd.linear.y > max_velocity)
			cmd.linear.y = max_velocity;
		else if (cmd.linear.y < -max_velocity)
			cmd.linear.y = -max_velocity;

		if (cmd.angular.z > max_velocity)
			cmd.angular.z = max_velocity;
		else if (cmd.angular.z < -max_velocity)
			cmd.angular.z = -max_velocity;

		if (cmd.linear.x > 0 && cmd.linear.x < min_velocity)
			cmd.linear.x = min_velocity;

		if (cmd.angular.z > 0 && cmd.angular.z < min_angular_velocity)
			cmd.angular.z = min_angular_velocity;

		if (cmd.angular.z < 0 && cmd.angular.z < -min_angular_velocity)
			cmd.angular.z = -min_angular_velocity;

		bool sign_prev_angvel = (prev_angvel > 0);
		bool sign_curr_angvel = (cmd.angular.z > 0);

		std::cout << "prev_angvel" << prev_angvel << std::endl;
		std::cout << "curr_angvel" << cmd.angular.z << std::endl;
		if ((sign_curr_angvel != sign_prev_angvel) && (!((prev_angvel == 0) || (cmd.angular.z == 0))) && ((fabs(prev_angvel) + fabs(cmd.angular.z)) < 0.26))
		{
			oscillation += 1;
		}
		prev_angvel = cmd.angular.z;

		//	std::cout << "cmd.linear.x:  " << cmd.linear.x << std::endl;
		//	std::cout << "cmd.angular.z:  " << cmd.angular.z << std::endl;
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

		ros::Duration max_time(20.0);
		ros::Time stamp = ros::Time::now();
		OrientToBaseResult result;
		bool oriented = false;
		int iterator = 0;

		while (true)
		{
			ROS_DEBUG("Call service Client");

			if (client.call(srv))
			{
				ROS_DEBUG("Called service LaserScanLinearRegressionService");
				//std::cout << "result: " << srv.response.center << ", " << srv.response.a << ", " << srv.response.b << std::endl;

				geometry_msgs::Twist cmd = calculateVelocityCommand(srv.response.center, srv.response.a, srv.response.b, oriented, iterator);
				cmd_pub.publish(cmd);

				//	std::cout << "cmd x:" << cmd.linear.x << ", y: "  << cmd.linear.y << ", z: " << cmd.angular.z << std::endl;

				if ((fabs(cmd.angular.z) + fabs(cmd.linear.x)) < 0.001)
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
	std::string service_name = "scan_front_linearregression";

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

