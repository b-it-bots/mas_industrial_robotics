/*
 * Orient Goal. Should not be used by any other function
 * other than basic_navigation (move_base)
 * Works fine ... Avoids osciallation at the begining and end of the goal navigation.
 * Will return null incase tolerance is less than or equal to 0.1
 *
 * Author : Praveen Ramanujam
 *          Ravi Kumar Venkat
 */



#include <mir_2dnav/orient_goal.hpp>
#include <pluginlib/class_list_macros.h>


namespace basic_navigation {
OrientGoal::OrientGoal():global_costmap_(NULL), local_costmap_(NULL),
		  tf_(NULL), initialized_(false), world_model_(NULL), goal_orientation_(NULL) {}

void OrientGoal::initialize(std::string name,tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap,
    geometry_msgs::Quaternion* goal_orientation ){
  if(!initialized_){
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;
    goal_orientation_ = goal_orientation;

    //get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name_);
    ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

    //we'll simulate every degree by default
    private_nh.param("sim_granularity", sim_granularity_, 0.017);
    private_nh.param("frequency", frequency_, 20.0);

    blp_nh.param("acc_lim_th", acc_lim_th_, 1.0);
    blp_nh.param("max_rotational_vel", max_rotational_vel_, 1.0);
    blp_nh.param("min_in_place_rotational_vel", min_rotational_vel_, 0.4);
    blp_nh.param("yaw_goal_tolerance", tolerance_, 0.10);

    local_costmap_->getCostmapCopy(costmap_);
    world_model_ = new base_local_planner::CostmapModel(costmap_);

    initialized_ = true;
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

OrientGoal::~OrientGoal(){
	ROS_INFO("Orient Goal destroyed Called\n");
	delete world_model_;

}

void OrientGoal::runBehavior(){
  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if(global_costmap_ == NULL || local_costmap_ == NULL){
    ROS_ERROR("The costmaps passed to the ClearCostmapRecovery object cannot be NULL. Doing nothing.");
    return;
  }

  ros::Rate r(frequency_);
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  tf::Stamped<tf::Pose> global_pose;
  global_costmap_->getRobotPose(global_pose);
  double yaw_req = angles::normalize_angle(tf::getYaw(*goal_orientation_));
  double yaw_now = angles::normalize_angle(tf::getYaw(global_pose.getRotation()));
  ROS_INFO("The required orientation is %f but the orientation of the robot is %f\n",yaw_req,yaw_now);
  double tol = fabs(yaw_req - yaw_now);
  ROS_INFO("Tolerance with in limits. Allowed tolerance is 0.1 and the difference is %f\n",tol);
  if (tol <= 0.1)
  {
	  ROS_INFO("No Goal Correction is required\n");
	  return;
  }
  ROS_INFO("Goal Correction is required\n");

  double current_angle = -1.0 * M_PI;

  while(n.ok()){
    global_costmap_->getRobotPose(global_pose);

    current_angle = angles::normalize_angle(tf::getYaw(global_pose.getRotation()));
    double dist_left = angles::shortest_angular_distance(current_angle,yaw_req);
    ROS_INFO("Current angle = %f Required Angle = %f", current_angle,yaw_req);
    ROS_INFO("Shortest angle = %f",dist_left);

    //update the costmap copy that the world model holds
   // local_costmap_->getCostmapCopy(costmap_);
    global_costmap_->getCostmapCopy(costmap_);

    //check if that velocity is legal by forward simulating
    double sim_angle = 0.0;
    while(sim_angle < dist_left){
      std::vector<geometry_msgs::Point> oriented_footprint;
      double theta = tf::getYaw(global_pose.getRotation())+sim_angle;
      geometry_msgs::Point position;
      position.x = global_pose.getOrigin().x();
      position.y = global_pose.getOrigin().y();

      global_costmap_->getOrientedFootprint(position.x, position.y, theta, oriented_footprint);
     double footprint_cost = 1.0;
      //double footprint_cost = world_model_->footprintCost(position, oriented_footprint, local_costmap_->getInscribedRadius(), local_costmap_->getCircumscribedRadius());
      if(footprint_cost < 0.0){
        ROS_WARN("Rotation towards goal cannot take place because there is a potential collision. Cost: %.2f", footprint_cost);
        return;
      }

      sim_angle += sim_granularity_;
    }

    if (fabs(dist_left) < 0.1)
    	return;
    

    double vel = sqrt(2 * acc_lim_th_ * fabs(dist_left));

    (dist_left <=0.0)?(vel= -vel):(vel = vel);
    vel = std::min(std::max(vel, -0.5),0.5);
    ROS_INFO("The required orientation is %f but the orientation of the robot is %f\n",yaw_req,current_angle);
    ROS_INFO("Velocity:: %f",vel);

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = vel;

    vel_pub.publish(cmd_vel);

    r.sleep();
  }
}
};

