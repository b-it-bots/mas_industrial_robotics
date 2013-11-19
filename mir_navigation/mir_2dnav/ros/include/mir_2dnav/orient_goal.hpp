/*
 * Orient Goal. Should not be used by any other function
 * other than basic_navigation (move_base)
 * Works fine ... Avoids osciallation at the begining and end of the goal navigation.
 * Will return null incase tolerance is less than or equal to 0.1
 *
 * Author : Praveen Ramanujam
 *          Ravi Kumar Venkat
 */


#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <base_local_planner/costmap_model.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>

namespace basic_navigation{

  class OrientGoal {
    public:

      OrientGoal();
      void initialize(std::string name, tf::TransformListener* tf,costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap,
    		  geometry_msgs::Quaternion *goal_orientation);
      void runBehavior();
      ~OrientGoal();

    private:
      costmap_2d::Costmap2DROS* global_costmap_, *local_costmap_;
      costmap_2d::Costmap2D costmap_;
      std::string name_;
      tf::TransformListener* tf_;
      bool initialized_;
      double sim_granularity_, min_rotational_vel_, max_rotational_vel_, acc_lim_th_, tolerance_, frequency_;
      base_local_planner::CostmapModel* world_model_;
      geometry_msgs::Quaternion* goal_orientation_;
  };
};
