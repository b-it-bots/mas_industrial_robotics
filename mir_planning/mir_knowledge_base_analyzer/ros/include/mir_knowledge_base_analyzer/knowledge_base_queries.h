#include <ros/ros.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <std_msgs/String.h>

class KnowledgeBaseQueries {
 public:
  KnowledgeBaseQueries(ros::NodeHandle &nh);
  virtual ~KnowledgeBaseQueries();

 private:
  void queryCallback(const std_msgs::String::ConstPtr &msg);
  void queryParamCallback(const std_msgs::String::ConstPtr &msg);

  bool publishObjectsAtLocation(const std::string &location);

  std::string getCurrentRobotLocation();
  bool publishRobotLocation();

  void stripObjectID(std::string &object);

  ros::NodeHandle nh_;
  ros::ServiceClient current_knowledge_client_;

  ros::Subscriber query_sub_;
  ros::Subscriber query_param_sub_;
  ros::Publisher event_out_;
  ros::Publisher objects_at_location_pub_;
  ros::Publisher robot_location_pub_;

  std::string query_param_;
};
