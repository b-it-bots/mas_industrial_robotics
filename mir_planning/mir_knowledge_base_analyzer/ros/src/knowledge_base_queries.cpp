#include <mir_knowledge_base_analyzer/knowledge_base_queries.h>
#include <mir_planning_msgs/ObjectsAtLocation.h>

#include <ctype.h>

KnowledgeBaseQueries::KnowledgeBaseQueries(ros::NodeHandle &nh) : nh_(nh)

{
    current_knowledge_client_ = nh_.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/rosplan_knowledge_base/state/propositions");

    query_sub_ = nh_.subscribe("query", 1, &KnowledgeBaseQueries::queryCallback, this);
    query_param_sub_ = nh_.subscribe("query_param", 1, &KnowledgeBaseQueries::queryParamCallback, this);

    event_out_ = nh_.advertise<std_msgs::String>("event_out", 1);
    objects_at_location_pub_ = nh_.advertise<mir_planning_msgs::ObjectsAtLocation>("objects_at_location", 1);
    robot_location_pub_ = nh_.advertise<std_msgs::String>("robot_location", 1);
}

KnowledgeBaseQueries::~KnowledgeBaseQueries()
{

}

void KnowledgeBaseQueries::queryCallback(const std_msgs::String::ConstPtr &msg)
{
    std_msgs::String eout;
    if (msg->data == "get_objects_at_location")
    {
        if (!query_param_.empty())
        {
            bool result = publishObjectsAtLocation(query_param_);
            if (result)
            {
                eout.data = "e_success";
            }
            else
            {
                eout.data = "e_failure";
            }
        }
        else
        {
            eout.data = "e_failure";
        }
    }
    else if (msg->data == "get_objects_at_current_location")
    {
        std::string location = getCurrentRobotLocation();
        if (!location.empty())
        {
            bool result = publishObjectsAtLocation(location);
            if (result)
            {
                eout.data = "e_success";
            }
            else
            {
                eout.data = "e_failure";
            }
        }
        else
        {
            eout.data = "e_failure";
        }
    }
    else if (msg->data == "get_robot_location")
    {
        bool result = publishRobotLocation();
        if (result)
        {
            eout.data = "e_success";
        }
        else
        {
            eout.data = "e_failure";
        }
    }
    else
    {
        eout.data = "e_failure";
    }
    query_param_ = "";
    event_out_.publish(eout);
}

void KnowledgeBaseQueries::queryParamCallback(const std_msgs::String::ConstPtr &msg)
{
    query_param_ = msg->data;
}

bool KnowledgeBaseQueries::publishObjectsAtLocation(const std::string &location)
{
    rosplan_knowledge_msgs::GetAttributeService srv;
    srv.request.predicate_name = "on";
    std::vector<std::string> objects;
    if (current_knowledge_client_.call(srv))
    {
        mir_planning_msgs::ObjectsAtLocation objects_msg;
        objects_msg.location = location;
        for (int i = 0; i < srv.response.attributes.size(); i++)
        {
            std::vector<diagnostic_msgs::KeyValue> keyvalues = srv.response.attributes[i].values;
            std::string current_object;
            bool use_current_object = false;
            for (int j = 0; j < keyvalues.size(); j++)
            {
                if (keyvalues[j].key == "l" && keyvalues[j].value == location)
                {
                    use_current_object = true;
                }
                if (keyvalues[j].key == "o")
                {
                    current_object = keyvalues[j].value;
                }
            }
            if (use_current_object && !current_object.empty())
            {
                stripObjectID(current_object);
                objects_msg.objects.push_back(current_object);
            }
        }
        objects_at_location_pub_.publish(objects_msg);
        return true;
    }
    else
    {
        return false;
    }
}

void KnowledgeBaseQueries::stripObjectID(std::string &object)
{
    if (isdigit(object.at(object.length() - 1)) &&
        isdigit(object.at(object.length() - 2)) &&
        object.at(object.length() - 3) == '-')
    {
        object = object.substr(0, object.length() - 3);
    }
}

std::string KnowledgeBaseQueries::getCurrentRobotLocation()
{
    rosplan_knowledge_msgs::GetAttributeService srv;
    srv.request.predicate_name = "at";
    if (current_knowledge_client_.call(srv))
    {
        if (srv.response.attributes.size() != 1)
        {
            return "";
        }

        std::vector<diagnostic_msgs::KeyValue> keyvalues = srv.response.attributes[0].values;
        std::string location;
        for (int j = 0; j < keyvalues.size(); j++)
        {
            if (keyvalues[j].key == "l")
            {
                location = keyvalues[j].value;
                break;
            }
        }
        return location;
    }
    else
    {
        return "";
    }
}

bool KnowledgeBaseQueries::publishRobotLocation()
{
    std::string location_str = getCurrentRobotLocation();
    if (!location_str.empty())
    {
        std_msgs::String location;
        location.data = location_str;
        robot_location_pub_.publish(location);
        return true;
    }
    else
    {
        return false;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "knowledge_base_queries");
    ros::NodeHandle nh("~");
    KnowledgeBaseQueries kbq(nh);
    int loop_rate = 5;
    nh.param<int>("loop_rate", loop_rate, 5);
    ros::Rate rate(loop_rate);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
