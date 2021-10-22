#include <ros/ros.h>
#include <mir_cavity_detector/cavity_finder_ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cavity_finder");
    ros::NodeHandle nh("~");

    int frame_rate = 30;    // in Hz
    CavityFinderROS cavity_finder_ros_;

    nh.param<int>("frame_rate", frame_rate, 30);
    ROS_INFO("[cavity_finder] node started");

    ros::Rate loop_rate(frame_rate);

    while (ros::ok())
    {
        cavity_finder_ros_.update();

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
