#include <mcr_cavity_detector/cavity_finder_ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <mas_perception_msgs/PointCloud2List.h>
#include <mas_perception_msgs/Cavity.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>

#include <Eigen/Eigenvalues>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <mas_perception_msgs/ImageList.h>

CavityFinderROS::CavityFinderROS() : nh_("~"), it_(nh_),pointcloud_msg_received_(false), publish_debug_image_(true),
                                     img_msg_received_(false), cavity_msg_received_count_(0), img_msg_received_count_(0),pointcloud_msg_received_count_(0),
                                     offset_in_z_(0.055)
{
    dynamic_reconfigure_server_.setCallback(boost::bind(&CavityFinderROS::dynamicReconfigCallback, this, _1, _2));
    pub_cavity_pointclouds_ = nh_.advertise<mas_perception_msgs::PointCloud2List>("output/pointclouds", 1);
    pub_cavity_pointclouds_combined_ = nh_.advertise<sensor_msgs::PointCloud2>("output/pointclouds_combined", 1);
    pub_rgb_debug_image_ = it_.advertise("output/rgb_debug_image", 1);
    pub_depth_debug_image_ = it_.advertise("output/depth_debug_image", 1);
    pub_pose_ = nh_.advertise<geometry_msgs::PoseArray>("output/pose", 1);
    pub_cavity_ = nh_.advertise<mas_perception_msgs::Cavity>("output/cavity", 10);
    pub_cropped_cavities_ = nh_.advertise<mas_perception_msgs::ImageList>("output/cropped_cavities", 10);
    
    sub_cavities_name = nh_.subscribe("output/cavities_list", 10, &CavityFinderROS::cavitiesCallback, this);
    
    sub_event_in_ = nh_.subscribe("input/event_in", 1, &CavityFinderROS::eventInCallback, this);
    pub_event_out_ = nh_.advertise<std_msgs::String>("output/event_out", 1 );

    nh_.param<std::string>("target_frame", target_frame_, "base_link");
    nh_.param<std::string>("source_frame", source_frame_, "arm_cam4d_rgb_optical_frame");
}

CavityFinderROS::~CavityFinderROS()
{
}

void CavityFinderROS::update()
{
    //if (pointcloud_msg_received_ && img_msg_received_)
    if (pointcloud_msg_received_count_ > 1 && img_msg_received_count_ > 1 )
    {
        pointcloud_msg_received_ = false;
        img_msg_received_ = false;
        pointcloud_msg_received_count_ = 0;
        img_msg_received_count_ = 0;
        sub_pointcloud_.shutdown();
        sub_image_.shutdown();
        tmp_sub_image_.shutdown();
        findCavities();
        /* sub_cavities_name.shutdown(); */
    }
}

void CavityFinderROS::pointcloudCallback(const sensor_msgs::PointCloud2::Ptr &msg)
{
    if (! pointcloud_msg_received_ )
    {
        ROS_INFO("[cavity_finder_ros] Received pointcloud message");
        pointcloud_msg_ = msg;
        //pointcloud_msg_received_ = true;
        pointcloud_msg_received_count_ += 1;
        //sub_pointcloud_.shutdown();
    }
}

void CavityFinderROS::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    if (! img_msg_received_)
    {
        ROS_INFO("[cavity_finder_ros] Received rgb image ");
        rgb_image_ = msg;
        //img_msg_received_ = true; 
        img_msg_received_count_ += 1;
        //sub_image_.shutdown();
    }
}


void CavityFinderROS::cavitiesCallback(const mas_perception_msgs::ObjectList &msg)
{

    ROS_INFO("Received cavities name callback ");
    if (cavity_msg_received_count_ < 1)
    {
        cavity_list_ = msg;
        cavity_msg_received_count_ += 1;
    }

}

void CavityFinderROS::eventInCallback(const std_msgs::String &msg)
{
    if (msg.data == "e_trigger")
    {
        cavity_list_.objects.clear();
        cavity_msg_received_count_ = 0;
        sub_pointcloud_ = nh_.subscribe("input/pointcloud", 1, &CavityFinderROS::pointcloudCallback, this);
        /* sub_cavities_name = nh_.subscribe("output/cavities_list", 10, &CavityFinderROS::cavitiesCallback, this); */
        sub_image_ = it_.subscribe("image", 1, &CavityFinderROS::imageCallback, this);
        ROS_INFO("Subscribed to pointcloud and RGB image");
    }
}

void CavityFinderROS::dynamicReconfigCallback(mcr_cavity_detector::CavityFinderConfig &config, uint32_t level)
{
    cavity_finder_.setCannyThreshold(config.canny_threshold);
    cavity_finder_.setCannyMultiplier(config.canny_multiplier);
    cavity_finder_.setBinaryThreshold(config.binary_threshold);
    cavity_finder_.setEpsilonApproxPoly(config.epsilon_approx_poly);
    cavity_finder_.setEpsilonFinerPoly(config.epsilon_finer_poly);
    cavity_finder_.setMinArea(config.min_area_depth_image);
    cavity_finder_.setMaxArea(config.max_area_depth_image);
    offset_in_z_ = config.offset_in_z;
}

void CavityFinderROS::findCavities()
{
    //Output message to publish on event_out
    std_msgs::String output_msg;

    pcl::PCLPointCloud2::Ptr pcl_input_cloud(new pcl::PCLPointCloud2);

    // Convert to PCL data type
    pcl_conversions::toPCL(*pointcloud_msg_, *pcl_input_cloud);

    pcl::PCLImage pcl_image;
    pcl::toPCLPointCloud2(*pcl_input_cloud, pcl_image);

    sensor_msgs::ImagePtr image_msg = boost::make_shared<sensor_msgs::Image>();
    pcl_conversions::moveFromPCL(pcl_image, *image_msg);
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

    // Detect Bounding rect and the cavities
    cv::Mat depth_debug_image;
    std::vector<cv::Point2f> centroids;

    std::vector<std::vector<cv::Point> > cavities = cavity_finder_.find2DCavities(cv_image->image,
                                                                                 depth_debug_image,
                                                                                 centroids);
    ROS_INFO("[cavity_finder_ros] Found %i cavities", (int)cavities.size());
    if (cavities.size() == 0)
    {
        ROS_ERROR("[cavity_finder_ros] No  %i cavities found", (int)cavities.size());
        output_msg.data = std::string("e_failed");
        pub_event_out_.publish(output_msg);
        return;
    }

    std::vector<pcl::PCLPointCloud2::Ptr> pcl_cavities = cavity_finder_.get3DCavities(cavities, pcl_input_cloud);

    ROS_INFO("[cavity_finder_ros] Found %i pcl cavities", (int)pcl_cavities.size());

    filterCavities(cavities, pcl_cavities, centroids);

    ROS_INFO_STREAM("cavities " << (int)cavities.size() << "; pcl cavities" << (int)pcl_cavities.size());

    mas_perception_msgs::PointCloud2List ros_cavities;

    for (size_t i = 0; i < pcl_cavities.size(); i++)
    {
        sensor_msgs::PointCloud2 ros_pointcloud;
        ros_pointcloud.header = pointcloud_msg_->header;
        // Convert to ROS data type
        pcl_conversions::fromPCL(*(pcl_cavities[i]), ros_pointcloud);
        ros_cavities.pointclouds.push_back(ros_pointcloud);
    }

    // Publish the cavities
    pub_cavity_pointclouds_.publish(ros_cavities);

    // Publish pose of cavity
    geometry_msgs::PoseArray pose_array = estimatePose(pcl_cavities);

    //Recognize the 2d Cavities 
    cv_bridge::CvImagePtr cv_rgb_image = cv_bridge::toCvCopy(rgb_image_, sensor_msgs::image_encodings::BGR8);
    cv::Mat rgb_debug_image;
    mas_perception_msgs::ImageList image_list;
    std::vector<cv::Mat> cropped_cavities;
    std::vector<std::string> cavities_name_cv = cavity_finder_.recognize2DCavities(cv_rgb_image->image,
                                                                                 rgb_debug_image,
                                                                                 cropped_cavities,
                                                                                 centroids);


    // store cropped cavities in image list and publish it
    image_list.images.resize(centroids.size());

    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg; // >> message to be sent

    std_msgs::Header header; // empty header
    header.seq = 1; // user defined counter
    header.stamp = ros::Time::now(); // time

    for( int i = 0; i< centroids.size(); i++ )
    {
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, cropped_cavities[i]);
        img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
        image_list.images[i] = img_msg;
    }

    // ROS_INFO("publishing images ");
    pub_cropped_cavities_.publish(image_list);

    // get name of cavities from topic
    std::vector<std::string> cavities_name;
    // ROS_INFO_STREAM("cavity_msg value " << cavity_msg_received_count_);
    int frame_rate = 30;
    ros::Rate loop_rate(frame_rate);

    while (cavity_msg_received_count_ == 0){

        loop_rate.sleep();
        ros::spinOnce();

    }

    if (cavity_msg_received_count_ > 0)
    {

        // ROS_INFO_STREAM("cavity_list_.objects[i].name) " << cavity_list_.objects[0].name);
        // ROS_INFO_STREAM("cavity_msg_received_count_ true  ");
        for (int i = 0; i < cavity_list_.objects.size(); i++)
        {
            ROS_INFO_STREAM("predicted cavities" << cavity_list_.objects[i].name);
            cavities_name.push_back(cavity_list_.objects[i].name);
        }

        // ROS_INFO_STREAM("cavities name " << cavities_name_sam.size());
        // ROS_INFO_STREAM("cavities name " << cavities_name_sam);
    }
    
    bool cavity_alt_approach = false;

    ROS_INFO("cavities name subscribed");
    ROS_INFO_STREAM("pose array " << pose_array.poses.size() << " cavities name " << cavities_name.size());
    //std::cout<<"cavities name " << cavities_name;
    publish_cavity_msg(pose_array, cavities_name);
    if (publish_debug_image_)
    {
        cv_bridge::CvImage debug_image_msg;
        debug_image_msg.encoding = sensor_msgs::image_encodings::RGB8;
        debug_image_msg.image = depth_debug_image;
        pub_depth_debug_image_.publish(debug_image_msg.toImageMsg());

        debug_image_msg.encoding = sensor_msgs::image_encodings::RGB8;
        debug_image_msg.image = rgb_debug_image;
        pub_rgb_debug_image_.publish(debug_image_msg.toImageMsg());

        pcl::PointCloud<pcl::PointXYZ>::Ptr cavities_combined(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < pcl_cavities.size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cavity(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(*(pcl_cavities[i]), *xyz_cavity);
            if (i == 0)
            {
                *cavities_combined = *xyz_cavity;
            }
            else
            {
                *(cavities_combined) += *xyz_cavity;
            }
        }

        sensor_msgs::PointCloud2 ros_pointcloud;
        pcl::PCLPointCloud2::Ptr pcl_cavity(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(*cavities_combined, *pcl_cavity);
        pcl_conversions::fromPCL(*pcl_cavity, ros_pointcloud);
        ros_pointcloud.header = pointcloud_msg_->header;
        pub_cavity_pointclouds_combined_.publish(pcl_cavity);
    }


    output_msg.data = std::string("e_done");
    pub_event_out_.publish(output_msg);
    return;
} 


// Based on code by Nicola Fioraio here:
// http://www.pcl-users.org/Finding-oriented-bounding-box-of-a-cloud-td4024616.html
geometry_msgs::PoseArray CavityFinderROS::estimatePose(std::vector<pcl::PCLPointCloud2::Ptr> &pcl_cavities)
{
    geometry_msgs::PoseArray pose_array;

    for (size_t i = 0; i < pcl_cavities.size(); i++)
    {
        //pcl::PCLPointCloud2::Ptr pcl_input_cloud(new pcl::PCLPointCloud2);

        //pcl_conversions::toPCL(*pointcloud_msg_, *pcl_input_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*(pcl_cavities[i]), *xyz_input_cloud);

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*xyz_input_cloud, centroid);

        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized(*xyz_input_cloud, centroid, covariance);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();

        // swap largest and second largest eigenvector so that y-axis aligns with largest eigenvector and z with the second largest
        eigen_vectors.col(0).swap(eigen_vectors.col(2));
        eigen_vectors.col(1) = eigen_vectors.col(2).cross(eigen_vectors.col(0));

        Eigen::Matrix4f eigen_vector_transform(Eigen::Matrix4f::Identity());
        eigen_vector_transform.block<3, 3>(0, 0) = eigen_vectors.transpose();
        eigen_vector_transform.block<3, 1>(0, 3) = -(eigen_vector_transform.block<3, 3>(0, 0) * centroid.head<3>());

        // transform cloud to eigenvector space
        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        pcl::transformPointCloud(*xyz_input_cloud, transformed_cloud, eigen_vector_transform);

        // find mean diagonal
        pcl::PointXYZ min_point, max_point;
        pcl::getMinMax3D(transformed_cloud, min_point, max_point);
        Eigen::Vector3f mean_diag = (max_point.getVector3fMap() + min_point.getVector3fMap()) / 2.0;

        // orientation and position of bounding box of cloud
        Eigen::Quaternionf orientation(eigen_vectors);
        Eigen::Vector3f position = eigen_vectors * mean_diag + centroid.head<3>();

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = position(0);
        pose_stamped.pose.position.y = position(1);
        pose_stamped.pose.position.z = position(2) + 0.002;
        pose_stamped.pose.orientation.w = orientation.w();
        pose_stamped.pose.orientation.x = orientation.x();
        pose_stamped.pose.orientation.y = orientation.y();
        pose_stamped.pose.orientation.z = orientation.z();
        pose_stamped.header = pointcloud_msg_->header;
        //std::cout<<"frame id "<<pose_stamped.header.frame_id<<std::endl;
        //Converting to baselink or provided link
        geometry_msgs::PoseStamped pose_in_baselink_stamped;
        std::cout<<"target frame "<<target_frame_<<std::endl;
        try
        {
            listener_.waitForTransform(target_frame_, pose_stamped.header.frame_id, pose_stamped.header.stamp, ros::Duration(3.0));
            listener_.transformPose(target_frame_, pose_stamped, pose_in_baselink_stamped);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }

        tf::Quaternion temp;
        tf::quaternionMsgToTF(pose_in_baselink_stamped.pose.orientation, temp);
        tf::Matrix3x3 m(temp);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        if (yaw < 0)
            yaw = 6.28 + yaw; // if negative make it positive
        std::cout<< "Yaw of the pose : "<<yaw<<std::endl;
        if (((yaw > 5.49) && (yaw < 0.785)) || ((yaw > 2.35) && (yaw < 3.92)))
            yaw = 0;
        else if (((yaw > 0.785) && (yaw < 2.35)) || ((yaw > 3.92) && (yaw < 5.49)))
            yaw = 1.57;

        pose_in_baselink_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
        pose_in_baselink_stamped.pose.position.z += offset_in_z_;

        pose_array.header = pose_in_baselink_stamped.header;

        //std::cout<<"baselink stampled frame id"<<pose_in_baselink_stamped.header.frame_id<<std::endl;
        pose_array.poses.push_back(pose_in_baselink_stamped.pose);
    }

    pub_pose_.publish(pose_array);
    return pose_array;
}

void CavityFinderROS::publish_cavity_msg(geometry_msgs::PoseArray& pose_array, std::vector<std::string>& cavities_name)

{
    // ROS_INFO("Inside publish cavitiy msg ");
    if (pose_array.poses.size() != cavities_name.size())
    {
        ROS_ERROR_STREAM("Error in sizes of pose array " << pose_array.poses.size() << " and cavities name " << cavities_name.size());
        return;
    }

    for (int i = 0; i< cavities_name.size(); i++)
    {

        mas_perception_msgs::Cavity cavity;

        cavity.pose.header = pose_array.header;
        cavity.pose.pose = pose_array.poses[i];
        //ROS_INFO_STREAM("pose fream id in for loop " <<pose_array.header.frame_id);
        //std::cout<<" inside for loop frame id "<<cavity.pose.header.frame_id<<std::endl;
        cavity.name = cavities_name[i];

        pub_cavity_.publish(cavity);
    }
}

void CavityFinderROS::filterCavities(std::vector<std::vector<cv::Point> > &cavities, std::vector<pcl::PCLPointCloud2::Ptr> &pcl_cavities, std::vector<cv::Point2f> &centroids)
{
    std::vector<std::vector<cv::Point> >::iterator cavities_it;
    std::vector<pcl::PCLPointCloud2::Ptr>::iterator pcl_cavities_it;
    std::vector<cv::Point2f>::iterator centroids_it;
    for (cavities_it = cavities.begin(), 
            pcl_cavities_it = pcl_cavities.begin(),
            centroids_it = centroids.begin();
            cavities_it != cavities.end(), 
            pcl_cavities_it != pcl_cavities.end(),
            centroids_it != centroids.end();)
    {
        if ((*pcl_cavities_it)->data.empty())
        {
            cavities_it = cavities.erase(cavities_it);
            pcl_cavities_it = pcl_cavities.erase(pcl_cavities_it);
            centroids_it = centroids.erase(centroids_it);
        }
        else
        {
            ++cavities_it;
            ++pcl_cavities_it;
            ++centroids_it;
        }
    }
}
