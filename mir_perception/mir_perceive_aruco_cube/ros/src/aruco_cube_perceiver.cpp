#include <mir_perceive_aruco_cube/aruco_cube_perceiver.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <algorithm>
#include <math.h>

ArucoCubePerceiver::ArucoCubePerceiver() : nh("~"), image_transporter_(nh)
{
    nh.param<std::string>("target_frame", this->target_frame_, "/base_link");
    nh.param<int>("num_of_retries", this->num_of_retries_, 30);
    nh.param<bool>("debug", this->debug_, false);

    image_pub_ = image_transporter_.advertise("debug_image", 1);

    sub_pointcloud_.subscribe(nh, "input_pointcloud", 10);
    sub_image_.subscribe(nh, "input_image", 10);

    sync_input_ = boost::make_shared<message_filters::Synchronizer<ImageSyncPolicy> > (10);
    sync_input_->connectInput(sub_pointcloud_, sub_image_);
    sync_input_->registerCallback(boost::bind(&ArucoCubePerceiver::synchronizedCallback, this, _1, _2));

    this->debug_point_pub_ = nh.advertise<geometry_msgs::PointStamped>("debug", 1);
    this->output_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("object_pose", 1);

    this->aruco_dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
}

ArucoCubePerceiver::~ArucoCubePerceiver()
{
}

void ArucoCubePerceiver::synchronizedCallback(const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg,
                                              const sensor_msgs::Image::ConstPtr &image_msg)
{
    cv_bridge::CvImagePtr input_img_ptr;
    bool success = this->imgToCV(image_msg, input_img_ptr);
    if (!success)
    {
        ROS_ERROR("[aruco_cube_perceiver] Could not convert sensor_msgs::Image to cv::Image.");
        return;
    }

    std::vector<cv::Point2f> corners;
    bool detection_success = this->getBestArucoMarkerCorner(input_img_ptr, corners);
    if (!detection_success)
    {
        ROS_ERROR("[aruco_cube_perceiver] Could not find Aruco markers in image.");
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pointcloud_msg, *pc_input);
    if (input_img_ptr->image.rows != pc_input->height || input_img_ptr->image.cols != pc_input->width)
    {
        ROS_WARN("[aruco_cube_perceiver] Image size did not match PointCloud size.");
        return;
    }

    /* create a point cloud of aruco square */
    pcl::PointCloud<pcl::PointXYZ>::Ptr aruco_square(new pcl::PointCloud<pcl::PointXYZ>);
    aruco_square->width = 2;
    aruco_square->height = 2;
    aruco_square->points.resize(4);
    for (int i = 0; i < corners.size(); ++i) {
        int col = corners[i].x;
        int row = corners[i].y;
        pcl::PointXYZ point = pc_input->points[row*pc_input->width + col];
        aruco_square->points[i] = point;
        if (!pcl::isFinite(point))
        {
            ROS_WARN("[aruco_cube_perceiver] One of the corner of aruco marker is NaN.");
            return;
        }
    }

    pcl::PointXYZ cube_center;
    this->calculateCenterOfArucoCube(aruco_square, cube_center);

    geometry_msgs::PoseStamped pose, transformed_pose;
    pose.header.frame_id = pointcloud_msg->header.frame_id;
    pose.pose.position.x = cube_center.x;
    pose.pose.position.y = cube_center.y;
    pose.pose.position.z = cube_center.z;
    /* TODO: remove hardcoded orientation */
    pose.pose.orientation.w = 1.0;
    /* pose.pose.orientation.x = -0.047304; */
    /* pose.pose.orientation.y = 0.2083682; */
    /* pose.pose.orientation.z = 0.9562657; */
    /* pose.pose.orientation.w = 0.1997519; */
    /* pose.pose.orientation = quat; */
    this->transformPose(pose, transformed_pose);
    /* this->debug_point_pub_.publish(output_ps); */
    this->output_pose_pub_.publish(transformed_pose);
    /* pcl::PointXYZ diff = aruco_square->points[0] - aruco_square->points[1]; */
}

void ArucoCubePerceiver::calculateCenterOfArucoCube(pcl::PointCloud<pcl::PointXYZ>::Ptr aruco_square,
                                                    pcl::PointXYZ &cube_center)
{
    Eigen::Vector4f aruco_square_center;
    pcl::compute3DCentroid(*aruco_square, aruco_square_center);
    float side_length = this->euclideanDistance(aruco_square->points[0],
                                                aruco_square->points[1]);
    Eigen::Vector4f normal;
    float curvature;
    computePointNormal(*aruco_square, {0, 1, 2, 3}, normal, curvature);
    float lambda = side_length / 2;
    pcl::PointXYZ possible_cube_center_1(aruco_square_center.x() + lambda * normal.x(),
                                         aruco_square_center.y() + lambda * normal.y(),
                                         aruco_square_center.z() + lambda * normal.z());
    pcl::PointXYZ possible_cube_center_2(aruco_square_center.x() - lambda * normal.x(),
                                         aruco_square_center.y() - lambda * normal.y(),
                                         aruco_square_center.z() - lambda * normal.z());
    pcl::PointXYZ camera;
    float dist_1 = this->euclideanDistance(possible_cube_center_1, camera);
    float dist_2 = this->euclideanDistance(possible_cube_center_2, camera);
    cube_center = (dist_1 > dist_2) ? possible_cube_center_1 : possible_cube_center_2;
}

bool ArucoCubePerceiver::transformPose(geometry_msgs::PoseStamped &pose,
                                       geometry_msgs::PoseStamped &transformed_pose)
{
    try
    {
        ros::Time common_time;
        this->tf_listener_.getLatestCommonTime(this->target_frame_,
                                               pose.header.frame_id,
                                               common_time, NULL);
        this->tf_listener_.waitForTransform(this->target_frame_,
                                            pose.header.frame_id,
                                            common_time, ros::Duration(3.0));
        pose.header.stamp = common_time;
        this->tf_listener_.transformPose(this->target_frame_, pose, transformed_pose);
        return true;
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("PCL transform error: %s", ex.what());
        ros::Duration(1.0).sleep();
        return false;
    }
}

bool ArucoCubePerceiver::transformPoint(geometry_msgs::PointStamped &point,
                                       geometry_msgs::PointStamped &transformed_point)
{
    try
    {
        ros::Time common_time;
        this->tf_listener_.getLatestCommonTime(this->target_frame_,
                                               point.header.frame_id,
                                               common_time, NULL);
        this->tf_listener_.waitForTransform(this->target_frame_,
                                            point.header.frame_id,
                                            common_time, ros::Duration(3.0));
        point.header.stamp = common_time;
        this->tf_listener_.transformPoint(this->target_frame_, point, transformed_point);
        return true;
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("PCL transform error: %s", ex.what());
        ros::Duration(1.0).sleep();
        return false;
    }
}

bool ArucoCubePerceiver::getBestArucoMarkerCorner(cv_bridge::CvImagePtr &img_ptr,
                                                  std::vector<cv::Point2f> &corners)
{
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::aruco::detectMarkers(img_ptr->image, this->aruco_dictionary, marker_corners, marker_ids);
    if (marker_ids.size() == 0)
        return false;


    std::vector<float> variances;
    this->calculateVariances(marker_corners, variances);
    std::vector<float>::iterator max_var_it = max_element(variances.begin(), variances.end());
    int best_index = std::distance(variances.begin(), max_var_it);
    corners = marker_corners[best_index];
    
    if (this->debug_)
    {
        std::cout << std::endl << "Detected Marker: " << std::endl;
        for (int i = 0; i < marker_ids.size(); ++i) {
            std::cout << "ID: " << marker_ids[i];
            if (i == best_index)
                std::cout << " [SELECTED]";
            std::cout << std::endl;
            std::cout << "Corners: " << std::endl;
            for (cv::Point2f p : marker_corners[i])
                std::cout << p << std::endl;
        }
        std::cout << std::endl;
        cv::aruco::drawDetectedMarkers(img_ptr->image, marker_corners, marker_ids);
        image_pub_.publish(img_ptr->toImageMsg());
    }
    return true;
}

void ArucoCubePerceiver::calculateVariances(std::vector<std::vector<cv::Point2f>> &marker_corners,
                                            std::vector<float> &variances)
{
    variances.clear();
    for (int i = 0; i < marker_corners.size(); ++i) {
        /* calculate mean */
        cv::Point2f mean;
        for (int j = 0; j < marker_corners[i].size(); ++j) {
            mean += marker_corners[i][j];
        }
        mean *= 1.0/marker_corners[i].size();

        /* calculate variance */
        float squared_sum = 0.0;
        for (int j = 0; j < marker_corners[i].size(); ++j) {
            cv::Point2f diff = marker_corners[i][j] - mean;
            squared_sum += pow(diff.x, 2) + pow(diff.y, 2);
        }
        float var = squared_sum/marker_corners[i].size();
        variances.push_back(var);
    }
}

bool ArucoCubePerceiver::imgToCV(const sensor_msgs::Image::ConstPtr &image_msg,
                                 cv_bridge::CvImagePtr &cv_ptr)
{
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }
    return true;
}

float ArucoCubePerceiver::euclideanDistance(pcl::PointXYZ &p1, pcl::PointXYZ &p2)
{
    return sqrt(pow(p1.x-p2.x, 2) + pow(p1.y-p2.y, 2) + pow(p1.z-p2.z, 2));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_cube_perceiver");
    ArucoCubePerceiver perceiver;

    ros::Rate loop_rate(10.0);

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
