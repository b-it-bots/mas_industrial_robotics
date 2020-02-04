#include <mir_perceive_aruco_cube/aruco_cube_perceiver.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <tf/transform_datatypes.h>
/* #include <tf/LinearMath/Quaternion.h> */
/* #include <tf/LinearMath/Vector3.h> */
#include <algorithm>
#include <math.h>

ArucoCubePerceiver::ArucoCubePerceiver() : nh("~"), image_transporter_(nh)
{
    nh.param<std::string>("target_frame", this->target_frame_, "/base_link");
    nh.param<int>("num_of_retries", this->num_of_retries_, 30);
    nh.param<bool>("debug", this->debug_, false);

    this->image_pub_ = image_transporter_.advertise("debug_image", 1);

    this->sub_pointcloud_.subscribe(nh, "input_pointcloud", 10);
    this->sub_image_.subscribe(nh, "input_image", 10);

    this->sync_input_ = boost::make_shared<message_filters::Synchronizer<ImageSyncPolicy> > (10);
    this->sync_input_->connectInput(sub_pointcloud_, sub_image_);
    this->sync_input_->registerCallback(boost::bind(&ArucoCubePerceiver::synchronizedCallback, this, _1, _2));

    this->debug_polygon_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("debug_polygon", 1);
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
        ROS_ERROR("[aruco_cube_perceiver] Image size did not match PointCloud size.");
        return;
    }

    /* create a point cloud out of the corners of detected aruco square */
    pcl::PointCloud<pcl::PointXYZ>::Ptr aruco_square(new pcl::PointCloud<pcl::PointXYZ>);
    aruco_square->width = 2;
    aruco_square->height = 2;
    aruco_square->points.resize(4);
    for (int i = 0; i < corners.size(); ++i)
    {
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

    geometry_msgs::Point cube_center;
    this->calculateCenterOfArucoCube(aruco_square, cube_center);

    geometry_msgs::Quaternion quat;
    this->calculateArucoOrientation(aruco_square->points[0], aruco_square->points[1],
                                    aruco_square->points[3], quat);

    geometry_msgs::PoseStamped pose, transformed_pose;
    pose.header.frame_id = pointcloud_msg->header.frame_id;
    pose.header.stamp = ros::Time::now();
    pose.pose.position = cube_center;
    pose.pose.orientation = quat;
    bool transform_success = this->transformPose(pose, transformed_pose);
    if (!transform_success)
    {
        ROS_ERROR("[aruco_cube_perceiver] Could not transform pose to target_frame.");
        return;
    }
    this->output_pose_pub_.publish(transformed_pose);

    if (this->debug_)
    {
        geometry_msgs::PolygonStamped poly;
        poly.header.frame_id = pointcloud_msg->header.frame_id;
        poly.header.stamp = ros::Time::now();
        for (int i = 0; i < corners.size(); ++i)
        {
            geometry_msgs::Point32 p;
            p.x = aruco_square->points[i].x;
            p.y = aruco_square->points[i].y;
            p.z = aruco_square->points[i].z;
            poly.polygon.points.push_back(p);
        }
        this->debug_polygon_pub_.publish(poly);
    }
}

/*
 * 1) Calculate the center of the square formed by aruco marker corners
 * 2) Calculate the side_length of the aruco marker
 * 3) Calculate the normal to the plane formed by 4 corners
 * 4) Find any point on the line passing through the center and parallel to
 *    normal to the plane with equation `new_point = center + dist_from_center*normal`
 *    where `dist_from_center = side_length / 2` (obviously)
 * 5) Step 4 would provide 2 points on normal which are side_lenght/2 meters
 *    away from center (one which is actually the cube center and the other one
 *    is outside the cube (between camera and center))
 * 6) Find the point furthest from camera frame out of the 2 points from Step 5
 *    and convert to geometry_msgs::Point
 */
void ArucoCubePerceiver::calculateCenterOfArucoCube(pcl::PointCloud<pcl::PointXYZ>::Ptr aruco_square,
                                                    geometry_msgs::Point &cube_center)
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
    pcl::PointXYZ cube_center_pcl = (dist_1 > dist_2) ? possible_cube_center_1 : possible_cube_center_2;
    cube_center.x = cube_center_pcl.x;
    cube_center.y = cube_center_pcl.y;
    cube_center.z = cube_center_pcl.z;
}

/*
 * If the detected aruco marker corners are 'a', 'b', 'c' and 'd' in clockwise
 * direction then the orientation is such that the X axis is in direction formed
 * by vector b-a and Y axis is formed by vector d-a.
 * Note: Corner 'a' is top left corner when aruco marker is on a surface where X
 * axis is pointing right and Y is pointing top and Z is pointing towards the viewer.
 */
void ArucoCubePerceiver::calculateArucoOrientation(pcl::PointXYZ &a, pcl::PointXYZ &b,
                                                   pcl::PointXYZ &d, geometry_msgs::Quaternion &quat)
{
    Eigen::Vector3f X(b.x - a.x, b.y - a.y, b.z - a.z);
    Eigen::Vector3f Y(d.x - a.x, d.y - a.y, d.z - a.z);
    X.normalize();
    Y.normalize();
    Eigen::Vector3f Z = X.cross(Y);
    Eigen::Matrix3f mat;
    mat << X[0], Y[0], Z[0],
           X[1], Y[1], Z[1],
           X[2], Y[2], Z[2];
    Eigen::Quaternionf quaternion(mat);
    quat.x = quaternion.x();
    quat.y = quaternion.y();
    quat.z = quaternion.z();
    quat.w = quaternion.w();
}

/*
 * Transform geometry_msgs::PoseStamped from any frame to target_frame_
 */
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

/*
 * Detect all the aruco markers in the given image and assign the corners of the
 * marker which has the highest area (highest variance) to `corners`.
 * Returns false if no aruco markers were found.
 */
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

/*
 * Convert sensor_msgs::Image::ConstPtr to cv_bridge::CvImagePtr
 * Necessary to perform operations on the image using opencv library.
 */
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

/*
 * Find euclidean distance between 2 pcl::PointXYZ objects.
 */
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
