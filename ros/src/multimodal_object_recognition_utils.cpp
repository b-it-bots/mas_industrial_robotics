//get3DBoundingBox
//get2DBoundingBox
//get3DPose

#include <mir_object_recognition/multimodal_object_recognition_utils.h>

MultimodalObjectRecognitionUtils::MultimodalObjectRecognitionUtils(boost::shared_ptr<tf::TransformListener> tf_listener):
    tf_listener_(tf_listener)
{    
    if (!tf_listener_)
    {
      ROS_ERROR_THROTTLE(2.0, "[MultimodalObjectRecognitionUtils]: TF listener not initialized.");
    }
}

MultimodalObjectRecognitionUtils::~MultimodalObjectRecognitionUtils()
{
}

geometry_msgs::PoseStamped MultimodalObjectRecognitionUtils::estimatePose(
                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &xyz_input_cloud, 
                    std::string name = "None")
{
    // If is not object name is not m20, m30, bearing, distance tube,
    // do passtrhough to filter
    // TODO: Pose zero after filter
    bool use_filter = true;
    pcl::PointCloud<pcl::PointXYZRGB> filtered_cloud;
    // With filter, remove points belonging to plane for long objects
    if (name == "M30" || 
        name == "M20" ||
        name == "DISTANCE_TUBE" ||
        name == "BEARING") 
    {
        filtered_cloud = *xyz_input_cloud;
    }
    else
    {
        pcl::PassThrough<PointT> pass_through;
        pass_through.setFilterFieldName("z");
        pcl::PointXYZRGB min_pt;
        pcl::PointXYZRGB max_pt;
        pcl::getMinMax3D(*xyz_input_cloud, min_pt, max_pt);
        ROS_INFO_STREAM("Min and max z before filter: "<<min_pt.z<<", "<<max_pt.z);
        double limit_min = min_pt.z + 0.0060;
        double limit_max = max_pt.z;
        pass_through.setFilterLimits(limit_min, limit_max);
        pass_through.setInputCloud(xyz_input_cloud);
        pass_through.filter(filtered_cloud);
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(filtered_cloud, centroid);

    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(filtered_cloud, centroid, covariance);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();

    // swap largest and second largest eigenvector so that y-axis aligns with largest eigenvector and z with the second largest
    eigen_vectors.col(0).swap(eigen_vectors.col(2));
    eigen_vectors.col(1) = eigen_vectors.col(2).cross(eigen_vectors.col(0));

    Eigen::Matrix4f eigen_vector_transform(Eigen::Matrix4f::Identity());
    eigen_vector_transform.block<3, 3>(0, 0) = eigen_vectors.transpose();
    eigen_vector_transform.block<3, 1>(0, 3) = -(eigen_vector_transform.block<3, 3>(0, 0) * centroid.head<3>());

    // transform cloud to eigenvector space
    pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud;
    pcl::transformPointCloud(filtered_cloud, transformed_cloud, eigen_vector_transform);

    // find mean diagonal
    pcl::PointXYZRGB min_point, max_point;
    pcl::getMinMax3D(transformed_cloud, min_point, max_point);
    Eigen::Vector3f mean_diag = (max_point.getVector3fMap() + min_point.getVector3fMap()) / 2.0;

    // orientation and position of bounding box of cloud
    Eigen::Quaternionf orientation(eigen_vectors);
    Eigen::Vector3f position = eigen_vectors * mean_diag + centroid.head<3>();

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = position(0);
    pose_stamped.pose.position.y = position(1);
    pose_stamped.pose.position.z = position(2);
    /* pose_stamped.pose.position.z = pointcloud_segmentation_->getWorkspaceHeight(); */
    pose_stamped.pose.orientation.w = orientation.w();
    pose_stamped.pose.orientation.x = orientation.x();
    pose_stamped.pose.orientation.y = orientation.y();
    pose_stamped.pose.orientation.z = orientation.z();

    tf::Quaternion temp;
    tf::quaternionMsgToTF(pose_stamped.pose.orientation, temp);
    tf::Matrix3x3 m(temp);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);

    return pose_stamped;
}

void MultimodalObjectRecognitionUtils::adjustContainerPose(mas_perception_msgs::Object &container_object,
                                                           float rgb_container_height=0.1)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(container_object.pointcloud, *cloud);
    //find min and max z
    pcl::PointXYZRGB min_pt;
    pcl::PointXYZRGB max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    ROS_INFO_STREAM("Min and max z "<<min_pt.z<<", "<<max_pt.z); 
    
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (min_pt.z, (min_pt.z + (min_pt.z + max_pt.z/2))/2 );
    pass.filter (*indices);

    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    reg.setMinClusterSize (300);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (200);
    reg.setInputCloud (cloud);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    pcl::PointIndices::Ptr filtered_cluster (new pcl::PointIndices() );
    int largest_index, largest_cluster;
    if (clusters.size() == 0)
    {
        return;
    }
    for(size_t i=0; i<clusters.size(); i++)
    {
        if(i == 0)
        {
            largest_index = 0;
            largest_cluster = clusters[0].indices.size();
        }
        else
        {
            if (largest_cluster <= clusters[i].indices.size())
            {
                largest_index = i;
                largest_cluster = clusters[i].indices.size();	
            }
        }
    }
    int counter = 0;
    while (counter < clusters[largest_index].indices.size ())
    {
        filtered_cluster->indices.push_back(clusters[largest_index].indices[counter]); 
        counter++;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    //point indices to cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> c_filter (true);
    c_filter.setInputCloud (cloud);
    c_filter.setIndices (filtered_cluster);
    c_filter.filter (*cloud_filtered); 
    
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    sensor_msgs::PointCloud2 ros_pointcloud;
    pcl::PCLPointCloud2::Ptr cloud_cl(new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*cloud_filtered, *cloud_cl);
    pcl_conversions::fromPCL(*cloud_cl, ros_pointcloud);

    //ros_pointcloud.header.frame_id = target_frame_id_;    
    //pub_pcl_cluster_.publish(ros_pointcloud);

    Eigen::Vector4f centroid;
    unsigned int valid_points = pcl::compute3DCentroid(*cloud_filtered, centroid);

    container_object.pose.pose.position.x = centroid[0];
    container_object.pose.pose.position.y = centroid[1];
    container_object.pose.pose.position.z = max_pt.z + rgb_container_height;
}

void MultimodalObjectRecognitionUtils::adjustAxisBoltPose(mas_perception_msgs::Object &object)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(object.pointcloud, *xyz_cloud);
    int pcl_point_size = object.pointcloud.height * object.pointcloud.width;
    pcl::PointXYZ min_pt;
    pcl::PointXYZ max_pt;

    pcl::getMinMax3D(*xyz_cloud, min_pt, max_pt);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_at_z(new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Vector4f centroid;
    for (size_t i=0; i<pcl_point_size; i++ )
    {
        if (xyz_cloud->points[i].z == max_pt.z)
        {
            point_at_z->points.push_back(xyz_cloud->points[i]);
        }
    }
    unsigned int valid_points = pcl::compute3DCentroid(*point_at_z, centroid);
    if (object.name == "M20_100")
    {
        ROS_INFO_STREAM("Updating M20_100 pose from object id: "<<object.database_id);
        float midpoint_x = (object.pose.pose.position.x + centroid[0])/2;
        float midpoint_y = (object.pose.pose.position.y + centroid[1])/2;
        object.pose.pose.position.x = midpoint_x;
        object.pose.pose.position.y = midpoint_y;
    }
    else if (object.name == "AXIS")
    { 
        ROS_INFO_STREAM("Updating AXIS pose from object id: "<<object.database_id);
        object.pose.pose.position.x = centroid[0];
        object.pose.pose.position.y = centroid[1]; 
    }
}

void MultimodalObjectRecognitionUtils::transformPose(std::string &source_frame, std::string &target_frame, 
                                            geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped &transformed_pose)
{
    try
    {
        ros::Time common_time;
        tf_listener_->getLatestCommonTime(source_frame, target_frame, common_time, NULL);
        pose.header.stamp = common_time;
        tf_listener_->waitForTransform(target_frame, source_frame, common_time, ros::Duration(0.1));
        tf_listener_->transformPose(target_frame, pose, transformed_pose);
    }
    catch(tf::LookupException& ex)
    {
        ROS_WARN("Failed to transform pose: (%s)", ex.what());
        transformed_pose = pose;
    }

}


void MultimodalObjectRecognitionUtils::saveDebugImage(const cv_bridge::CvImagePtr &cv_image_bbox_ptr, 
                                                      const sensor_msgs::ImageConstPtr &raw_image,
                                                      std::string logdir)
{
    ROS_WARN_STREAM("Saving raw image and bbox information");
    std::stringstream filename; // stringstream used for the conversion
    ros::Time time_now = ros::Time::now();

    // save image
    filename.str("");
    filename << logdir << time_now << "_bbox_rgb" <<".jpg";
    cv::imwrite(filename.str(), cv_image_bbox_ptr->image);

    cv_bridge::CvImagePtr raw_cv_image;
    try
    {
        raw_cv_image = cv_bridge::toCvCopy(raw_image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    filename.str("");
    filename << logdir << time_now << "_raw_rgb" <<".jpg";
    cv::imwrite(filename.str(), raw_cv_image->image);

}
