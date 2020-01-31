#include <mir_handle_detection/drawer_handle_perceiver.h>

DrawerHandlePerceiver::DrawerHandlePerceiver() : nh("~")
{
    nh.param<std::string>("output_frame", this->output_frame, "base_link_static");
    nh.param<bool>("enable_debug_pc_pub", this->enable_debug_pc_pub, "true");

    this->pc_sub = nh.subscribe("input_point_cloud", 1, &DrawerHandlePerceiver::pcCallback, this);
    this->pose_pub = nh.advertise<geometry_msgs::PoseStamped>("output_pose", 1);
    this->event_in_sub = nh.subscribe("event_in", 1, &DrawerHandlePerceiver::eventInCallback, this);
    this->event_out_pub = nh.advertise<std_msgs::String>("event_out", 1);

    if (this->enable_debug_pc_pub)
        this->pc_pub = nh.advertise<sensor_msgs::PointCloud2>("output_point_cloud", 1);

    float z_threshold_max, z_threshold_min;
    nh.param<float>("z_threshold_min", z_threshold_min, -0.1);
    nh.param<float>("z_threshold_max", z_threshold_max, 0.3);
    this->passthrough_filter_z.setFilterFieldName ("z");
    this->passthrough_filter_z.setFilterLimits (z_threshold_min, z_threshold_max);

    float y_threshold_max, y_threshold_min;
    nh.param<float>("y_threshold_min", y_threshold_min, -0.1);
    nh.param<float>("y_threshold_max", y_threshold_max, 0.1);
    this->passthrough_filter_y.setFilterFieldName ("y");
    this->passthrough_filter_y.setFilterLimits (y_threshold_min, y_threshold_max);

    float leaf_size_x, leaf_size_y, leaf_size_z;
    nh.param<float>("leaf_size_x", leaf_size_x, 0.01);
    nh.param<float>("leaf_size_y", leaf_size_y, 0.01);
    nh.param<float>("leaf_size_z", leaf_size_z, 0.01);
    this->voxel_grid_filter.setLeafSize (leaf_size_x, leaf_size_y, leaf_size_z);

    float seg_dist_threshold;
    nh.param<float>("seg_dist_threshold", seg_dist_threshold, 0.01);
    this->seg.setOptimizeCoefficients(true);
    this->seg.setModelType(pcl::SACMODEL_PLANE);
    this->seg.setMethodType(pcl::SAC_RANSAC);
    this->seg.setDistanceThreshold(seg_dist_threshold);

    this->project_inliers.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
    this->project_inliers.setCopyAllData(false);

    float polygon_prism_height_min, polygon_prism_height_max;
    nh.param<float>("polygon_prism_height_min", polygon_prism_height_min, 0.01);
    nh.param<float>("polygon_prism_height_max", polygon_prism_height_max, 0.1);
    this->extract_polygonal_prism.setHeightLimits(polygon_prism_height_min, polygon_prism_height_max);

    this->extract_indices.setNegative(false);

    float cluster_tolerance;
    int min_cluster_size, max_cluster_size;
    nh.param<float>("cluster_tolerance", cluster_tolerance, 0.02);
    nh.param<int>("min_cluster_size", min_cluster_size, 50);
    nh.param<int>("max_cluster_size", max_cluster_size, 10000);
    this->euclidean_cluster_extraction.setClusterTolerance(cluster_tolerance);
    this->euclidean_cluster_extraction.setMinClusterSize(min_cluster_size);
    this->euclidean_cluster_extraction.setMaxClusterSize(max_cluster_size);

    this->listening = false;
}

DrawerHandlePerceiver::~DrawerHandlePerceiver()
{
}

void DrawerHandlePerceiver::eventInCallback(const std_msgs::String::ConstPtr &msg)
{
    if (msg->data == "e_start")
    {
        ROS_INFO_STREAM("starting listening");
        this->listening = true;
    }
    else if (msg->data == "e_stop")
    {
        ROS_INFO_STREAM("stoping listening");
        this->listening = false;
    }
}

void DrawerHandlePerceiver::pcCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if (!this->listening)
        return;

    sensor_msgs::PointCloud2 msg_transformed;
    bool success = this->transformPC(msg, msg_transformed);
    if (!success)
        return;

    PCloudT::Ptr pc_input(new PCloudT);
    pcl::fromROSMsg(msg_transformed, *pc_input);

    PCloudT::Ptr pc_downsampled(new PCloudT);
    this->downSamplePC(pc_input, pc_downsampled, 3);

    PCloudT::Ptr pc_passthrough_filtered(new PCloudT);
    this->passthroughFilterPC(pc_downsampled, pc_passthrough_filtered);

    PCloudT::Ptr pc_filtered(new PCloudT);
    this->voxel_grid_filter.setInputCloud(pc_passthrough_filtered);
    this->voxel_grid_filter.filter(*pc_filtered);

    PCloudT::Ptr pc_segmented(new PCloudT);
    this->extractPlaneOutlier(pc_filtered, pc_passthrough_filtered, pc_segmented);

    Eigen::Vector4f closest_centroid(0.0, 0.0, 0.0, 0.0);
    bool cluster_success = this->getClosestCluster(pc_segmented, closest_centroid);
    if (!cluster_success)
        return;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = closest_centroid[0];
    pose_stamped.pose.position.y = closest_centroid[1];
    pose_stamped.pose.position.z = closest_centroid[2];
    pose_stamped.pose.orientation.w = 1.0;
    pose_stamped.header.frame_id = this->output_frame;
    pose_stamped.header.stamp = ros::Time::now();
    this->pose_pub.publish(pose_stamped);

    std_msgs::String event_out_msg;
    event_out_msg.data = "e_done";
    this->event_out_pub.publish (event_out_msg);
    ROS_INFO("[drawer_handle_perceiver] SUCCESSFULL");

    this->listening = false;

    if (this->enable_debug_pc_pub)
    {
        /* publish debug pointcloud */
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*pc_segmented, output);
        output.header.frame_id = this->output_frame;
        output.header.stamp = ros::Time::now();
        this->pc_pub.publish(output);
        ROS_INFO("Publishing debug pointcloud");
    }
}

bool DrawerHandlePerceiver::transformPC(const sensor_msgs::PointCloud2::ConstPtr &msg, sensor_msgs::PointCloud2 &msg_transformed)
{
    msg_transformed.header.frame_id = this->output_frame;
    try
    {
        ros::Time common_time;
        this->tf_listener.getLatestCommonTime(this->output_frame, msg->header.frame_id, common_time, NULL);
        tf::StampedTransform transform;
        this->tf_listener.waitForTransform(this->output_frame, msg->header.frame_id,
                                           common_time, ros::Duration(1.0));
        this->tf_listener.lookupTransform(this->output_frame, msg->header.frame_id,
                                          common_time, transform);
        pcl_ros::transformPointCloud(this->output_frame, transform, *msg, msg_transformed);
        return true;
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("PCL transform error: %s", ex.what());
        ros::Duration(1.0).sleep();
        return false;
    }
}

void DrawerHandlePerceiver::downSamplePC(PCloudT::Ptr input, PCloudT::Ptr output, int scale)
{
    output->width = input->width / scale;
    output->height = input->height / scale;
    output->points.resize(output->width * output->height);
    for( size_t i = 0, ii = 0; i < output->height; ii += scale, i++)
    {
        for( size_t j = 0, jj = 0; j < output->width; jj += scale, j++)
        {
            output->at(j, i) = input->at(jj, ii); //at(column, row)
        }
    }
}

void DrawerHandlePerceiver::passthroughFilterPC(PCloudT::Ptr input, PCloudT::Ptr output)
{
    PCloudT::Ptr pc_intermediate(new PCloudT);
    this->passthrough_filter_z.setInputCloud(input);
    this->passthrough_filter_z.filter(*pc_intermediate);
    this->passthrough_filter_y.setInputCloud(pc_intermediate);
    this->passthrough_filter_y.filter(*output);
}

void DrawerHandlePerceiver::extractPlaneOutlier(PCloudT::Ptr input, PCloudT::Ptr dense_input, PCloudT::Ptr output)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    this->seg.setInputCloud(input);
    this->seg.segment(*inliers, *coefficients);

    //Project plane model inliers to plane
    PCloudT::Ptr pc_plane(new PCloudT);
    this->project_inliers.setInputCloud(input);
    this->project_inliers.setModelCoefficients(coefficients);
    this->project_inliers.setIndices(inliers);
    this->project_inliers.filter(*pc_plane);

    //Compute plane convex hull
    PCloudT::Ptr pc_hull(new PCloudT);
    this->convex_hull.setInputCloud(pc_plane);
    this->convex_hull.reconstruct(*pc_hull);

    //Extract points inside polygonal prism of plane convex hull
    pcl::PointIndices::Ptr segmented_cloud_inliers(new pcl::PointIndices);
    this->extract_polygonal_prism.setInputPlanarHull(pc_hull);
    this->extract_polygonal_prism.setInputCloud(dense_input);
    this->extract_polygonal_prism.segment(*segmented_cloud_inliers);

    // Extract the outliers
    this->extract_indices.setInputCloud(dense_input);
    this->extract_indices.setIndices(segmented_cloud_inliers);
    this->extract_indices.filter(*output);
}

bool DrawerHandlePerceiver::getClosestCluster(PCloudT::Ptr input, Eigen::Vector4f &closest_centroid)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    std::vector<pcl::PointIndices> clusters_indices;
    this->euclidean_cluster_extraction.setSearchMethod(tree);
    this->euclidean_cluster_extraction.setInputCloud(input);
    this->euclidean_cluster_extraction.extract(clusters_indices);

    if (clusters_indices.size() == 0)
        return false;

    float closest_dist = 100.0;
    Eigen::Vector4f zero_point(0.0, 0.0, 0.0, 0.0);
    for (size_t i = 0; i < clusters_indices.size(); i++)
    {
        const pcl::PointIndices& cluster_indices = clusters_indices[i];
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*input, cluster_indices, centroid);
        float dist = pcl::L2_Norm(centroid, zero_point, 3);
        if (dist < closest_dist)
        {
            closest_dist = dist;
            closest_centroid = centroid;
        }
    }
    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "drawer_handle_perceiver");
    DrawerHandlePerceiver dhperceiver;
    ros::Rate loop_rate(10.0);
    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
