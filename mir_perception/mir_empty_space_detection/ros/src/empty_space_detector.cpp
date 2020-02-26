#include <mir_empty_space_detection/empty_space_detector.h>

EmptySpaceDetector::EmptySpaceDetector() : nh_("~")
{
    nh_.param<std::string>("output_frame", output_frame_, "base_link");
    nh_.param<bool>("enable_debug_pc_pub", enable_debug_pc_pub_, true);
    float octree_resolution;
    nh_.param<float>("octree_resolution", octree_resolution, 0.0025);
    this->add_to_octree_ = true;

    this->pc_sub_ = nh_.subscribe("input_point_cloud", 1, &EmptySpaceDetector::pcCallback, this);
    this->pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("output_pose", 1);
    this->event_in_sub_ = nh_.subscribe("event_in", 1, &EmptySpaceDetector::eventInCallback, this);
    this->event_out_pub_ = nh_.advertise<std_msgs::String>("event_out", 1);

    if (enable_debug_pc_pub_)
    {
        this->pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("output_point_cloud", 1);
    }

    cloud_accumulation_ = CloudAccumulation::UPtr(new CloudAccumulation(octree_resolution));
    scene_segmentation_ = SceneSegmentationSPtr(new SceneSegmentation());
    this->loadParams();

    tf_listener_.reset(new tf::TransformListener);
}

EmptySpaceDetector::~EmptySpaceDetector()
{
}

void EmptySpaceDetector::loadParams()
{
    /* float z_threshold_max, z_threshold_min; */
    /* nh.param<float>("z_threshold_min", z_threshold_min, -0.1); */
    /* nh.param<float>("z_threshold_max", z_threshold_max, 0.3); */

    /* float y_threshold_max, y_threshold_min; */
    /* nh.param<float>("y_threshold_min", y_threshold_min, -0.1); */
    /* nh.param<float>("y_threshold_max", y_threshold_max, 0.1); */
    /* scene_segmentation_->setPassthroughParams(enable_passthrough_filter, */
    /*         passthrough_filter_field_name, */
    /*         passthrough_filter_limit_min, */
    /*         passthrough_filter_limit_max); */
    scene_segmentation_->setPassthroughParams(false, "z", 0.0, 0.0);

    float voxel_leaf_size, voxel_filter_limit_min, voxel_filter_limit_max;
    std::string voxel_filter_field_name;
    nh_.param<float>("voxel_leaf_size", voxel_leaf_size, 1.0);
    nh_.param<std::string>("voxel_filter_field_name", voxel_filter_field_name, "z");
    nh_.param<float>("voxel_filter_limit_min", voxel_filter_limit_min, -0.15);
    nh_.param<float>("voxel_filter_limit_max", voxel_filter_limit_max, 0.3);
    scene_segmentation_->setVoxelGridParams(voxel_leaf_size, 
            voxel_filter_field_name,
            voxel_filter_limit_min, 
            voxel_filter_limit_max);

    float normal_radius_search;
    bool use_omp;
    int num_cores;
    nh_.param<float>("normal_radius_search", normal_radius_search, 0.03);
    nh_.param<bool>("use_omp", use_omp, false);
    nh_.param<int>("num_cores", num_cores, 4);
    scene_segmentation_->setNormalParams(normal_radius_search, use_omp, num_cores);

    int sac_max_iterations;
    float sac_distance_threshold, sac_x_axis, sac_y_axis, sac_z_axis, sac_eps_angle;
    float sac_normal_distance_weight;
    bool sac_optimize_coefficients;
    nh_.param<int>("sac_max_iterations", sac_max_iterations, 1000);
    nh_.param<float>("sac_distance_threshold", sac_distance_threshold, 0.01);
    nh_.param<bool>("sac_optimize_coefficients", sac_optimize_coefficients, true);
    nh_.param<float>("sac_x_axis", sac_x_axis, 0.0);
    nh_.param<float>("sac_y_axis", sac_y_axis, 0.0);
    nh_.param<float>("sac_z_axis", sac_z_axis, 1.0);
    Eigen::Vector3f axis(sac_x_axis, sac_y_axis, sac_z_axis); 
    nh_.param<float>("sac_eps_angle", sac_eps_angle, 0.09);
    nh_.param<float>("sac_normal_distance_weight", sac_normal_distance_weight, 0.05);
    scene_segmentation_->setSACParams(sac_max_iterations, 
            sac_distance_threshold,
            sac_optimize_coefficients,
            axis, 
            sac_eps_angle,
            sac_normal_distance_weight);

    float prism_min_height, prism_max_height;
    nh_.param<float>("prism_min_height", prism_min_height, 0.01);
    nh_.param<float>("prism_max_height", prism_max_height, 0.1);
    scene_segmentation_->setPrismParams(prism_min_height, prism_max_height);

    int outlier_min_neighbors;
    float outlier_radius_search;
    nh_.param<int>("outlier_min_neighbors", outlier_min_neighbors, 20);
    nh_.param<float>("outlier_radius_search", outlier_radius_search, 0.03);
    scene_segmentation_->setOutlierParams(outlier_radius_search, outlier_min_neighbors);

    float cluster_tolerance, cluster_min_height, cluster_max_height, cluster_max_length;
    float cluster_min_distance_to_polygon;
    int cluster_min_size, cluster_max_size;
    nh_.param<float>("cluster_tolerance", cluster_tolerance, 0.02);
    nh_.param<int>("cluster_min_size", cluster_min_size, 25);
    nh_.param<int>("cluster_max_size", cluster_max_size, 20000);
    nh_.param<float>("cluster_min_height", cluster_min_height, 0.011);
    nh_.param<float>("cluster_max_height", cluster_max_height, 0.09);
    nh_.param<float>("cluster_max_length", cluster_max_length, 0.25);
    nh_.param<float>("cluster_min_distance_to_polygon", cluster_min_distance_to_polygon, 0.04);
    scene_segmentation_->setClusterParams(cluster_tolerance, cluster_min_size,
            cluster_max_size, cluster_min_height, cluster_max_height,
            cluster_max_length, cluster_min_distance_to_polygon);
}

void EmptySpaceDetector::eventInCallback(const std_msgs::String::ConstPtr &msg)
{
}

void EmptySpaceDetector::pcCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if (add_to_octree_)
    {
        sensor_msgs::PointCloud2 msg_transformed;   
        if (!mpu::pointcloud::transformPointCloudMsg(tf_listener_, output_frame_, *msg, msg_transformed))
            return;
        
        PointCloud::Ptr input_pc(new PointCloud);
        pcl::fromROSMsg(msg_transformed, *input_pc);

        cloud_accumulation_->addCloud(input_pc);
        
        findEmptySpace();

        /* if (this->enable_debug_pc_pub) */
        /* { */
        /*     /1* publish debug pointcloud *1/ */
        /*     sensor_msgs::PointCloud2 output; */
        /*     pcl::toROSMsg(*pc_segmented, output); */
        /*     output.header.frame_id = this->output_frame; */
        /*     output.header.stamp = ros::Time::now(); */
        /*     this->pc_pub.publish(output); */
        /*     ROS_INFO("Publishing debug pointcloud"); */
        /* } */
    }
}

void EmptySpaceDetector::findEmptySpace()
{
    PointCloud::Ptr cloud_in(new PointCloud);
    cloud_accumulation_->getAccumulatedCloud(*cloud_in);

    std::vector<PointCloud::Ptr> clusters;
    std::vector<BoundingBox> boxes;
    pcl::ModelCoefficients::Ptr model_coefficients(new pcl::ModelCoefficients);
    double workspace_height;
    PointCloud::Ptr debug = scene_segmentation_->segmentScene(cloud_in, clusters,
                                                              boxes, model_coefficients,
                                                              workspace_height);
    std::cout << "workspace height " << workspace_height << std::endl;
    std::cout << "num of points " << debug->points.size() << std::endl;
    if (enable_debug_pc_pub_)
    {
        /* publish debug pointcloud */
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*debug, output);
        output.header.frame_id = output_frame_;
        output.header.stamp = ros::Time::now();
        pc_pub_.publish(output);
        ROS_INFO("Publishing debug pointcloud");
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "empty_space_detector");
    EmptySpaceDetector es_detector;
    ros::Rate loop_rate(10.0);
    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
