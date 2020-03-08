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
    std::string passthrough_filter_field_name;
    float passthrough_filter_limit_min, passthrough_filter_limit_max;
    bool enable_passthrough_filter;
    nh_.param<bool>("enable_passthrough_filter", enable_passthrough_filter, false);
    nh_.param<std::string>("passthrough_filter_field_name", passthrough_filter_field_name, "x");
    nh_.param<float>("passthrough_filter_limit_min", passthrough_filter_limit_min, 0.0);
    nh_.param<float>("passthrough_filter_limit_max", passthrough_filter_limit_max, 0.8);
    scene_segmentation_->setPassthroughParams(enable_passthrough_filter,
                                              passthrough_filter_field_name,
                                              passthrough_filter_limit_min,
                                              passthrough_filter_limit_max);

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
        
        add_to_octree_ = false;
        PointCloud::Ptr plane(new PointCloud);
        bool plane_found = this->findPlane(plane);
        if (!plane_found)
        {
            return;
        }

        this->findEmptySpacesOnPlane(plane);

        /* if (enable_debug_pc_pub_) */
        /* { */
        /*     /1* publish debug pointcloud *1/ */
        /*     sensor_msgs::PointCloud2 output; */
        /*     pcl::toROSMsg(*plane, output); */
        /*     output.header.frame_id = output_frame_; */
        /*     output.header.stamp = ros::Time::now(); */
        /*     pc_pub_.publish(output); */
        /*     ROS_INFO("Publishing debug pointcloud"); */
        /* } */
    }
}

void EmptySpaceDetector::findEmptySpacesOnPlane(const PointCloud::Ptr &plane)
{
    PointCloud::Ptr new_plane(new PointCloud);
    pcl::copyPointCloud(*plane, *new_plane);

    std::cout << new_plane->width << std::endl;
    int random_index = 500;
    if (random_index < new_plane->width)
    {
        kdtree_.setInputCloud (new_plane);

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        double radius = 0.05;

        if ( kdtree_.radiusSearch (random_index, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
        {
            std::cout << "num of points within that radius " << pointIdxRadiusSearch.size() << std::endl;
            for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
            {
                std::cout << "    "  <<   new_plane->points[ pointIdxRadiusSearch[i] ].x 
                    << " " << new_plane->points[ pointIdxRadiusSearch[i] ].y 
                    << " " << new_plane->points[ pointIdxRadiusSearch[i] ].z << std::endl;
                new_plane->points[pointIdxRadiusSearch[i]].z += 0.1; 
            }
        }
    }
    if (enable_debug_pc_pub_)
    {
        /* publish debug pointcloud */
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*new_plane, output);
        output.header.frame_id = output_frame_;
        output.header.stamp = ros::Time::now();
        pc_pub_.publish(output);
        ROS_INFO("Publishing debug pointcloud");
    }
}

bool EmptySpaceDetector::findPlane(PointCloud::Ptr plane)
{
    PointCloud::Ptr cloud_in(new PointCloud);
    cloud_accumulation_->getAccumulatedCloud(*cloud_in);

    PointCloud::Ptr hull(new PointCloud);
    pcl::ModelCoefficients::Ptr model_coefficients(new pcl::ModelCoefficients);
    double workspace_height;

    PointCloud::Ptr debug = scene_segmentation_->findPlane(cloud_in, hull, plane,
                                                           model_coefficients,
                                                           workspace_height);
    /* std::cout << "workspace height" << workspace_height << std::endl; */
    /* std::cout << "num of point in plane" << plane->points.size() << std::endl; */
    bool success = plane->points.size() > 0;
    return success;
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
