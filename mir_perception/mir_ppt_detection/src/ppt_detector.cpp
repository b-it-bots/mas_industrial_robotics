#include <mir_ppt_detection/ppt_detector.h>

PPTDetector::PPTDetector():
    nh_("~")
{
    // Create a ROS subscriber for the input point cloud
    // pc_sub_ = nh_.subscribe<PointCloud> ("points", 1, &PPTDetector::cloud_cb, this);
    float planar_projection_thresh;
    float target_pose_z_pos_;
    nh_.param<float>("planar_projection_thresh", planar_projection_thresh, 0.03);
    nh_.param<float>("target_pose_z_pos", target_pose_z_pos_, 0.035);
    event_in_sub_ = nh_.subscribe("event_in", 1, &PPTDetector::eventInCallback, this);
    dynamic_reconfigure::Server<mir_ppt_detection::PPTDetectionConfig>::CallbackType f =
              boost::bind(&PPTDetector::configCallback, this, _1, _2);
    server_.setCallback(f);
    // Create a ROS publisher for the output point cloud
    cloud_pub0 = nh_.advertise<sensor_msgs::PointCloud2> ("cloud_non_planar", 1);
    cloud_pub1 = nh_.advertise<sensor_msgs::PointCloud2> ("cloud_planar", 1);
    cloud_pub2 = nh_.advertise<sensor_msgs::PointCloud2> ("cloud_cavity_clusters", 1);
    cavity_pub = nh_.advertise<mir_ppt_detection::Cavities> ("cavities", 1);

    cavity_msg_pub_ = nh_.advertise<mas_perception_msgs::Cavity>("output_cavity", 10);
    debug_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("output_debug_pose", 1);
    event_out_pub_ = nh_.advertise<std_msgs::String>("event_out", 1);

    // read ros param
    bool success = readObjectShapeParams();
    if ( !success )
    {
        ROS_FATAL("Failed to read object_shape_learned_params_file.");
        ros::shutdown();
    }

    nh_.param<std::string>("target_frame", target_frame_, "base_link");
    nh_.param<std::string>("source_frame", source_frame_, "tower_cam3d_front_camera_color_optical_frame");
    nh_.param<bool>("debug_pub", debug_pub_, true);
}

void PPTDetector::configCallback(mir_ppt_detection::PPTDetectionConfig &config, uint32_t level)
{
  planar_projection_thresh = config.planar_projection_thresh;
  target_pose_z_pos_ = config.target_pose_z_pos;
//   ROS_INFO_STREAM("planar_projection_thresh " << planar_projection_thresh);
}

\
bool PPTDetector::readObjectShapeParams()
{
    std::string object_shape_learned_params_file;
    nh_.param<std::string>("object_shape_learned_params_file", object_shape_learned_params_file, "");

    if ( object_shape_learned_params_file == "" )
    {
        ROS_ERROR_STREAM("Object shape learned params file not provided");
        return false;
    }

    YAML::Node object_shape_learned_params_yaml;
    try
    {
        object_shape_learned_params_yaml = YAML::LoadFile(object_shape_learned_params_file);
    }
    catch(YAML::BadFile)
    {
        ROS_ERROR_STREAM("YAML threw BadFile exception. Does the file exist?"
                         << std::endl << object_shape_learned_params_file );
        return false;
    }

    if ( !object_shape_learned_params_yaml.IsMap() )
    {
        ROS_ERROR_STREAM("Object shape learned params file does not have a correct format.");
        return false;
    }

    // std::cout << object_shape_learned_params_yaml << std::endl;
    for ( YAML::const_iterator it = object_shape_learned_params_yaml.begin();
            it != object_shape_learned_params_yaml.end(); ++it )
    {
        std::string obj_name = it->first.as<std::string>();
        if ( !it->second["mu"] || !it->second["cov"] )
        {
            ROS_ERROR_STREAM("Object shape learned params file contains object "
                             << obj_name << " with invalid info");
            return false;
        }
        std::vector<float> mu_vector = it->second["mu"].as<std::vector<float>>();
        std::vector<float> cov_vector = it->second["cov"].as<std::vector<float>>();
        if ( mu_vector.size() != 2 || cov_vector.size() != 4 )
        {
            ROS_ERROR_STREAM("Object shape learned params file contains object "
                             << obj_name << " with invalid size info");
            return false;
        }
        LearnedObjectParams learned_obj_params;
        learned_obj_params.mu(0, 0) = mu_vector[0];
        learned_obj_params.mu(1, 0) = mu_vector[1];
        // FIXME: remove magic number 16
        learned_obj_params.cov(0, 0) = 16 * cov_vector[0];
        learned_obj_params.cov(0, 1) = 16 * cov_vector[1];
        learned_obj_params.cov(1, 0) = 16 * cov_vector[2];
        learned_obj_params.cov(1, 1) = 16 * cov_vector[3];
        learned_obj_params_map_[obj_name] = learned_obj_params;
    }

    // for ( auto itr = learned_obj_params_map_.begin();
    //       itr != learned_obj_params_map_.end();
    //       itr ++ )
    // {
    //     std::cout << itr->first << std::endl;
    //     std::cout << "mu: " << itr->second.mu << std::endl;
    //     std::cout << "cov: " << itr->second.cov << std::endl;
    // }

    return true;
}

PointRGBA PPTDetector::get_point_rgba(const pcl::PointXYZRGB& pt_rgb){
    PointRGBA pt_rgba;
    pt_rgba.x = pt_rgb.x;
    pt_rgba.y = pt_rgb.y;
    pt_rgba.z = pt_rgb.z;
    pt_rgba.r = pt_rgb.r;
    pt_rgba.g = pt_rgb.g;
    pt_rgba.b = pt_rgb.b;
    return pt_rgba;
}

PointCloudRGBA::Ptr PPTDetector::get_point_cloud_rgba(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb){
    PointCloudRGBA::Ptr cloud_rgba(new PointCloudRGBA);
    for( size_t i = 0;  i < cloud_rgb->points.size(); i++){
        cloud_rgba->points.push_back(get_point_rgba(cloud_rgb->points[i]));
    }   
    return cloud_rgba; 
}

template<typename T>
void PPTDetector::downsample_organized_cloud(T cloud_in, PointCloud::Ptr cloud_downsampled, int scale) {
    cloud_downsampled->width = cloud_in->width / scale;
    cloud_downsampled->height = cloud_in->height / scale;
    cloud_downsampled->points.resize(cloud_downsampled->width * cloud_downsampled->height);
    for( size_t i = 0, ii = 0; i < cloud_downsampled->height; ii += scale, i++){
        for( size_t j = 0, jj = 0; j < cloud_downsampled->width; jj += scale, j++){
            cloud_downsampled->at(j, i) = cloud_in->at(jj, ii);
        }
	// if we find NaN's fill them with valid values by extrapolating from previous points (along width)
        for (size_t j = 0; j < cloud_downsampled->width; j++) {
            if (std::isnan(cloud_downsampled->at(j, i).x) and j > 2)
            {
                float diff_x = cloud_downsampled->at(j-1, i).x - cloud_downsampled->at(j-2, i).x;
                float diff_y = cloud_downsampled->at(j-1, i).y - cloud_downsampled->at(j-2, i).y;
                float current_x = cloud_downsampled->at(j-1, i).x + diff_x;
                float current_y = cloud_downsampled->at(j-1, i).y + diff_y;
                cloud_downsampled->at(j, i).x = current_x;
                cloud_downsampled->at(j, i).y = current_y;
                cloud_downsampled->at(j, i).z = cloud_downsampled->at(j - 1, i).z;
            }
        }
	// if we find NaN's fill them with valid values by extrapolating from the previous rows
        for (size_t j = 0; j < cloud_downsampled->width; j++) {
            if (std::isnan(cloud_downsampled->at(j, i).x) and i > 2)
            {
                float diff_x = cloud_downsampled->at(j, i-1).x - cloud_downsampled->at(j, i-2).x;
                float diff_y = cloud_downsampled->at(j, i-1).y - cloud_downsampled->at(j, i-2).y;
                float current_x = cloud_downsampled->at(j, i-1).x + diff_x;
                float current_y = cloud_downsampled->at(j, i-1).y + diff_y;
                cloud_downsampled->at(j, i).x = current_x;
                cloud_downsampled->at(j, i).y = current_y;
                cloud_downsampled->at(j, i).z = cloud_downsampled->at(j, i-1).z;
            }
        }
    }
}

bool PPTDetector::compute_dominant_plane_and_hull(PointCloud::Ptr cloud_in,
                                     pcl::ModelCoefficients::Ptr plane_coeffs,
                                     PointCloud::Ptr hull){
    //Estimate most dominant plane coefficients and inliers
    PointIndices::Ptr inliers (new PointIndices);
    pcl::SACSegmentation<PointT> sac_seg;
    sac_seg.setModelType (pcl::SACMODEL_PLANE);
    sac_seg.setMethodType (pcl::SAC_RANSAC);
    sac_seg.setOptimizeCoefficients (true);
    sac_seg.setDistanceThreshold (0.01);
    sac_seg.setInputCloud (cloud_in);
    sac_seg.segment (*inliers, *plane_coeffs);

    if (inliers->indices.size() > cloud_in->points.size() / 5) {
        //Project plane model inliers to plane
        PointCloud::Ptr cloud_plane(new PointCloud);
        pcl::ProjectInliers<PointT> project_inliers;
        project_inliers.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
        project_inliers.setInputCloud(cloud_in);
        project_inliers.setModelCoefficients(plane_coeffs);
        project_inliers.setIndices(inliers);
        project_inliers.setCopyAllData(false);
        project_inliers.filter(*cloud_plane);

        //Compute plane convex hull
        pcl::ConvexHull<PointT> convex_hull;
        convex_hull.setInputCloud(cloud_plane);
        convex_hull.reconstruct(*hull);
        hull->points.push_back(hull->at(0));
        return true;
    } else {
        return false;
    }
}

void PPTDetector::project_points_to_plane(PointCloud::Ptr cloud_in,
                             pcl::ModelCoefficients::Ptr plane_coeffs,
                             PointIndices::Ptr planar_indices, 
                             PointIndices::Ptr non_planar_indices,
                             PointCloudRGBA::Ptr cloud_projected){ 
    cloud_projected->width = cloud_in->width;
    cloud_projected->height = cloud_in->height;
    for( size_t row = 0;  row < cloud_in->height; row++){
        for( size_t col = 0; col < cloud_in->width; col++){
            PointT pt_in = cloud_in->at(col, row);

            float x_bar = (downsample_scale*col-cam_cx)/cam_fx;
            float y_bar = (downsample_scale*row-cam_cy)/cam_fy;        
            float z = -plane_coeffs->values[3] / (plane_coeffs->values[0]*x_bar + plane_coeffs->values[1]*y_bar + plane_coeffs->values[2]) + 5e-4f*rand()/(RAND_MAX);
            PointRGBA pt_projected = get_point_rgba(pt_in);
            pt_projected.x = x_bar*z;
            pt_projected.y = y_bar*z;
            pt_projected.z = z;

            float pt_in_dist = sqrt(pow(pt_in.x,2) + pow(pt_in.y,2) + pow(pt_in.z,2)); 
            float pt_proj_dist = sqrt(pow(pt_projected.x,2) + pow(pt_projected.y,2) + pow(pt_projected.z,2));
            if (fabs(pt_in_dist - pt_proj_dist) < planar_projection_thresh) {
                planar_indices->indices.push_back(col + row*cloud_in->width);      
                pt_projected.a = 0;
            } else if (std::isnan(pt_in_dist)){
		// Do nothing
            } else if (pt_in_dist > pt_proj_dist){
                non_planar_indices->indices.push_back(col + row*cloud_in->width);       
                pt_projected.a = 1;
            }
            cloud_projected->points.push_back(pt_projected);
        }
    }
}

void PPTDetector::extract_polygonal_prism_inliers(PointCloudRGBA::Ptr cloud_in,
                                     PointIndices::Ptr indices_in,
                                     PointCloud::Ptr cloud_hull,
                                     PointIndices::Ptr hull_inlier_indices){
    pcl::ExtractPolygonalPrismData<PointRGBA> epp;
    PointCloudRGBA::Ptr cloud_hull_rgba = get_point_cloud_rgba(cloud_hull);
    epp.setInputPlanarHull(cloud_hull_rgba);
    epp.setInputCloud(cloud_in);
    epp.setIndices(indices_in);
    double z_min = -planar_projection_thresh;
    double z_max = planar_projection_thresh;
    epp.setHeightLimits(z_min, z_max);
    epp.segment(*hull_inlier_indices);
}

bool PPTDetector::customRegionGrowing1 (const PointRGBA& point_a, const PointRGBA& point_b,
                           float squared_distance){
    float thresh = 5;
    if (fabs(point_a.r-point_b.r) < thresh && fabs(point_a.g-point_b.g) < thresh && fabs(point_a.b-point_b.b) < thresh){
        return true;
    }
    return false;
}

bool PPTDetector::customRegionGrowing2 (const PointRGBA& point_a, const PointRGBA& point_b,
                           float squared_distance){
    if (point_a.a == 1 && point_b.a == 1){
        return true; 
    } else {
        float thresh = 5;
        if (fabs(point_a.r-point_b.r) < thresh && fabs(point_a.g-point_b.g) < thresh && fabs(point_a.b-point_b.b) < thresh){
            return true;
        } 
    }
    return false;
}

float PPTDetector::get_non_planar_pt_frac(PointCloudRGBA::Ptr cloud_in){
    int non_planar_pt_cnt = 0;
    for( size_t i= 0;  i < cloud_in->points.size(); i++){
        if (cloud_in->points[i].a == 1) {non_planar_pt_cnt++;}
    }
    return (float)non_planar_pt_cnt/cloud_in->points.size();
}

void PPTDetector::compute_cavity_clusters(PointCloudRGBA::Ptr cloud_in,
                             PointIndices::Ptr planar_idx,
                             PointIndices::Ptr non_planar_idx,
                             pcl::IndicesClustersPtr clusters){
    pcl::ConditionalEuclideanClustering<PointRGBA> cec;
    cec.setInputCloud (cloud_in);

    pcl::IndicesClustersPtr planar_cavity_candidate_idx_clusters (new pcl::IndicesClusters);
    cec.setIndices (planar_idx);
    cec.setClusterTolerance (0.005);
    cec.setMinClusterSize (1);
    cec.setMaxClusterSize (cloud_in->points.size() / 50);
    cec.setConditionFunction (&PPTDetector::customRegionGrowing1);
    cec.segment (*planar_cavity_candidate_idx_clusters);

    pcl::PointIndices::Ptr cavity_candidate_indices (new pcl::PointIndices (*non_planar_idx));
    for (std::vector<PointIndices>::const_iterator cluster_it = planar_cavity_candidate_idx_clusters->begin (); 
            cluster_it != planar_cavity_candidate_idx_clusters->end (); ++cluster_it) {
        cavity_candidate_indices->indices.insert(cavity_candidate_indices->indices.end(),
                                                 cluster_it->indices.begin(),
                                                 cluster_it->indices.end());
    }

    cec.setIndices (cavity_candidate_indices);
    cec.setClusterTolerance (0.005);
    cec.setMinClusterSize (cloud_in->points.size() / 400);
    cec.setMaxClusterSize (cloud_in->points.size() / 10);
    cec.setConditionFunction (&PPTDetector::customRegionGrowing2);
    cec.segment (*clusters);
}

void PPTDetector::detectCavities(const PointCloud::ConstPtr& input,
                                 mir_ppt_detection::Cavities& cavities_msg,
                                 PointCloudRGBA::Ptr& non_planar_cloud,
                                 PointCloudRGBA::Ptr& planar_cloud,
                                 PointCloudRGBA::Ptr& cavity_cloud)
{
    //Downsample by downsample_scale
    PointCloud::Ptr cloud_downsampled(new PointCloud);
    downsample_organized_cloud(input, cloud_downsampled, downsample_scale);

    //Downsample by downsample_scale a second time
    PointCloud::Ptr cloud_downsampled_x2(new PointCloud);
    downsample_organized_cloud(cloud_downsampled, cloud_downsampled_x2, downsample_scale);

    //Estimate most dominant plane coefficients and hull
    pcl::ModelCoefficients::Ptr plane_coefficients (new pcl::ModelCoefficients);
    PointCloud::Ptr cloud_hull(new PointCloud);

    if (!compute_dominant_plane_and_hull(cloud_downsampled_x2, plane_coefficients, cloud_hull)) 
    {
        return;
    }
    dist_to_hull.setConvexHullPointsAndEdges(get_point_cloud_rgba(cloud_hull));   
    // std::cout << "plane segmentation time: " << ros::Time::now().toSec() - t.toSec() << std::endl; 

    PointIndices::Ptr planar_indices (new PointIndices);
    PointIndices::Ptr non_planar_indices (new PointIndices);
    PointCloudRGBA::Ptr cloud_projected(new PointCloudRGBA);
    project_points_to_plane(cloud_downsampled, plane_coefficients, planar_indices,
                            non_planar_indices, cloud_projected);

    PointIndices::Ptr planar_hull_inlier_indices (new PointIndices);
    extract_polygonal_prism_inliers(cloud_projected, planar_indices,
                                    cloud_hull, planar_hull_inlier_indices);
    PointIndices::Ptr non_planar_hull_inlier_indices (new PointIndices);
    extract_polygonal_prism_inliers(cloud_projected, non_planar_indices,
                                    cloud_hull, non_planar_hull_inlier_indices);

    pcl::IndicesClustersPtr cavity_clusters (new pcl::IndicesClusters);
    compute_cavity_clusters(cloud_projected, planar_hull_inlier_indices,
                            non_planar_hull_inlier_indices, cavity_clusters);
    std::cout << "Number of clusters: " << cavity_clusters->size() << std::endl;

    PointIndices::Ptr cavity_cluster_indices (new PointIndices);
    for (std::vector<PointIndices>::const_iterator cluster_it = cavity_clusters->begin ();
         cluster_it != cavity_clusters->end (); ++cluster_it)
    {
        cavity_cluster_indices->indices.insert(cavity_cluster_indices->indices.end(),
                                               cluster_it->indices.begin(),
                                               cluster_it->indices.end());
    }

    pcl::ExtractIndices<PointRGBA> extract (true);
    extract.setInputCloud (cloud_projected);
    PointCloudRGBA::Ptr cloud_cavity(new PointCloudRGBA);
    pcl::ConvexHull<PointRGBA> convex_hull;
    convex_hull.setComputeAreaVolume(true);
    PointCloudRGBA::Ptr cavity_hull(new PointCloudRGBA);
    for (std::vector<pcl::PointIndices>::const_iterator cluster_it = cavity_clusters->begin ();
            cluster_it != cavity_clusters->end (); ++cluster_it)
    {
        PointIndices::Ptr cavity_indices (new PointIndices (*cluster_it));
        extract.setIndices (cavity_indices);
        extract.filter (*cloud_cavity);
        if (get_non_planar_pt_frac(cloud_cavity) < 0.6)
        {
            continue;
        }
        convex_hull.setInputCloud(cloud_cavity);
        convex_hull.reconstruct(*cavity_hull);
        if (convex_hull.getTotalArea() < min_cavity_area ||
            dist_to_hull.computeMinDistanceToHull(cavity_hull) < 0.01) {
            continue;
        }
        // std::cerr << "Cavity cloud points added: " << cloud_cavity->points.size () << std::endl;
        PointCloudRGBA::Ptr cavity_cloud_filtered(new PointCloudRGBA);
        pcl::VoxelGrid<PointRGBA> sor;
        sor.setInputCloud (cloud_cavity);
        sor.setLeafSize (0.002f, 0.002f, 0.002f);
        sor.filter (*cavity_cloud_filtered);

        Eigen::Vector4f pcaCentroid;
        pcl::compute3DCentroid(*cavity_cloud_filtered, pcaCentroid);
        Eigen::Matrix3f covariance;
        computeCovarianceMatrixNormalized(*cavity_cloud_filtered,
                                          pcaCentroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(
                covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();      

        int first_principle_component_direction_signum;
        float first_principle_component_displacement;
        float second_principle_component_displacement;
        float displacement_metric = 0.0f;
        Eigen::Vector3f pt_to_centroid_vec, pcaCentroid_vec3f = pcaCentroid.head<3>();
        for (std::vector<PointRGBA, Eigen::aligned_allocator<PointRGBA> >::const_iterator cpt = cavity_cloud_filtered->points.begin ();
                cpt != cavity_cloud_filtered->points.end (); ++cpt){   
            pt_to_centroid_vec = cpt->getVector3fMap()-pcaCentroid_vec3f;
            first_principle_component_displacement = eigenVectorsPCA.col(2).dot(pt_to_centroid_vec);
            second_principle_component_displacement = eigenVectorsPCA.col(1).dot(pt_to_centroid_vec);
            displacement_metric += first_principle_component_displacement * pow(second_principle_component_displacement,2);
        }
        first_principle_component_direction_signum = displacement_metric/fabs(displacement_metric);
        eigenVectorsPCA.col(0) = first_principle_component_direction_signum * eigenVectorsPCA.col(2);
        // Ensure orientation z-axis (3rd principle component / eigenvector column) points towards camera
        if (eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1))[2] > 0){
            eigenVectorsPCA.col(1) = -eigenVectorsPCA.col(1);
        } 
        eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));


        mir_ppt_detection::Cavity cavity_msg;
        cavity_msg.cov_minor = eigen_solver.eigenvalues()[1];
        cavity_msg.cov_major = eigen_solver.eigenvalues()[2];
        geometry_msgs::Pose pose;
        Eigen::Quaternionf quaternionPCA(eigenVectorsPCA);
        pose.orientation.x = quaternionPCA.x();
        pose.orientation.y = quaternionPCA.y();
        pose.orientation.z = quaternionPCA.z();
        pose.orientation.w = quaternionPCA.w();
        pose.position.x = pcaCentroid[0];
        pose.position.y = pcaCentroid[1];
        pose.position.z = pcaCentroid[2];
        cavity_msg.pose = pose;
        cavities_msg.cavities.push_back(cavity_msg);
        // std::cout << "eigenval minor: " << eigen_solver.eigenvalues()[1]
        //           << "   eigenval major: " << eigen_solver.eigenvalues()[2]  << std::endl;      
    }

    extract.setIndices (non_planar_hull_inlier_indices);
    extract.filter (*non_planar_cloud);
    non_planar_cloud->width = non_planar_cloud->points.size ();
    non_planar_cloud->height = 1;
    non_planar_cloud->is_dense = true;

    extract.setIndices (planar_hull_inlier_indices);
    extract.filter (*planar_cloud);
    planar_cloud->width = planar_cloud->points.size ();
    planar_cloud->height = 1;
    planar_cloud->is_dense = true;

    extract.setIndices (cavity_cluster_indices);
    extract.filter (*cavity_cloud);
    cavity_cloud->width = cavity_cloud->points.size ();
    cavity_cloud->height = 1;
    cavity_cloud->is_dense = true;
}

float PPTDetector::get_mahalanobis_distance(Eigen::Vector2f x, Eigen::Vector2f mu, Eigen::Matrix2f cov)
{
    Eigen::Vector2f x_minus_mu = x-mu;
    float answer = (cov.inverse() * x_minus_mu).transpose() * x_minus_mu;
    return sqrt(answer);
}

std::string PPTDetector::predictCavityName(const mir_ppt_detection::Cavity& cavity)
{
    std::string obj_name = "unknown";
    if ( cavity.cov_minor < 2.5e-5f )
    {
        std::cout << "cov minor is too small" << std::endl;
        return obj_name;
    }

    float min_mahalanobis_distance = 1000.0f;

    Eigen::Vector2f obj_x;
    obj_x(0, 0) = cavity.cov_minor;
    obj_x(1, 0) = cavity.cov_major;

    for ( auto itr = learned_obj_params_map_.begin();
          itr != learned_obj_params_map_.end();
          itr ++ )
    {
        float mahalanobis_distance = get_mahalanobis_distance(obj_x, itr->second.mu,
                                                              itr->second.cov);
        // FIXME: remove magic number 2.0
        if ( mahalanobis_distance < 2.0f && mahalanobis_distance < min_mahalanobis_distance )
        {
            obj_name = itr->first;
            min_mahalanobis_distance = mahalanobis_distance;
        }
    }
    return obj_name;
}


void PPTDetector::publish_cavity_msg(const mir_ppt_detection::Cavities& cavities)
{
    geometry_msgs::PoseArray pose_array_msg;
    pose_array_msg.header.stamp = ros::Time::now();
    pose_array_msg.header.frame_id = target_frame_;


    for ( size_t i = 0; i < cavities.cavities.size(); i ++ )
    {
        std::string cavity_name = predictCavityName(cavities.cavities[i]);
        std::cout << "i:" << i << " winner cavity:" << cavity_name << std::endl;
        
        if ( cavity_name == "unknown" )
        {
            continue;
        }

        geometry_msgs::PoseStamped pose_in_source_frame;
        pose_in_source_frame.pose = cavities.cavities[i].pose;
        pose_in_source_frame.header.frame_id = source_frame_;
        pose_in_source_frame.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped pose_in_target_frame;
        try
        {
            listener_.waitForTransform(target_frame_, source_frame_, pose_in_source_frame.header.stamp, ros::Duration(3.0));
            listener_.transformPose(target_frame_, pose_in_source_frame, pose_in_target_frame);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }
        pose_in_target_frame.pose.position.z = target_pose_z_pos_; //TODO: do not hardcode this; use workspace height + object_height_above_workspace

        mas_perception_msgs::Cavity cavity;

        cavity.pose = pose_in_target_frame;
        //ROS_INFO_STREAM("pose fream id in for loop " <<pose_array.header.frame_id);
        //std::cout<<" inside for loop frame id "<<cavity.pose.header.frame_id<<std::endl;
        cavity.name = cavity_name;
        cavity_msg_pub_.publish(cavity);

        pose_array_msg.poses.push_back(pose_in_target_frame.pose);
    }
    debug_pose_pub_.publish(pose_array_msg);
}

void PPTDetector::cloud_cb (const PointCloud::ConstPtr& input)
{
    mir_ppt_detection::Cavities cavities;
    PointCloudRGBA::Ptr non_planar_cloud(new PointCloudRGBA);
    PointCloudRGBA::Ptr planar_cloud(new PointCloudRGBA);
    PointCloudRGBA::Ptr cavity_cloud(new PointCloudRGBA);

    detectCavities(input, cavities, non_planar_cloud, planar_cloud, cavity_cloud);

    publish_cavity_msg(cavities);

    cavity_pub.publish(cavities);

    if ( debug_pub_ )
    {
        sensor_msgs::PointCloud2 output;

        pcl::toROSMsg(*non_planar_cloud, output);
        output.header.frame_id = input->header.frame_id ;
        output.header.stamp = ros::Time::now();
        cloud_pub0.publish (output);

        pcl::toROSMsg(*planar_cloud, output);
        output.header.frame_id = input->header.frame_id ;
        output.header.stamp = ros::Time::now();
        cloud_pub1.publish (output);

        pcl::toROSMsg(*cavity_cloud, output);
        output.header.frame_id = input->header.frame_id ;
        output.header.stamp = ros::Time::now();
        cloud_pub2.publish (output);
    }
    pc_sub_.shutdown();

    std_msgs::String output_msg;
    output_msg.data = std::string("e_done");
    event_out_pub_.publish(output_msg);
}

void PPTDetector::eventInCallback(const std_msgs::String &msg)
{
    if (msg.data == "e_trigger")
    {
        pc_sub_ = nh_.subscribe<PointCloud> ("points", 1, &PPTDetector::cloud_cb, this);
        ROS_INFO("Subscribed to pointcloud");
    }
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "ppt_3d_detector");
    PPTDetector ppt_detector;
    // Spin
    ros::spin ();
}
