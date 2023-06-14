/*
 * Copyright 2019 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil
 *
 */
#ifndef MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_ROS_H
#define MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_ROS_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/RegionOfInterest.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <dynamic_reconfigure/server.h>

#include <mas_perception_msgs/ObjectList.h>

#include <mir_object_recognition/SceneSegmentationConfig.h>
#include <mir_object_recognition/multimodal_object_recognition_utils.h>
#include <mir_object_segmentation/scene_segmentation_ros.h>
#include <mir_perception_utils/object_utils_ros.h>
#include <mir_perception_utils/pointcloud_utils_ros.h>

/** \brief This node subscribes to pointcloud and image_raw topics synchronously.
 * Inputs:
 * ~event_in:
 *      - e_start:  - starts subscribing to pointcloud and image topics simultaneously,
 *              - segments pointcloud, recognize the table top clusters, estimate pose and workspace height
 *              - detects rgb object, find 3D ROI, estimate pose
 *              - adjusts object pose and publish them
 *      - e_stop:   - stop subscribing to the input topics and clear accumulated pointcloud
 *      - e_data_collection -  start dataset collection mode (save dir is defined in launch file).
 *                   If enable, this node will not do any recognition.
 * Outputs:
 * ~event_out:
 *      - e_done:   - done recognizing pointcloud and image, done pose estimation and done publishing object_list
 *      - e_stopped:  - done unsubscribing, done clearing accumulated point clouds
 *      - e_data_collection - data collection mode started
 * 
 * Topics output: rgb, pointcloud and multimodal object_lists, workspace_height.
 * Topics output for visualization: pose array, bounding boxes and labels for both rgb and point cloud
 * 
 * \author Mohammad Wasil
**/

namespace mpu = mir_perception_utils;
using mpu::visualization::BoundingBoxVisualizer;
using mpu::visualization::ClusteredPointCloudVisualizer;
using mpu::visualization::LabelVisualizer;
using mpu::visualization::Color;

struct Object
{
  std::string name;
  std::string shape;
  std::string color;
};

typedef std::vector<Object> ObjectInfo;

class MultimodalObjectRecognitionROS
{
  public:
    /** \brief Constructor
     * \param[in] NodeHandle */
    MultimodalObjectRecognitionROS(ros::NodeHandle nh);

    /** \brief Destructor */
    virtual ~MultimodalObjectRecognitionROS();
    /** \brief If pointcloud and image messages has been received,
     * multimodal object recognition will be called.
     * This function can be called once or periodically.
     */
    void update();

  private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_event_in_;
    ros::Publisher pub_event_out_;
    ros::Subscriber sub_cloud_;

    boost::shared_ptr<tf::TransformListener> tf_listener_;
    
    dynamic_reconfigure::Server<mir_object_recognition::SceneSegmentationConfig> server_;

    // Publisher for clouds and images recognizer
    ros::Publisher pub_cloud_to_recognizer_;
    ros::Publisher pub_image_to_recognizer_;
    // Subscriber for clouds and images recognizer
    ros::Subscriber sub_recognized_image_list_;
    ros::Subscriber sub_recognized_cloud_list_;
    // Publisher object list
    ros::Publisher pub_object_list_;
    ros::Publisher pub_workspace_height_;
    // Publisher pose array (debug_mode only)
    ros::Publisher pub_pc_object_pose_array_;
    ros::Publisher pub_rgb_object_pose_array_;
    // Publisher debug
    ros::Publisher pub_debug_cloud_plane_;
    ros::Publisher pub_filtered_rgb_cloud_plane_;
    std::string horizontal_object_list[9];

    // Synchronize callback for image and pointcloud
    message_filters::Subscriber<sensor_msgs::Image> *image_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> msgSyncPolicy;
    message_filters::Synchronizer<msgSyncPolicy> *msg_sync_;
    void synchronizeCallback(const sensor_msgs::ImageConstPtr &image, 
                 const sensor_msgs::PointCloud2ConstPtr &cloud);

    // Recognize Clouds and Image callback
    void recognizedImageCallback(const mas_perception_msgs::ObjectList &msg);
    void recognizedCloudCallback(const mas_perception_msgs::ObjectList &msg);
  
  protected:
    typedef std::shared_ptr<SceneSegmentationROS> SceneSegmentationROSSPtr;
    SceneSegmentationROSSPtr scene_segmentation_ros_;
    typedef std::shared_ptr<MultimodalObjectRecognitionUtils> MultimodalObjectRecognitionUtilsSPtr;
    MultimodalObjectRecognitionUtilsSPtr mm_object_recognition_utils_;

    // Used to store pointcloud and image received from callback
    sensor_msgs::PointCloud2ConstPtr pointcloud_msg_;
    sensor_msgs::ImageConstPtr image_msg_;
    PointCloud::Ptr cloud_;

    // Flags for pointcloud and image subscription
    int pointcloud_msg_received_count_;
    int image_msg_received_count_;
    
    // rgb_object_id used to differentiate 2D and 3D objects
    int rgb_object_id_;
    
    // Flags for object recognition
    bool received_recognized_image_list_flag_;
    bool received_recognized_cloud_list_flag_;

    //Recognized image list
    mas_perception_msgs::ObjectList recognized_image_list_;
    mas_perception_msgs::ObjectList recognized_cloud_list_;

    // Enable recognizer
    bool enable_rgb_recognizer_;
    bool enable_pc_recognizer_ ;

    // Visualization
    BoundingBoxVisualizer bounding_box_visualizer_pc_;
    ClusteredPointCloudVisualizer cluster_visualizer_rgb_;
    ClusteredPointCloudVisualizer cluster_visualizer_pc_;
    ClusteredPointCloudVisualizer cluster_visualizer_filtered_rgb_;
    LabelVisualizer label_visualizer_rgb_;
    LabelVisualizer label_visualizer_pc_;

    // Parameters
    bool debug_mode_;
    std::string target_frame_id_;
    std::string pointcloud_source_frame_id_;
    std::set<std::string> round_objects_;
    ObjectInfo object_info_;
    std::string object_info_path_;

    // Dynamic parameter
    double object_height_above_workspace_;
    double container_height_;
    int rgb_roi_adjustment_;
    int rgb_bbox_min_diag_;
    int rgb_bbox_max_diag_;
    double rgb_cluster_filter_limit_min_;
    double rgb_cluster_filter_limit_max_;
    bool enable_roi_;
    double roi_base_link_to_laser_distance_;
    double roi_max_object_pose_x_to_base_link_;
    double roi_min_bbox_z_;
    bool rgb_cluster_remove_outliers_;

    //cluster
    bool center_cluster_;
    bool pad_cluster_;
    unsigned int padded_cluster_size_;

     // logdir for saving debug image
    std::string logdir_;
    bool data_collection_;

  private:
    /** \brief Event in callback
     * */
    void eventCallback(const std_msgs::String::ConstPtr &msg);

    /** \brief Dynamic reconfigure callback
     * */
    void configCallback(mir_object_recognition::SceneSegmentationConfig &config, uint32_t level);
    
    /** \brief Transform pointcloud to the given frame id ("base_link" by default)
     * \param[in] PointCloud2 input
    */
    void preprocessPointCloud(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

    /** \brief Add cloud accumulation, segment accumulated pointcloud, find the plane, 
     *     clusters table top objects, find object heights.
     * \param[out] 3D object list with unknown label
     * \param[out] Table top pointcloud clusters
     **/
    void segmentPointCloud(mas_perception_msgs::ObjectList &object_list, 
                 std::vector<PointCloud::Ptr> &clusters,
                 std::vector<BoundingBox> boxes);

    /** \brief Recognize 2D and 3D objects, estimate their pose, filter them, and publish the object_list*/
    void recognizeCloudAndImage();

    /** \brief Adjust object pose, make it flat, adjust container, axis and bolt poses.
     * \param[in] Object_list.pose, .name,
     * 
     **/
    void adjustObjectPose(mas_perception_msgs::ObjectList &object_list);

    /** \brief Publish object_list to object_list merger 
     * \param[in] Object list to publish
     **/
    void publishObjectList(mas_perception_msgs::ObjectList &object_list);

    /** \brief Publish debug info such as bbox, poses, labels for both 2D and 3D objects.
     * \param[in] combined object list
     * \param[in] 3D pointcloud cluster from 3D object segmentation
     * \param[in] 3D pointcloud cluster from 2D bounding box proposal
     **/
    void publishDebug(mas_perception_msgs::ObjectList &combined_object_list,
              std::vector<PointCloud::Ptr> &clusters_3d,
              std::vector<PointCloud::Ptr> &clusters_2d,
              std::vector<PointCloud::Ptr> &filtered_clusters_2d);

    /** \brief Load qualitative object info
     * \param[in] Path to the xml object file
     * */ 
    void loadObjectInfo(const std::string &filename);        
};

#endif  // MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_ROS_H
