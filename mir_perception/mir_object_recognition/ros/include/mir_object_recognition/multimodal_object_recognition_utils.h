/*
 * Copyright 2019 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil
 *
 */
#ifndef MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_UTILS_H
#define MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_UTILS_H

#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>

#include <mir_perception_utils/aliases.h>
#include <mas_perception_msgs/Object.h>

class MultimodalObjectRecognitionUtils
{
  public:
    /** Constructor
     * */
    MultimodalObjectRecognitionUtils();
    virtual ~MultimodalObjectRecognitionUtils();

  public:
    /** \brief Make adjustment for AXIS and Bolt (M20_100) pose. Adjust it so that the pose
     *     is around to the center of mass (the tip).
     * \param[in] object
     * 
    */
    void adjustAxisBoltPose(mas_perception_msgs::Object &object);

    /** \brief Adjust container pose by finding the largest cluster to find the center
     * \param[in] object.views[0].point_cloud
     * \param[in] the height adjustment of the container, default 10cm
    */
    void adjustContainerPose(mas_perception_msgs::Object &container_object, float rgb_container_height=0.1);
};
#endif  // MIR_OBJECT_RECOGNITION_MULTIMODAL_OBJECT_RECOGNITION_UTILS_H
