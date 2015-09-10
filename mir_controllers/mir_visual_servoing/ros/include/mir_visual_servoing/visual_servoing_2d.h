#ifndef VISUALSERVOING2D_H_
#define VISUALSERVOING2D_H_

// ROS Includes
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <moveit_msgs/JointLimits.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointPositions.h>
#include <std_msgs/String.h>

#include <dynamic_reconfigure/server.h>
#include <mir_visual_servoing/VisualServoingConfig.h>

// OpenCV Includes
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

// cvBlobsLib Includes.
#include <cvblobs/BlobResult.h>

// BOOST
#include <boost/units/systems/si.hpp>
#include <boost/units/io.hpp>
#include <string>

/**
 *  This is the class that is responsible for performing visual servoing on
 *  2 Dimensional images typically provided in the RGB spectrum. We are
 *  able to also take in and deal with Black & White images.
 *
 * We take the largest blob that we can find in the image and then we try to
 * clean it up and then to track it through the environment. While tracking it
 * as the robot moves through the environment we compute required velocities
 * which will be used in order to account for the offsets in various coordinates.
 */
class VisualServoing2D
{
public:
    /**
     * This is the constructor for the 2D Visual Servoing system that sets up all of the
     * variables that the visual servoing system requires in order to function correctly.
     *
     * Modes:
     * 0 - Standard Visual Servoing
     * 1 - Conveyer Belt Visual Servoing
     */
    VisualServoing2D(bool debugging,
                     int mode,
                     std::vector<std::string> arm_joint_names);
    /**
     * Standard C++ destructor method.
     */
    virtual ~VisualServoing2D();

    /**
     * This function takes in a provided image and performs visual servoing on
     * the provided image. This
     *
     * Return Values:
     * 0 - Still running
     * 1 - Success
     * 2 - Blob lost timeout
     * 3 - General Failure
     */
    int VisualServoing(IplImage* input_image);

    /**
     * Setter function which allows the visual servoing application to pass down updated gripper
     * positions so that we can use them in future computations.
     */
    void UpdateGripperPosition(float new_position);

    void UpdateDynamicVariables(mir_visual_servoing::VisualServoingConfig config);

    /**
     * This function creates the publishers that will publish velcities for both the robotic base
     * through the GeometryTwist message as well as for the arm based on the arm model.
     *
     * Arm Models:
     * 0 - Unknown.
     * 1 - KUKA YouBot Arm
     * 2 - KUKA Lightweight Arm.
     */
    void CreatePublishers(int arm_model);

    /**
     * A general function that will properly zero and close and publishers or subscribers that are
     * being used by the VisualServoing2D mode of operation.
     */
    void DestroyPublishers();

private:

    /**
     * This function takes in a given x offset in a standard Cartesian coordinate
     * system. It will determine the direction to move the robot base to account
     * for the provided offset.
     */
    bool BaseAdjustmentX(double x_offset);

    /**
     * This function takes in a given y offset in a standard Cartesian coordinate
     * system. It will determine the direction to move the robot base to account
     * for the provided offset.
     */
    bool BaseAdjustmentY(double y_offset);

    /**
     * This function is designed to take the determined rotational offset that
     * has been previously determined and will use it to determine how the arm
     * should be moved in order to account for the difference.
     */
    bool ArmAdjustment(double orientation);

    /**
     * This function loads in the background image that will be subtracted from the incoming image
     * during the visual servoing to allow the system to better focus on non-standard parts of the
     * image.
     */
    IplImage* LoadBackgroundImage();

    /**
     * This function takes in an image and it crops it so that it is done according to the provided
     * scaling factor from (0.0 - 1.0).
     */
    IplImage* RegionOfInterest(IplImage* input_image, double scale);

    /**
     * This is a function that will take in an arbitrary number of images and create a display for
     * them that will serve as the Heads Up Display (HUD) of the Visual Servoing Application.
     * This is a modified version of the source code found here:
     * http://opencv.willowgarage.com/wiki/DisplayManyImages
     */
    void HUD(std::string title, int nArgs, ...);

protected:
    /*
     * Global Variable.
     */
    bool                                            g_debugging;
    int                                             g_operating_mode;

    /*
     * Modifiable Class Level Variables.
     */
    bool                                            m_first_pass;
    bool                                            m_done_base_x_adjustment;
    bool                                            m_done_base_y_adjustment;
    bool                                            m_done_arm_rot_adjustment;
    bool                                            m_blob_detection_completed;
    bool                                            m_head_left;
    bool                                            m_head_right;

    int                                             m_image_height;
    int                                             m_image_width;

    double                                          m_tracked_x;
    double                                          m_tracked_y;

    float                                           m_gripper_position;

    geometry_msgs::Twist                            m_youbot_base_velocities;
    brics_actuator::JointVelocities                 m_youbot_arm_velocities;
    std::vector<std::string>                        m_arm_joint_names;
    ros::Publisher                                  m_base_velocities_publisher;
    ros::Publisher                                  m_arm_velocities_publisher;
    ros::Publisher                                  m_pub_visual_servoing_status;
    ros::NodeHandle                                 m_node_handler;
    image_transport::ImageTransport m_image_transport;
    image_transport::Publisher  m_image_publisher;

    bool                                            m_is_blob_lost;
    ros::Time                                       m_time_when_lost;
    int                             m_lost_blob_timeout;

    IplImage*                                       m_background_image;

    mir_visual_servoing::VisualServoingConfig       m_dynamic_variables;

    /*
     * Constant values
     */
    int                             m_min_blob_area;
    int                             m_max_blob_area;
    int                                 m_verticle_offset;
    double                          m_x_velocity;
    double                          m_y_velocity;
    double                          m_rot_velocity;

    int                             m_x_target;
    int                                 m_x_threshold;

    int                             m_y_target;
    int                                 m_y_threshold;

    int                             m_rot_target;
    int                             m_rot_tolerance;

    ros::Publisher pub_cart_vel_;
    geometry_msgs::TwistStamped cart_zero_vel_;

};

#endif /* VISUALSERVOING2D_H_ */
