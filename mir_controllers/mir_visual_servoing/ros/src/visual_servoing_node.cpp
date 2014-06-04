/**
 * This is the main source code file for running the Visual Servoing application. This application 
 * is dependent on the Robot Operating System (ROS) [www.ros.org], and will not run unless the
 * required ROS components are installed. You can do this by running the "rosdep install" command
 * from the base folder.
 *
 * This file is provided as is without any form or warranty.
 *
 * Author: Matthew Roscoe (mat.roscoe@unb.ca)
 */

// ROS
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>


// OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/JointState.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

// ARM STUFF
//#include <kdl/kdl.hpp>
//#include <kdl/chainiksolvervel_wdls.hpp>

#include <mir_controller_msgs/StartVisualServoing.h>
#include <mir_controller_msgs/VisualServoingState.h>

#include <moveit_msgs/JointLimits.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointPositions.h>

#include "mir_visual_servoing/visual_servoing_2d.h"

namespace enc = sensor_msgs::image_encodings;

/**
 * This is the ROS Node for the visual servoing application. It will get all of the ROS dependent
 * attributes and determine which library should be run 2D or 3D visual servoing.
 */
class VisualServoing 
{
public:
	/**
	 * This is the constructor for the visual seroving application. It will start the advertising of
	 * the visual seroving service so that the process can be started and stopped on command. If you
	 * want to start the visual servoing you need to run the do_visual_servoing service hook.
	 */
	VisualServoing( ): m_image_transporter( m_node_handler )
	{
		ros::NodeHandle temp( "~" );

		SetupYoubotArm();

		m_visual_servoing = new VisualServoing2D( false, 0, m_arm_joint_names );
 
		m_dynamic_reconfigre_subscriber.setCallback(boost::bind( &VisualServoing::dynamic_reconfig_callback, this, _1, _2 ) );

		// Service commands to allow this node to be started and stopped externally
		service_do_visual_serv = m_node_handler.advertiseService( "do_visual_servoing", &VisualServoing::do_visual_servoing, this );
		ROS_INFO( "Advertised 'start_visual_servoing' service for visual_servoing" );
		ROS_INFO( "Visual servoing node initialized." );
	}

	/**
	 * Standard destructor.
	 */
	~VisualServoing()
	{
	}

	/**
	 * This is the service hook for visual servoing. If you want to run the acutal visual servoing
	 * you wll need to call the "do_visual_servoing" service call through ROS.
	 */
	bool do_visual_servoing( mir_controller_msgs::StartVisualServoing::Request &req,
							 mir_controller_msgs::StartVisualServoing::Response &res )
	{
		m_is_visual_servoing_completed = 0;

		//  Incoming message from usbs_cam. This must be running in order for this ROS node to run.
		m_image_subscriber = m_image_transporter.subscribe( "/arm_cam/image_raw", 1, &VisualServoing::imageCallback, this );

		// get joint states and store them to a variable and go through them (arm_link_5) and check to see if the current state is
		// to close to the min or max value.
		m_sub_joint_states = m_node_handler.subscribe( "/joint_states", 1, &VisualServoing::jointstateCallback, this );

		// Velocity control for the YouBot base.
		base_velocities_publisher = m_node_handler.advertise<geometry_msgs::Twist>( "/cmd_vel_safe", 1 );

		m_visual_servoing->CreatePublishers( 1 );

		ros::Time start_time = ros::Time::now();

		ROS_INFO("VisualServoing: Starting Blob Detection");

		while( ( m_is_visual_servoing_completed == 0 ) && ros::ok() && ( (ros::Time::now() - start_time).toSec() < m_visual_servoing_timeout ) )
		{
			ros::spinOnce();
		}

		/**
		 * TODO: set visual servoing output to an enumeration.
		 */
		if( m_is_visual_servoing_completed == 2 )
		{
			ShutDown();
			ROS_ERROR( "Visual servoing failure due to lost object" );
			res.return_value.error_code = mir_controller_msgs::VisualServoingState::LOST_OBJ;
			return true;
		}
		else if( m_is_visual_servoing_completed == 3 )
		{
			ShutDown();
			ROS_ERROR( "Visual servoing failure due to general unrecoverable error" );
			res.return_value.error_code = mir_controller_msgs::VisualServoingState::FAILED;
			return true;
		}

		if( (ros::Time::now() - start_time).toSec() < m_visual_servoing_timeout )
		{
			ShutDown();
			ROS_INFO( "Visual Servoing Sucessful." );
			res.return_value.error_code = mir_controller_msgs::VisualServoingState::SUCCESS;
			return true;
		}
		else
		{
			ShutDown();
			ROS_ERROR( "Visual Servoing Failure due to Timeout" );
			res.return_value.error_code = mir_controller_msgs::VisualServoingState::TIMEOUT;
			return true;
		}

		return false;
	}

	/**
	 * This is the service call that is used to stop the visual servoing from running. It will only
	 * turn off the subscribers and publishers but keep libraries loaded if they are required later
	 * on. In order to reduce memory footprint we also close any currently open HighGUI windows.
	 */
	bool stop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
	{
		ShutDown();
		ROS_INFO( "Blob Detection Disabled" );
		return true;
	}

private:
  /**
   * This function is responsible for querying the parameter server that is currently running
   * for any robotic arm parameters that relate directly to the KUKA YouBot.
   */
  void SetupYoubotArm()
  {
	  XmlRpc::XmlRpcValue parameter_list;
	  m_node_handler.getParam("/arm_1/arm_controller/joints", parameter_list);
	  ROS_ASSERT(parameter_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

	  for (int32_t i = 0; i < parameter_list.size(); ++i)
	  {
		ROS_ASSERT(parameter_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
		m_arm_joint_names.push_back(static_cast<std::string>(parameter_list[i]));
	  }

	  //read joint limits
	  for(unsigned int i=0; i < m_arm_joint_names.size(); ++i)
	  {
		moveit_msgs::JointLimits joint_limits;
		joint_limits.joint_name = m_arm_joint_names[i];
		m_node_handler.getParam("/arm_1/arm_controller/limits/" + m_arm_joint_names[i] + "/min", joint_limits.min_position);
		m_node_handler.getParam("/arm_1/arm_controller/limits/" + m_arm_joint_names[i] + "/max", joint_limits.max_position);
		m_upper_joint_limits.push_back( joint_limits.max_position );
		m_lower_joint_limits.push_back( joint_limits.min_position );
	  }
  }

  /**
   * This function is responsible for calling the libraries that will perform the visual servoing
   * on the image that is coming in from either the 2D or 3D camera depending on which sensors are
   * currently available to the user.
   */
  void imageCallback( const sensor_msgs::ImageConstPtr& image_message )
  	{
  		IplImage *cv_image = NULL;

  		/**
  		cv_bridge::CvImagePtr cv_ptr;

		try
		{
			//cv_ptr = cv_bridge::toCvCopy( image_message, enc::BGR8 );
			cv_image = sensor_msgs::CvBridge::imgMsgToCv( image_message, enc::BGR8 ); 
		}
		catch( cv_bridge::Exception& e )
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
		}
		//cv_image = cv_ptr->image;

		*/
		
  		try
  		{
            cv_img_tmp =  cv_bridge::toCvCopy(image_message, "bgr8");
            ipl_img_tmp = cv_img_tmp->image;
    
  			cv_image = &ipl_img_tmp;
  		}
  		catch( cv_bridge::Exception& e )
  		{
  			ROS_ERROR( "Could not convert from '%s' to 'bgr8'.", image_message->encoding.c_str() );
  		}

 		m_is_visual_servoing_completed = m_visual_servoing->VisualServoing( cv_image );
  	}

  /**
   * This function is a call back that updates the current positions of the
   */
  void jointstateCallback( sensor_msgs::JointStateConstPtr joints )
  {
		for (unsigned i = 0; i < joints->position.size(); i++)
		{
			if( i == 4 )
			{
				ROS_DEBUG_STREAM( "Joint Name: " << joints->name[i].c_str() );
				ROS_DEBUG_STREAM( "Updated Gripper Position: " << joints->position[i] );
				m_visual_servoing->UpdateGripperPosition( joints->position[i] );
			}

		}
  }

  void dynamic_reconfig_callback(mir_visual_servoing::VisualServoingConfig &config, uint32_t level) 
  {
  		ROS_DEBUG_STREAM( "New Var: " << config.binary_threshold ); 
	    m_visual_servoing->UpdateDynamicVariables( config );    
	}

  /**
   * This function is used to determine if the joint limits of the robotic arm being used by the
   * robot are about to be exceeded. If they are near the "soft limit" (5% before the hard limit)
   * we will abort visual servoing as this is currently a behavior that we are unable to recover
   * from.
   *
   * TODO: instead of simply aborting on a soft limit being reached we should have a better reaction
   * to this type of occurrence.
   */
  /*
  bool checkLimits( KDL::JntArray joint_positions )
  {
	  const double joint_threshold = 0.05;

	  if( m_upper_joint_limits.size() < m_arm_chain.getNrOfJoints() ||
		  m_lower_joint_limits.size() < m_arm_chain.getNrOfJoints())
	  {
		  ROS_ERROR( "No Joint Limits Defined" );
		  return false;
	  }

	  for( unsigned int x = 0; x < m_arm_chain.getNrOfJoints(); x++)
	  {
		  double diff_up = fabs( (double)joint_positions.data(x) - m_upper_joint_limits[x] );
		  double diff_dn = fabs( (double)joint_positions.data(x) - m_lower_joint_limits[x] );

		  if( diff_up < joint_threshold || diff_dn < joint_threshold )
		  {
			  ROS_ERROR( "Joint soft-limit reached" );
			  return false;
		  }
	  }
	  ROS_INFO( "Joint states okay" );
	  return true;
  }
  */

  /**
   * This is a function that takes care of zeroing and shutting down any ROS publishers and
   * subscribers that the current class or any of its instantiated classes have created.
   */
  void ShutDown()
  {
  	  m_visual_servoing->DestroyPublishers();
	  // Zero all publishers
	  geometry_msgs::Twist zero_vel;
	  base_velocities_publisher.publish(zero_vel);

	  // shutdown any subscribers and publishers
	  m_image_subscriber.shutdown();
	  base_velocities_publisher.shutdown();
	  m_sub_joint_states.shutdown();

	  // Shut down any open windows.
	  cvDestroyAllWindows();
  }

protected:

  VisualServoing2D*									m_visual_servoing;

  /*
   * Standard ROS Publishers and Subscribers.
   */
  ros::NodeHandle 									m_node_handler; 
  image_transport::ImageTransport 					m_image_transporter;
  ros::Publisher								 	base_velocities_publisher;

  ros::Subscriber 									m_sub_joint_states;
  image_transport::Subscriber 						m_image_subscriber;

  dynamic_reconfigure::Server<mir_visual_servoing::VisualServoingConfig> m_dynamic_reconfigre_subscriber; 

  std::vector<std::string> 							m_arm_joint_names;
  std::vector<double> 								m_upper_joint_limits;
  std::vector<double> 								m_lower_joint_limits;
  //KDL::JntArray 									m_joint_positions;
  std::vector<bool> 								m_joint_positions_initialized;

  ros::ServiceServer 								service_do_visual_serv;

  cv_bridge::CvImagePtr cv_img_tmp;
  IplImage ipl_img_tmp;

  int 												m_is_visual_servoing_completed;

  const static int 									m_visual_servoing_timeout = 20;

  //KDL::Chain 										m_arm_chain;
};

/**
 * The main function for our visual servoing application. This will launch all of the components
 * required to perform the visual servoing.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_servoing");
  VisualServoing ic;
  ros::spin();
  return 0;
}
