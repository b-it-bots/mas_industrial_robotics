/*
 * VisualServoing2D.cpp
 *
 *  Created on: Apr 22, 2013
 *      Author: badrobot
 */

#include "mir_visual_servoing/visual_servoing_2d.h"

VisualServoing2D::VisualServoing2D( bool debugging,
									int mode,
									std::vector<std::string> arm_joint_names ) : m_image_transport( m_node_handler )
{
  m_min_blob_area = 2000;
	m_max_blob_area = 90000;
	m_verticle_offset = 3;
	m_x_velocity = 0.012;
	m_y_velocity = 0.012;
	m_rot_velocity = 0.2;
	m_x_target = 0;
	m_x_threshold = 20;
	m_y_target = 0;
	m_y_threshold = 15;
	m_lost_blob_timeout = 20;

	m_rot_target = 90;
	m_rot_tolerance = 5;

	g_debugging = debugging;
	g_operating_mode = mode;

	m_first_pass = true;
	m_is_blob_lost = false;
	m_done_base_x_adjustment = true;
	m_done_base_y_adjustment = true;
	m_done_arm_rot_adjustment = true;
	m_blob_detection_completed = false;

	m_head_left = false;
	m_head_right = true;

	m_background_image = LoadBackgroundImage();

	m_arm_joint_names = arm_joint_names;

	if( g_debugging )
	{
		ROS_INFO( "Debugging Enabled" );
		if( g_operating_mode == 0 )
		{
			ROS_INFO( "Normal Visual Servoing" );
		}
		else if( g_operating_mode == 1 )
		{
			ROS_INFO( "Conveyer Belt Mode" );
		}
		else
		{
			ROS_ERROR( "BAD MODE" );
		}
	}
}

VisualServoing2D::~VisualServoing2D()
{
	cvDestroyWindow( "Original" );
	cvDestroyWindow( "Thresholding" );
	cvDestroyWindow( "Found Blobs" );
	cvDestroyWindow( "Background Image" );

	DestroyPublishers();
}

int
VisualServoing2D::VisualServoing( IplImage* input_image )
{
	/*  // Use this snippet to update new background image
	std::string path = ros::package::getPath("mir_visual_servoing") + "/common/data/background.png";
	cv::Mat m = cv::cvarrToMat(input_image);
	cv::imwrite(path.c_str(), m); */

	RegionOfInterest(input_image,0.7);
	RegionOfInterest(m_background_image,0.7);

	cvFlip(input_image, input_image, -1); //Flip image around horizontal and vertical axis to match new camera position
	cvFlip(m_background_image, m_background_image, -1); //Flip image around horizontal and vertical axis to match new camera position

	bool return_val = 0;

	double x_offset = 0;
	double y_offset = 0;
	double rot_offset = 0;

	IplImage* cv_image  ;
	IplImage* blob_image;

	CBlobGetOrientation get_orientation;

	CBlob   temp_tracked_blob;
	double  temp_tracked_blob_distance = 0;

	double maxx;
	double minx;
	double maxy;
	double miny;
	double temp_x;
	double temp_y;
	double dist_x;
	double dist_y;
	double distance;

  m_min_blob_area = 2000;
	m_max_blob_area = 90000;
	m_verticle_offset = 3;
	m_x_velocity = 0.012;
	m_y_velocity = 0.012;
	m_rot_velocity = 0.2;
	m_x_target = 0;
	m_x_threshold = 20;
	m_y_target = 0;
	m_y_threshold = 15;
	m_lost_blob_timeout = 20;

	m_rot_target = 90;
	m_rot_tolerance = 5;

	if( !input_image )
	{
		ROS_ERROR( "Error in input image!" );
		return false;
	}

	/**
	 * We now need to check and see if we have been lost for longer than the lost timeout.
	 */
	if( m_is_blob_lost )
	{
		if( ( ros::Time::now() - m_time_when_lost ).toSec() < m_lost_blob_timeout )
		{
			m_is_blob_lost = false; // Reintialize to false so that next service call is not affected
			return 2;
		}
	}

	/**
	 * In order to make Visual Servoing useful during the conveyer belt tests we need to ensure that
	 * we are able to focus only on the conveyer belt. For this reason we will resize the image so
	 * that we are lookin
g only at a region of interest instead of the whole image.
	 *
	 */
/*	if( g_operating_mode == 1 )
	{
		//cv_image = RegionOfInterest( input_image, 0.7 );
		ROS_INFO( "ROI" );
		cv_image = input_image;
	}
	else
	{
		cv_image = input_image;
	}
|*/					
	cv_image = input_image;

	m_image_height = cv_image->height;
	m_image_width = cv_image->width;

	// TODO: Modify the program so that it can still run without a background image!
	IplImage* background_threshold = cvCreateImage( cvGetSize( m_background_image ), 8, 1 );
	cvCvtColor( m_background_image, background_threshold, CV_BGR2GRAY );
	//cvSmooth( background_threshold, background_threshold, CV_GAUSSIAN, 11, 11 );
	//cvEqualizeHist(background_threshold, background_threshold);
	cvAdaptiveThreshold(background_threshold, background_threshold, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY_INV, 15, 0);
	//cvThreshold( background_threshold, background_threshold, m_dynamic_variables.binary_threshold , 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU );

	blob_image = cvCreateImage( cvGetSize( cv_image ), IPL_DEPTH_8U, cv_image->nChannels );

	IplImage* gray = cvCreateImage( cvGetSize( cv_image ), 8, 1 );
	cvCvtColor( cv_image, gray, CV_BGR2GRAY );
	//cvSmooth( gray, gray, CV_GAUSSIAN, 11, 11 );
	//cvEqualizeHist(gray, gray);

	ROS_WARN_STREAM( "Dynamic Var: " << m_dynamic_variables.binary_threshold ); 
	cvAdaptiveThreshold(gray, gray, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY_INV, 15, 0);
	//cvThreshold( gray, gray, m_dynamic_variables.binary_threshold, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU );

	//    This takes a background image (the gripper on a white background) and removes
	//  it from the current image (cv_image). The results are stored again in cv_image.
	//cvSub( gray, background_threshold, gray );

	// Find any blobs that are not white.
	CBlobResult blobs = CBlobResult( gray, NULL, 0 );

	blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, m_min_blob_area );
	blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_GREATER, m_max_blob_area );

	//  We will only grab the largest blob on the first pass from that point on we will look for the centroid
	//  of a blob that is closest to the centroid of the largest blob.
	if( m_first_pass == true )
	{
	  ROS_DEBUG( "First pass through visual servoing." );

	  CBlob largest_blob;
	  blobs.GetNthBlob( CBlobGetPerimeter(), 0, largest_blob );
	  maxx = largest_blob.MaxX();
	  minx = largest_blob.MinX();
	  maxy = largest_blob.MaxY();
	  miny = largest_blob.MinY();
	  m_tracked_x = ( ( minx + maxx ) / 2 );
	  m_tracked_y = ( ( miny + maxy ) / 2 );

	  m_first_pass = false;
	}

	std_msgs::String msg;
	if( blobs.GetNumBlobs() == 0 )
	{
		std::stringstream ss;
		ss << "NOT FOUND";
		msg.data = ss.str();

		ROS_WARN( "We have lost the blob" );
		m_time_when_lost = ros::Time::now();
		m_is_blob_lost = true;
	}
	else
	{
		std::stringstream ss;
		ss << "FOUND";
		msg.data = ss.str();

		m_is_blob_lost = false;
	}

	m_pub_visual_servoing_status.publish( msg );

	//  Go through all of the blobs and find the one that is the closest to the previously tracked blob.
	for( int x = 0; x < blobs.GetNumBlobs(); x++ )
	{
	  CBlob  temp_blob;

	  temp_blob = blobs.GetBlob( x );

	  maxx = temp_blob.MaxX();
	  minx = temp_blob.MinX();
	  maxy = temp_blob.MaxY();
	  miny = temp_blob.MinY();
	  temp_x = ( ( minx + maxx ) / 2 );
	  temp_y = ( ( miny + maxy ) / 2 );
	  dist_x = ( temp_x ) - ( m_tracked_x );
	  dist_y = ( temp_y ) - ( m_tracked_y );
	  distance = sqrt( ( dist_x * dist_x ) + ( dist_y * dist_y ) );

	  if( temp_tracked_blob_distance == 0 )
	  {
		temp_tracked_blob = temp_blob;
		temp_tracked_blob_distance = distance;
		m_tracked_x = temp_x;
		m_tracked_y = temp_y;
	  }
	  else
	  {
		if( distance < temp_tracked_blob_distance )
		{
		  temp_tracked_blob = temp_blob;
		  temp_tracked_blob_distance = distance;
		  m_tracked_x = temp_x;
		  m_tracked_y = temp_y;
		}
	  }
	}

	if( g_debugging )
	{
		cvShowImage( "GRAY - Substracted", gray );
		//  Draw the blob we are tracking as well as a circle to represent the centroid of that object.
		temp_tracked_blob.FillBlob( blob_image, CV_RGB( 0, 0, 255 ) );
		cvCircle( blob_image, cvPoint( m_tracked_x, m_tracked_y ), 10, CV_RGB( 255, 0, 0 ), 2 );
	}

	x_offset = ( m_tracked_x * 100 / 63.125 ) - ( m_image_width / 2 ); // Wierd math for present camera and gripper position // TODO later
	y_offset = ( m_tracked_y * 100 / 77.5 ) - ( (m_image_height/2) + m_verticle_offset ); // Wierd math for present camera and gripper position // TODO later
	rot_offset = get_orientation( temp_tracked_blob );
	if( rot_offset > 180 )
	{
	  rot_offset = rot_offset - 180;
	}

	bool done_x = false;
	bool done_y = false;
	bool done_t = false;

	if( m_gripper_position < 1.91622 )
	{
		m_head_left = false;
		m_head_right = true;
		done_x = BaseAdjustmentX( y_offset );
		done_y = BaseAdjustmentY( x_offset );
	}
	else if( m_gripper_position > 3.9277 )
	{
		m_head_left = true;
		m_head_right = false;
		done_x = BaseAdjustmentX( y_offset );
		done_y = BaseAdjustmentY( x_offset );
	}
	else
	{
		m_head_left = false;
		m_head_right = false;
		done_x = BaseAdjustmentX( x_offset );
		done_y = BaseAdjustmentY( y_offset );
	}
	done_t = ArmAdjustment( rot_offset );

	if( done_x && done_y && done_t )
	{
		return_val = 1;
		//DestroyPublishers(); //Taken care by visual servoing node shutdown
		ROS_INFO( "Visual Servoing Completed." );		
	}


	if( g_debugging )
	{
		// Setting up fonts for overlay information.
		CvFont font;
		cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 1, CV_AA);

		cvLine( blob_image,   cvPoint( 0, (m_image_height/2) + m_verticle_offset ), cvPoint( m_image_width, (m_image_height/2) + m_verticle_offset ), CV_RGB( 255, 0, 0 ), 2, 0 );
		cvLine( blob_image,   cvPoint( (m_image_width/2), 0 ), cvPoint( (m_image_width/2), m_image_height ), CV_RGB( 255, 0, 0 ), 2, 0 );
		cvRectangle( blob_image, cvPoint( 0, blob_image->height-m_verticle_offset ), cvPoint( blob_image->width, blob_image->height ), CV_RGB( 0, 0, 0 ), -1 );

		std::string x_str = "X: ";
		x_str += boost::lexical_cast<std::string>( x_offset );

		std::string y_str = "Y: ";
		y_str += boost::lexical_cast<std::string>( y_offset );

		std::string rot_str = "Rotation: ";
		rot_str += boost::lexical_cast<std::string>( rot_offset );

		cvPutText( blob_image, x_str.c_str(), cvPoint( 10, blob_image->height - 10 ), &font, CV_RGB( 255, 0, 0 ) );
		cvPutText( blob_image, y_str.c_str(),  cvPoint( 185, blob_image->height - 10 ), &font, CV_RGB( 255, 0, 0 ) );
		cvPutText( blob_image, rot_str.c_str(), cvPoint( 350, blob_image->height - 10 ), &font, CV_RGB( 255, 0, 0 ) );

		HUD("b-it-bots Visual Servoing", 3, cv_image, m_background_image, blob_image );
		cvWaitKey( 10 );
	}
	
	cv_bridge::CvImage cv_img_tmp;
	cv::Mat mat_image(blob_image);

	//imshow ("mat_image",mat_image);

	cv_img_tmp.encoding = sensor_msgs::image_encodings::BGR8;
	cv_img_tmp.image = mat_image;
	sensor_msgs::ImagePtr image_msg = cv_img_tmp.toImageMsg();

	m_image_publisher.publish(cv_img_tmp.toImageMsg());

	cvSetZero( gray );
	cvSetZero( cv_image );
	cvSetZero( blob_image );

	cvReleaseImage( &gray );
	cvReleaseImage( &blob_image );

	return return_val; 
}

bool
VisualServoing2D::BaseAdjustmentX( double x_offset )
{
	bool return_val = false; 
	double move_speed = 0.0;

	if( m_head_left )
	{
		if( x_offset > m_x_threshold )
		{
			// move the robot base right
			move_speed = -m_x_velocity;
			return_val = false;
		}
		else if( x_offset < -m_x_threshold )
		{
			// move the robot left
			move_speed = m_x_velocity;
			return_val = false;
		}
		else if( fabs( x_offset ) < m_x_threshold )
		{
			move_speed = 0.0;
			return_val = true;
			ROS_INFO( "Base Adjustment in X Finished" );
		}
		else
		{
			// should never happen but just in case.
			move_speed = 0.0;
		}
	}
	else if( m_head_right )
	{
		if( x_offset > m_x_threshold )
		{
			// move the robot base right
			move_speed = m_x_velocity;
			return_val = false;
		}
		else if( x_offset < -m_x_threshold )
		{
			// move the robot left
			move_speed = -m_x_velocity;
			return_val = false;
		}
		else if( fabs( x_offset ) < m_x_threshold )
		{
			move_speed = 0.0;
			return_val = true;
			ROS_INFO( "Base Adjustment in X Finished" );
		}
		else
		{
			// should never happen but just in case.
			move_speed = 0.0;
		}
	}
	else
	{
		if( x_offset > m_x_threshold )
		{
			// move the robot base right
			move_speed = -m_x_velocity;
			return_val = false;
		}
		else if( x_offset < -m_x_threshold )
		{
			// move the robot left
			move_speed = m_x_velocity;
			return_val = false;
		}
		else if( fabs( x_offset ) < m_x_threshold )
		{
			move_speed = 0.0;
			return_val = true;
			ROS_INFO( "Base Adjustment in X Finished" );
		}
		else
		{
			// should never happen but just in case.
			move_speed = 0.0;
		}
	}
	// Prepare and then send the base movement commands.
	m_youbot_base_velocities.linear.y = move_speed;
	m_base_velocities_publisher.publish( m_youbot_base_velocities );
	geometry_msgs::TwistStamped cart_vel_y_;
	cart_vel_y_.header.frame_id = "/base_link";
	cart_vel_y_.twist.linear.y = move_speed;
	//pub_cart_vel_.publish(cart_vel_y_);
	return return_val;
}

bool
VisualServoing2D::BaseAdjustmentY( double y_offset )
{
	bool return_val = false; 
	double move_speed = 0.0;

	if( m_head_left )
	{
		if( y_offset >= m_y_threshold )
		{
			// move the robot base right
			move_speed = m_y_velocity;
			return_val = false;
		}
		else if( y_offset <= -m_y_threshold )
		{
			// move the robot left
			move_speed = -m_y_velocity;
			return_val = false;
		}
		else if( fabs( y_offset ) < m_y_threshold )
		{
			move_speed = 0.0;
			return_val = true;
			ROS_INFO( "Base Adjustment in Y Finished" );
		}
		/**
		 * TODO: Change this so that we only return true when we can no longer line the object up in the
		 * y direction but the centroid of the blobHelp is still within an emergency range (praying we can
		 * grasp it). Otherwise we need to return that the object is not able to be grasped due to its
		 * distance on the platform. We could deal with this either by returning that we cannot move the
		 * object or to implement a grasp and drag scenario where we grab the last little bit and drag
		 * it into the frame. This would be the best idea as it would allow us to grab objects which are
		 * barely in our range and would not be normally graspable.
		 */
		else
		{
			// should never happen but just in case.
			return_val = true;
			move_speed = 0.0;
		}
	}
	else if( m_head_right )
	{
		if( y_offset >= m_y_threshold )
		{
			// move the robot base right
			move_speed = -m_y_velocity;
			return_val = false;
		}
		else if( y_offset <= -m_y_threshold )
		{
			// move the robot left
			move_speed = m_y_velocity;
			return_val = false;
		}
		else if( fabs( y_offset ) < m_y_threshold )
		{
			move_speed = 0.0;
			return_val = true;
			ROS_INFO( "Base Adjustment in Y Finished" );
		}
		/**
		 * TODO: Change this so that we only return true when we can no longer line the object up in the
		 * y direction but the centroid of the blobHelp is still within an emergency range (praying we can
		 * grasp it). Otherwise we need to return that the object is not able to be grasped due to its
		 * distance on the platform. We could deal with this either by returning that we cannot move the
		 * object or to implement a grasp and drag scenario where we grab the last little bit and drag
		 * it into the frame. This would be the best idea as it would allow us to grab objects which are
		 * barely in our range and would not be normally graspable.
		 */
		else
		{
			// should never happen but just in case.
			return_val = true;
			move_speed = 0.0;
		}
	}
	else
	{
		if( y_offset >= m_y_threshold )
		{
			// move the robot base right
			move_speed = -m_y_velocity;
			return_val = false;
		}
		else if( y_offset <= -m_y_threshold )
		{
			// move the robot left
			move_speed = m_y_velocity;
			return_val = false;
		}
		else if( fabs( y_offset ) < m_y_threshold )
		{
			move_speed = 0.0;
			return_val = true;
			ROS_INFO( "Base Adjustment in Y Finished" );
		}
		/**
		 * TODO: Change this so that we only return true when we can no longer line the object up in the
		 * y direction but the centroid of the blobHelp is still within an emergency range (praying we can
		 * grasp it). Otherwise we need to return that the object is not able to be grasped due to its
		 * distance on the platform. We could deal with this either by returning that we cannot move the
		 * object or to implement a grasp and drag scenario where we grab the last little bit and drag
		 * it into the frame. This would be the best idea as it would allow us to grab objects which are
		 * barely in our range and would not be normally graspable.
		 */
		else
		{
			// should never happen but just in case.
			return_val = true;
			move_speed = 0.0;
		}
	}



	// Prepare and then send the base movement commands.
	m_youbot_base_velocities.linear.x = move_speed;
	m_base_velocities_publisher.publish( m_youbot_base_velocities );
	geometry_msgs::TwistStamped cart_vel_x_;
	cart_vel_x_.header.frame_id = "/base_link";
	cart_vel_x_.twist.linear.x = move_speed;
	//pub_cart_vel_.publish(cart_vel_x_);

	return return_val;
}

bool
VisualServoing2D::ArmAdjustment( double orientation )
{
	geometry_msgs::TwistStamped cart_vel_r_;
	bool return_val = false; 
	double difference = fabs( orientation - m_rot_target );
	double rotational_speed = 0.0;


	if( orientation > m_rot_target && difference > m_rot_tolerance )
	{
		/**
		 * We are not to far to the right of the object and our difference is not small enough yet.
		 */
		rotational_speed = -m_rot_velocity; //Change the sign for joint space
		return_val = false;
	}
	else if( orientation < m_rot_target && difference > m_rot_tolerance )
	{
		/**
		 * we are to far to the left of the object and our difference is still to large.
		 */
		rotational_speed = m_rot_velocity; //Change the sign for joint space
		return_val = false;
	}
	else if( difference < m_rot_tolerance )
	{
		rotational_speed = 0.0;
		return_val = true;
		ROS_INFO( "Arm Rotation Finished" );
	}
	else
	{
		/**
		 * We should never arrive at this state but just encase.
		 */
		ROS_ERROR( "SHIT WENT WRONG!" );
		return_val = false;
	}

	ROS_INFO( "Orientation\t%f", orientation );
	ROS_INFO( "Difference\t%f", difference );

	/**
	 * we need to loop though all of the joint states because we need to set anything we do not want
	 * to move to 0. If we do not do this we could get uncontrolled movements from values that it
	 * had previously been sent.
	 */
	m_youbot_arm_velocities.velocities.clear();
	for(unsigned int i=0; i < m_arm_joint_names.size(); ++i)
	{
		brics_actuator::JointValue joint_value;

		joint_value.timeStamp = ros::Time::now();
		joint_value.joint_uri = m_arm_joint_names[i];
		joint_value.unit = to_string(boost::units::si::radian_per_second);

		if( i == 4 )
		{
			cart_vel_r_.twist.angular.z = rotational_speed;
		  	joint_value.value = rotational_speed;
		}
		else
		{
		cart_vel_r_.twist.angular.z = 0.0;
		  joint_value.value = 0.0;
		}

		m_youbot_arm_velocities.velocities.push_back(joint_value);
		
	}

	//m_arm_velocities_publisher.publish( m_youbot_arm_velocities );
	cart_vel_r_.header.frame_id = "/arm_link_5";
	pub_cart_vel_.publish(cart_vel_r_);
	return return_val;
}

void
VisualServoing2D::UpdateGripperPosition( float new_position )
{
	ROS_DEBUG( "Gripper position updated inside of VisualServoing2D" );
	m_gripper_position = new_position;
}

IplImage*
VisualServoing2D::LoadBackgroundImage()
{
	IplImage* background_image;
	std::string mode;

	if( g_operating_mode == 0 )
	{
		mode = "background.png";
	}
	else if( g_operating_mode == 1 )
	{
		mode = "conveyer_background.png";
	}
	else
	{
		ROS_ERROR( "Improper Mode (background)" );
	}

	try
	{
	  std::string package_path = ros::package::getPath("mir_visual_servoing") + "/common/data/" + mode;
	  std::cout << "Package Path:\t" << package_path.c_str() << std::endl;
	  background_image = cvLoadImage( package_path.c_str() );
	}
	catch ( cv::Exception& e )
	{
		ROS_ERROR( "Could not load background image" );
	}

	return background_image;
}

IplImage*
VisualServoing2D::RegionOfInterest( IplImage* input_image, double scale )
{
	if( scale <= 0 || scale >= 1 )
	{
		ROS_ERROR( "Invalid scale provided setting to 0.7" );
		scale = 0.7;
	}

	int width = (int)(input_image->width * scale );
	int height = (int)( input_image->height * scale );

	int x = ( (input_image->width - width) / 2 );
	int y = ( (input_image->height - height) / 2 );

	cvSetImageROI( input_image, cvRect( x, y, width-0.45*x, height+0.5*y ) ); //Wierd math for present camera and gripper position - TODO later

	if( g_debugging )
	{

	}

	return input_image;
}

void
VisualServoing2D::CreatePublishers( int arm_model )
{
	// Set up the base velocities publisher:
	m_base_velocities_publisher = m_node_handler.advertise<geometry_msgs::Twist>( "/cmd_vel", 1 );

	pub_cart_vel_ = m_node_handler.advertise< geometry_msgs::TwistStamped > ("/arm_1/arm_controller/cartesian_velocity_command", 1);
	
	ROS_INFO( "Robot Base Publisher Setup" );

	m_pub_visual_servoing_status = m_node_handler.advertise<std_msgs::String>( "/visual_servoing_status", 1 );
	ROS_INFO( "VISUAL SERVOING STATUS PUBLSHING" );

	if( arm_model == 0 )
	{
		ROS_INFO( "The robot has no arm to move." );
	}
	else if( arm_model == 1 )
	{
		m_arm_velocities_publisher = m_node_handler.advertise<brics_actuator::JointVelocities>( "/arm_1/arm_controller/velocity_command", 1 );
		ROS_INFO( "KUKA YouBot Arm Publisher is set up" );
	}
	else if( arm_model == 2 )
	{
		ROS_ERROR( "KUKA Lightwieght Arm has not been implemented" );
	}
	else
	{
		ROS_ERROR( "Unkown robotic arm model provided" );
	}

	m_image_publisher = m_image_transport.advertise("/arm_cam/visual_servoing/debug_image", 1);
}

void
VisualServoing2D::DestroyPublishers()
{
	/**
	 * Zero and shutdown the base velcoity publisher.
	 */
	geometry_msgs::Twist zero_vel;
	m_base_velocities_publisher.publish( zero_vel );
	
	ROS_INFO( "Base velocity publisher zeroed and shutdown" );

	/**
	 * Zero and shutdown the arm velocity publisher.
	 */
	brics_actuator::JointValue zero_joint_vel;
	brics_actuator::JointVelocities 	zero_arm_velocities;
	zero_joint_vel.timeStamp = ros::Time::now();
	zero_joint_vel.joint_uri = m_arm_joint_names[4];
	zero_joint_vel.unit = to_string(boost::units::si::radian_per_second);
	zero_joint_vel.value = 0.0;
	zero_arm_velocities.velocities.push_back(zero_joint_vel);
	//m_arm_velocities_publisher.publish( zero_arm_velocities );

	cart_zero_vel_.header.frame_id = "/arm_link_5";
	pub_cart_vel_.publish(cart_zero_vel_);

	ros::Rate(10).sleep();

	m_arm_velocities_publisher.shutdown();
	m_base_velocities_publisher.shutdown();
	pub_cart_vel_.shutdown();
	m_image_publisher.shutdown();
	
	ROS_INFO( "Arm velcoity publisher zeroed and shutdown" );

}

void
VisualServoing2D::HUD(std::string title, int nArgs, ...) {

    // img - Used for getting the arguments
    IplImage *img;

    // DispImage - the image in which input images are to be copied
    IplImage *DispImage;

    int size;
    int i;
    int m, n;
    int x, y;

    // w - Maximum number of images in a row
    // h - Maximum number of images in a column
    int w, h;

    // scale - How much we have to resize the image
    float scale;
    int max;

    // If the number of arguments is lesser than 0 or greater than 12
    // return without displaying
    if(nArgs <= 0) {
        printf("Number of arguments too small....\n");
        return;
    }
    else if(nArgs > 12) {
        printf("Number of arguments too large....\n");
        return;
    }
    // Determine the size of the image,
    // and the number of rows/cols
    // from number of arguments
    else if (nArgs == 1) {
        w = h = 1;
        size = 300;
    }
    else if (nArgs == 2) {
        w = 2; h = 1;
        size = 300;
    }
    else if (nArgs == 3 || nArgs == 4) {
        w = 2; h = 2;
        size = 350;
    }
    else if (nArgs == 5 || nArgs == 6) {
        w = 3; h = 2;
        size = 200;
    }
    else if (nArgs == 7 || nArgs == 8) {
        w = 4; h = 2;
        size = 200;
    }
    else {
        w = 4; h = 3;
        size = 150;
    }

    // Create a new 3 channel image
    DispImage = cvCreateImage( cvSize( 50 + size*w, 60 + size*h), 8, 3 );

    // Used to get the arguments passed
    va_list args;
    va_start(args, nArgs);

    // Loop for nArgs number of arguments
    for (i = 0, m = 20, n = 20; i < nArgs; i++, m += (20 + size)) {

        // Get the Pointer to the IplImage
        img = va_arg(args, IplImage*);

        // Check whether it is NULL or not
        // If it is NULL, release the image, and return
        if(img == 0) {
            printf("Invalid arguments");
            cvReleaseImage(&DispImage);
            return;
        }

        // Find the width and height of the image
        x = img->width;
        y = img->height;

        // Find whether height or width is greater in order to resize the image
        max = (x > y)? x: y;

        // Find the scaling factor to resize the image
        scale = (float) ( (float) max / size );

        // Used to Align the images
        if( i % w == 0 && m!= 20) {
            m = 20;
            n+= 20 + size;
        }

        // Set the image ROI to display the current image
        cvSetImageROI(DispImage, cvRect(m, n, (int)( x/scale ), (int)( y/scale )));

        // Resize the input image and copy the it to the Single Big Image
        cvResize( img, DispImage );

        // Reset the ROI in order to display the next image
        cvResetImageROI(DispImage);
    }

    // Create a new window, and show the Single Big Image
    cvNamedWindow( title.c_str(), 1 );
    cvShowImage( title.c_str(), DispImage);

    //cvWaitKey();
    //cvDestroyWindow(title);

    // End the number of arguments
    va_end(args);

    // Release the Image Memory
    //cvReleaseImage(&DispImage);
}

void 
VisualServoing2D::UpdateDynamicVariables( mir_visual_servoing::VisualServoingConfig config )
{
	m_dynamic_variables = config; 
}
