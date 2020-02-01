#include <mir_perceive_aruco_cube/aruco_cube_perceiver.h>

ArucoCubePerceiver::ArucoCubePerceiver() : nh("~"), image_transporter_(nh)
{
    transform_listener_ = new tf::TransformListener();

/*     nh.param<std::string>("target_frame", target_frame_, "/base_link"); */
/*     nh.param<int>("num_of_retrial", num_of_retrial_, 30); */
/*     nh.param<int>("num_pixels_to_extrapolate", num_pixels_to_extrapolate_, 30); */


    image_pub_ = image_transporter_.advertise("debug_image", 1);

    sub_pointcloud_.subscribe(nh, "input_pointcloud", 10);
    sub_image_.subscribe(nh, "input_image", 10);

    sync_input_ = boost::make_shared<message_filters::Synchronizer<ImageSyncPolicy> > (10);
    sync_input_->connectInput(sub_pointcloud_, sub_image_);
    sync_input_->registerCallback(boost::bind(&ArucoCubePerceiver::synchronizedCallback, this, _1, _2));

}

ArucoCubePerceiver::~ArucoCubePerceiver()
{
}

void ArucoCubePerceiver::synchronizedCallback(const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg,
                                              const sensor_msgs::Image::ConstPtr &image_msg)
{
    ROS_INFO("inside sync cb");
    cv_bridge::CvImagePtr input_img_ptr;
    bool success = this->imgToCV(image_msg, input_img_ptr);
    if (!success)
        return;

    cv::Mat img = input_img_ptr->image;
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    std::cout << "before detectMarkers" << std::endl;
    cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
    std::cout << "after detectMarkers" << std::endl;
    for (int i : markerIds) {
        std::cout << i << std::endl;
    }
    /* cv::Mat outputImage = inputImage.clone(); */
    /* cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds); */
    /* for (int i : markerIds) { */
    /*     std::cout << i << std::endl; */
    /* } */
    // Draw an example circle on the video stream for debugging
    if (input_img_ptr->image.rows > 60 && input_img_ptr->image.cols > 60)
        cv::circle(input_img_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Output modified video stream
    image_pub_.publish(input_img_ptr->toImageMsg());
}

bool ArucoCubePerceiver::imgToCV(const sensor_msgs::Image::ConstPtr &image_msg, cv_bridge::CvImagePtr &cv_ptr)
{
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_cube_perceiver");
    ArucoCubePerceiver perceiver;

    ros::Rate loop_rate(10.0);

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
