#ifndef VISION_BRIDGE
#define VISION_BRIDGE

#include <au_vision/vision_bridge.h>

namespace au_vision {

VisionBridge::VisionBridge() {

}

cv::Mat VisionBridge::toCvMat(const sensor_msgs::ImageConstPtr& msg) {
	//TODO: Make shared ptr
	cv_bridge::CvImagePtr cv_ptr;

	try	{
		//TODO: Use toCvShare
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		throw e;
	}

	return cv_ptr->image;
}

//TODO: take encoding as second parameter
sensor_msgs::ImagePtr VisionBridge::toImgMsg(cv::Mat& mat) {
	return cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat).toImageMsg();
}

void VisionBridge::publishImage(cv::Mat& mat, ros::Publisher& pub) {
	//Publisher should be advertising sensor_msgs::Image
	pub.publish(toImgMsg(mat));
}

void VisionBridge::publishImage(cv::Mat& mat, std::string node, int queue_size) {
	ros::Publisher pub = nh_.advertise<sensor_msgs::Image>(node, queue_size);
	publishImage(mat, pub);
}

ros::Publisher VisionBridge::createPublisher(ros::NodeHandle nh, std::string topic_name, int queue_size) {
	return nh.advertise<sensor_msgs::Image>(topic_name, queue_size);
}

} //namespace au_vision

#endif //VISION_BRIDGE
