#ifndef VISION_BRIDGE_H
#define VISION_BRIDGE_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

namespace au_vision {

/**
* @class VisionBridge
* @ingroup au_vision
* @brief Static helper class for converting and publishing image messages in ROS from OpenCV
*/
class VisionBridge {
private:
	ros::NodeHandle nh_;

public:
	/*
	 * @breif Empty Constructor for VisionBridge
	 */
	VisionBridge();

	/**
	 * @breif Converts a ROS image message to OpenCV Mat format
	 * @param msg The ROS image message with sensor_msgs::image_encodings::BGR8 encoding
	 * @return OpenCV Mat
	 *
	 * TODO: Handle more message encodings
	 */
	static cv::Mat toCvMat(const sensor_msgs::ImageConstPtr& msg);

	/**
	 * @breif Converts a Mat to a ROS image message
	 * @param mat The Mat to convert 
	 * @return ROS image message with sensor_msgs::image_encodings::BGR8 encoding
	 *
	 * TODO: Handle more message encodings
	 */
	static sensor_msgs::ImagePtr toImgMsg(cv::Mat& mat);

	/**
	 * @breif Publishes an image  
	 * @param mat The image to publish
	 * @param pub Publisher instance, advertizing on a topic
	 */
	static void publishImage(cv::Mat& mat, ros::Publisher& pub);

	/**
	 * @breif Publishes an image  
	 * @param mat The image to publish
	 * @param node The topic to advertize on
	 * @param queue_size Publisher queue size
	 */
	void publishImage(cv::Mat& mat, std::string node, int queue_size = 1);

	/**
	 * @breif Creates an image publisher  
	 * @param nh NodeHandle instance
	 * @param topic_name The topic to advertize on
	 * @param queue_size Publisher queue size
	 */
	static ros::Publisher createPublisher(ros::NodeHandle nh, std::string topic_name, int queue_size);
};

} //namespace au_vision

#endif //VISION_BRIDGE_H
