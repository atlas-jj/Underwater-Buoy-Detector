/**
 * @file detector.h
 * @author Sean Scheideman
 * @date 15 Oct 2015
 * @brief Header for the Detector class
 *
 * Detector object initializes a specific object detector method. It uses
 * ROS to pass in images, and pass out ROIs and
 * debug images that show detection.
 *
 * The tracking object is configured using the following ROS params:
 *  * detector: name of the detector to use
 *
 * Subscribes to:
 *  * inputFrame [sensor_msgs::Image]
 *
 * Publishes to:
 *  * debug [sensor_msgs::Image]
 *  * outputRoiArray [au_vision::RoiArray] @TODO move to au_core
 *
 */

#ifndef AU_VISION_DETECTOR_H
#define AU_VISION_DETECTOR_H

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <au_core/Roi.h>
#include <au_core/RoiArray.h>
#include <opencv2/core/core.hpp>
#include <au_vision/DetectObjects.h>
#include <au_vision/DetectObjectsAction.h>
#include <actionlib/server/simple_action_server.h>

namespace au_vision
{

/**
 * @class Detector
 * @ingroup au_vision
 * @brief Manages specific detector, by handling all the inputs and outputs
 *
 * Instantiates a specific object detection method as specified by the ros params.
 * Handles all inputs and outputs to the detector.
 *
 */
class Detector
{

 public:
	/**
	 * @brief Create a detector
	 * @param nh node handler
	 * @param private_nh node handler inside the private namespace of the node
	 *
	 * Creates the publishers and subscribers. Reads ros params and creates the
	 * specfic detector object.
	 */
	Detector( ros::NodeHandle& nh, ros::NodeHandle& private_nh );

	/**
	 * @brief Destroy detector
	 *
	 * Shuts down all publishers and subscribers. Destroys the specfic detector
	 */
	virtual ~Detector();

    /**
     * @brief callback function for ROS service server
     * @param request pointer to the image request message
     * @param response pointer to the RoiArray response message
     * @return returns boolean, true if success
     *
     * Calls the Detector detect function and returns true if detected objects
     * and puts detected ROI's in response
     */
    bool inputImageDetectRequest(au_vision::DetectObjects::Request& request,
             au_vision::DetectObjects::Response& response);

    /**
     * @brief callback function for ROS action server
     * @param frame input image for detection
     * @return returns roiArray, empty if failed
     *
     * Calls the Detector detect function and returns roiArray
     */
    std::vector<au_core::Roi> inputImageDetectAction( const sensor_msgs::Image& frame );

 protected:
	/**
	 * @brief create publishers and subscribers
	 */
	void initializeIo();

	/**
	 * @brief virtual func. implements object detection method
	 * @param frame image to use for detection
	 * @return returns RoiArray, if failed returns RoiArray of length 0
	 */

	virtual std::vector<au_core::Roi> detect( const cv::Mat& frame ) = 0;

	/**
	 * @brief draw roi on image
	 * @param frame input image to draw on (does not modify original image)
	 * @param roi region's of interest containing the detected object
	 */

	void debugDraw( cv::Mat& frame, std::vector<au_core::Roi> );

    /**
     *  @brief convert sensor_msgs::Image to cv::Mat
     *  @param input sensor_msgs::Image
     *  @param output cv::Mat
     *  @return returns bool, true if success
     */
    bool convertToMat( const sensor_msgs::Image &frame, cv::Mat &output );

	ros::NodeHandle nh_;              /**< node handler */
	ros::NodeHandle private_nh_;      /**< node handler inside the private ns */

	std::string detectorType_;        /**< type of detector used */

	std::vector<au_core::Roi> roiArray_;    /**< stores current ROI's */

 private:
	image_transport::Publisher debugImagePub_;  /**< publisher for debug output video */

	cv_bridge::CvImage cvBridgeImage_;          /**< cv_bridge object to convert between ros and opencv */

	cv::Mat inputImage_;        /**< input image converted to opencv mat */
	cv::Mat outputImage_;       /**< output image with ROI converted to opencv mat */

}; // end of detector class

} // end of namespace

#endif //AU_VISION_DETECTOR_H
