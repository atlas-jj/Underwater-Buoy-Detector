/**
 * @file tracker.h
 * @author Rumman Waqar
 * @date 2 Sep 2015
 * @brief Header for the Tracker class
 *
 * Tracker object initializes a specific tracker. It uses
 * ROS to pass in images, and pass out processed ROIs and
 * debug images that show tracking.
 *
 * Currently Tracker class is only built for single object tracking.
 *
 * The tracking object is configured using the following ROS params:
 *  * tracker: name of the tracker to use
 *
 * Subscribes to:
 *  * camera [sensor_msgs::Image]
 *  * inputRoi [au_core::Roi]
 *
 * Publishes to:
 *  * debug [sensor_msgs::Image]
 *  * outputRoi [au_core::Roi]
 *
 * @todo send message if tracking lost
 */

#ifndef AU_VISION_TRACKER_H
#define AU_VISION_TRACKER_H

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <au_core/Roi.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>

namespace au_vision
{

/**
 * @class Tracker
 * @ingroup au_vision
 * @brief Manages s specific tracker, by handling all the inputs and outputs
 *
 * Instantiates a specific tracker as specified by the ros params. Handles all
 * inputs and outputs to the tracker.
 *
 */
class Tracker
{

 public:
	/**
	 * @brief Create a Tracker
	 * @param nh node handler
	 * @param private_nh node handler inside the private namespace of the node
	 *
	 * Creates the publishers and subscribers. Reads ros params and creates the
	 * specfic tracker object.
	 */
	Tracker( ros::NodeHandle& nh, ros::NodeHandle& private_nh );

	/**
	 * @brief Destroy Tracker
	 *
	 * Shuts down all publishers and subscribers. Destroys the specfic tracker
	 */
	virtual ~Tracker();

 protected:
	/**
	 * @brief create publishers and subscribers
	 */
	void initializeIo();

	/**
	 * @brief virtual func. implement to initialize or reinitialize the tracker
	 * @param frame image with tracked object
	 * @param roi region of interest containing the tracked object
	 * @return returns true if initialization successful
	 */
	virtual bool initializeTracker( cv::Mat& frame, cv::Rect roi ) = 0;

	/**
	 * @brief virtual func. implement to track object
	 * @param frame image to use for tracking
	 * @return returns ROI, if failed returns rectangle with area 0
	 */
	virtual cv::Rect track( cv::Mat& frame ) = 0;

	/**
	 * @brief draw roi on image
	 * @param frame input image to draw on (does not modify original image)
	 * @param roi region of interest containing the tracked object
	 */
	void debugDraw( cv::Mat& frame, cv::Rect roi );

	/**
	 * @brief callback function for input video subscriber
	 * @param msg constant pointer to the image msg
	 *
	 * Calls the image processing function and passes it image as opencv mat
	 */
	void inputImageCallback( const sensor_msgs::ImageConstPtr& msg );

	/**
	 * @brief callback function for input ROI subscriber
	 * @param msg constant pointer to roi to track
	 */
	void roiCallback( const au_core::RoiConstPtr msg );


	ros::NodeHandle nh_;         /**< node handler */
	ros::NodeHandle private_nh_; /**< node handler inside the private ns */

	std::string trackerType_;   /**< type of tracker used */
	cv::Rect trackedWindow_;    /**< stores current ROI */

 private:
	image_transport::Subscriber inputImageSub_; /**< subscriber to input video */
	ros::Subscriber roiSubscriber_;             /**< input started ROI by detector */
	image_transport::Publisher debugImagePub_;  /**< publisher for debug output video */
	ros::Publisher roiPublisher_;               /**< publisher for output roi of tracked object */

	cv_bridge::CvImage cvBridgeImage_;          /**< cv_bridge object to convert between ros and opencv */

	cv::Mat inputImage_;        /**< input image converted to opencv mat */
	cv::Mat outputImage_;       /**< output image with ROI converted to opencv mat */

	bool isTracking_;            /**< are we tracking? */
}; // end of Tracker class

} // end of namespace

#endif //AU_VISION_TRACKER_H
