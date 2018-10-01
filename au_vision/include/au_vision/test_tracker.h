/**
 * @file test_tracker.h
 * @author Rumman Waqar
 * @date 5 Sep 2015
 * @brief Header for the TestTracker class
 *
 * Test class used to test the tracking virtual class. It sets the inputROI to outputROI.
 *
 * The tracking object is configured using the following ROS params:
 *  * tracker: "test_tracker"
 *
 * Subscribes to:
 *  * camera [sensor_msgs::Image]
 *  * inputRoi [sensor_msgs::RegionOfInterest]
 *
 * Publishes to:
 *  * debug [sensor_msgs::Image]
 *  * outputRoi [sensor_msgs::RegionOfInterest]
 *
 */


#ifndef AU_VISION_TEST_TRACKER_H
#define AU_VISION_TEST_TRACKER_H

#include <au_vision/tracker.h>

namespace au_vision
{

class TestTracker : public Tracker
{

 public:

	/**
	 * @brief creates TestTracker
	 * @param nh node handler
	 * @param private_nh node handler inside the private namespace of the node
	 *
	 * Calls parent constructor which initializes ROS I/O
	 */
	TestTracker( ros::NodeHandle& nh, ros::NodeHandle& private_nh );

	/**
	 * @brief destroys TestTracker
	 */
	~TestTracker();

 protected:

	/**
	 * @brief initialize or reinitialize the tracker
	 * @param frame image with tracked object
	 * @param roi region of interest containing the tracked object
	 * @return returns true if initialization successful
	 */
	bool initializeTracker( cv::Mat& frame, cv::Rect roi );

	/**
	* @brief dummy function. returns previous ROI
	* @param frame image to use for tracking
	* @return returns ROI, if failed returns rectangle with area 0
	*/
	cv::Rect track( cv::Mat& frame );

};

} // end of namespace

#endif //AU_VISION_TEST_TRACKER_H
