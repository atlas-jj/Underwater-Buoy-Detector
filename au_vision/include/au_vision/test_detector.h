/**
 * @file test_detector.h
 * @author Sean Scheideman
 * @date 30 Oct 2016
 * @brief Header for the TestDetector class
 *
 * Test class used to test the Detector virtual class. It picks 3 random ROI's as detected objects
 *
 * The tracking object is configured using the following ROS params:
 *  * detector: "test_detector"
 *
 * Subscribes to:
 *  * inputFrame [sensor_msgs::Image]
 *
 * Publishes to:
 *  * debug [sensor_msgs::Image]
 *  * outputRoiArray [au_vision::RoiArray] @TODO move to au_core
 *
 */

#include <au_vision/detector.h>

#ifndef AU_VISION_TEST_DETECTOR_H
#define AU_VISION_TEST_DETECTOR_H

namespace au_vision
{
    class TestDetector : public Detector
    {
    public:

       /**
        * @brief creates TestDetector
        * @param nh node handler
        * @param private_nh node handler inside the private namespace of the node
        *
        * Calls parent constructor which initializes ROS I/O
        */
       TestDetector( ros::NodeHandle& nh, ros::NodeHandle& private_nh );

       /**
        * @brief destroys TestTracker
        */
       ~TestDetector();

    protected:

       /**
       * @brief dummy function. returns vector of 3 random ROIs
       * @param frame image to use for detection
  	   * @return returns RoiArray, if failed returns RoiArray of length 0
       */
       std::vector<au_core::Roi> detect( const cv::Mat& frame );
   };
} // end of namespace

#endif //AU_VISION_TEST_DETECTOR_H
