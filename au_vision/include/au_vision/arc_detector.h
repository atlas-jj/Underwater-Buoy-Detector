/**
 * @file test_detector.h
 * @author Sean Scheideman
 * @date 30 Oct 2016
 * @brief Header for the arc_Detector class
 *
 * Test class used to test the Detector virtual class. It picks 3 random ROI's as detected objects
 *
 * The tracking object is configured using the following ROS params:
 *  * detector: "arc_detector"
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
 #include <iostream>
 #include <cstdlib>
 #include <fstream>
 #include <list>
 #include <stack>
 #include <ros/ros.h>
 #include <cmath>
 //#include <geometry_msgs/Twist.h>//for geometry_mesgs::Twist
 #include "std_msgs/String.h"
 #include <cv_bridge/cv_bridge.h>
 #include <opencv2/imgproc/imgproc.hpp>
 #include <opencv2/highgui/highgui.hpp>
 #include <opencv2/core/core.hpp>
 #include <opencv2/opencv.hpp>
 extern "C"{
 #include "au_vision/external/image.h"
 #include "au_vision/external/pgm.h"
 #include "au_vision/external/misc.h"
 #include "au_vision/external/svg.h"
 #include "au_vision/external/polygon.h"
 #include "au_vision/external/ring.h"
 #include "au_vision/external/elsdc.h"
 }

 #include <au_vision/vision_bridge.h>


#ifndef AU_VISION_ARC_DETECTOR_H
#define AU_VISION_ARC_DETECTOR_H

namespace au_vision
{
    class ArcDetector : public Detector
    {
    public:

       /**
        * @brief creates TestDetector
        * @param nh node handler
        * @param private_nh node handler inside the private namespace of the node
        *
        * Calls parent constructor which initializes ROS I/O
        */
       ArcDetector( ros::NodeHandle& nh, ros::NodeHandle& private_nh );

       /**
        * @brief destroys TestTracker
        */
       ~ArcDetector();



    protected:
      /**
      * @brief dummy function. returns vector of 3 random ROIs
      * @param frame image to use for detection
     * @return returns RoiArray, if failed returns RoiArray of length 0
      */
      cv::Mat src_HLS;
      std::vector<au_core::Roi> detect( const cv::Mat& frame );
      PImageDouble read_pgm_image_double1(cv::Mat cv_image );
      //std::string doubleToString(double db);

   };
} // end of namespace

#endif //AU_VISION_TEST_DETECTOR_H
