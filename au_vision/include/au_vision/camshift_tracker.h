/**
 * @file camshift_tracker.h
 * @author Sean Scheideman
 * @date 4 Sept 2016
 * @brief Header for CamshiftTracker class
 *
 * Uses camshift to track object.
 *
 * The tracking object is configured using the following ROS params:
 *  * tracker: "camshift_tracker"
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

#ifndef AU_VISION_CAMSHIFT_TRACKER_H_
#define AU_VISION_CAMSHIFT_TRACKER_H_

#include <iostream>

#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <au_vision/tracker.h>

namespace au_vision {

/**
 *  @ingroup au_vision
 *  @brief Class CamshiftTracker creates tracker using OpenCV Camshift and
 *  backprojection algorithms
 */
class CamshiftTracker : public Tracker
{

 public:

     /**
      * @brief creates CamshiftTracker
      * @param nh node handler
      * @param private_nh node handler inside the private namespace of the node
      *
      * Calls parent constructor which initializes ROS I/O
      */
     CamshiftTracker( ros::NodeHandle& nh, ros::NodeHandle& private_nh );

     /**
      * @brief destroys CamShiftTracker
      */
     virtual ~CamshiftTracker();

 protected:

     /**
      *  @brief Initializes camshift tracker using rectangular roi and video frame.
      *  @param roi rectangle referencing region of interest area
      *  @param frame video frame \p roi is referencing
      *
      *  Creates historgram of Hue channel for roi, which is used by
      *  backprojection algorithm.
      */
     bool initializeTracker( cv::Mat& frame, cv::Rect roi );

     /**
      * @brief Uses camshift algorithm to find tracked ROI
      * @param frame
      * @return Rect containing tracked object
      */
     cv::Rect track( cv::Mat& frame );

 private:
     cv::Mat hsv_, mask_;
     cv::Mat histogram_;                       /**>Histogram model for initial roi*/
     cv::Mat backproj_;                        /**>Pixel wise probability image*/
     const int channels_[2] = { 0, 1 };
     int valueMax_, valueMin_;                 /**>Value Threshold values */
     int saturationMin_, saturationMax_;       /**>Saturation Threshold values */
     const int hueBins_ = 32 , saturationBins_ = 32;
     const int histogramSize_[2] = { hueBins_, saturationBins_ };
     const float hueRange_[2] = { 0, 180 };
     const float saturationRange_[2] = { 0, 255 };
     const float* ranges_[2] = { hueRange_, saturationRange_ }; /**> Histogram dimensions */

     /**
      * @brief Converts to HSV colorspace and creates threshold mask
      * @param frame frame to be processed
      */
     void processFrame( cv::Mat& frame );
};

} // namespace au_vision

#endif  // AU_VISION_CAMSHIFT_TRACKER_H_
