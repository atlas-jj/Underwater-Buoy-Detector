/**
 * @file naive_buoy_detector.cpp
 * @author Jacky Chung
 * @date 01 Jan 2017
 * @brief Header for the Buoy Detector class
 *
 * ContourDetector object takes in a image and draws a rectangle on the 
 * inside of the bin
 * 
 */
#include <au_vision/detector.h>

#ifndef AU_VISION_NAIVE_BUOY_DETECTOR_H
#define AU_VISION_NAIVE_BUOY_DETECTOR_H

#include <math.h>



/**
 *  @ingroup au_vision
 *  @brief Class NaiveBuoyDetector detector to detect buoys
 */
namespace au_vision
{
	class NaiveBuoyDetector : public Detector 
	{

		public:
	    
	      	NaiveBuoyDetector( ros::NodeHandle& nh, ros::NodeHandle& private_nh );
			~NaiveBuoyDetector();

		protected:
			/**
			* @brief detects buoys 
			* @param input input image to detect from
			*/
			std::vector<au_core::Roi> detect( const cv::Mat &input/*, cv::Mat &outFrame*/);

			/**
			 * @brief given a vector of contours, find the largest one
			 * @param contours a vector of contours
			 * @param highest a contour passed by reference to hold the largest contour
			*/
			void largestContour(std::vector<std::vector<cv::Point> > &contours, std::vector<cv::Point> &highest);

			/**
			 * @brief sets the low/high x and y of the rect that defines a buoy
			 * @param highest the contour that is the buoy
			 * @param lowX the lowest x value of a point on the buoy
			 * @param lowY the lowest y value of a point on the buoy
			 * @param highX the highest x value of a point on the buoy
			 * @param highY the highest y value of a point on the buoy
			 *
			*/
			void getBuoy(std::vector<cv::Point> &highest, int &lowX, int &lowY, int &highX, int &highY);
			
			/**
			 * @brief given a bounding rectangle, get the ROI
			 * @param highest a bounding rectangle
			 *
			*/
			void getRoiParams(au_core::Roi &roi, std::vector<cv::Point> &highest);

			/**
			 * @brief checks the ratio of the rectangle's height to width
			 * @param rectangle a vector of points that define a dectangle
			 * @param lower the lower bound of the allowed ratio
			 * @param upper the upper bound of the allowed ratio
			*/
			bool checkRatio(std::vector<cv::Point> &rectangle, float lower, float upper);

	};

} // end of namespace

#endif //AU_VISION_CONTOUR_DETECTOR_H
