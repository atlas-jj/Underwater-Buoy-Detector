/**
 * @file bin_detector.cpp
 * @author Jacky Chung
 * @date 01 Jan 2017
 * @brief Header for the Bin Detector class
 *
 * BinDetector object takes in a image and draws a rectangle on the 
 * inside of the bin
 * 
 */
#include <au_vision/detector.h>

#ifndef AU_VISION_BIN_DETECTOR_H
#define AU_VISION_BIN_DETECTOR_H

#include <math.h>

/**
 *  @ingroup au_vision
 *  @brief Class BinDetector detector to detect marker bins
 */
namespace au_vision
{
	class BinDetector : public Detector 
	{
		public:
		
			BinDetector( ros::NodeHandle& nh, ros::NodeHandle& private_nh );
			~BinDetector();
		
		protected:
			
			/**
			* @brief detects rectangles 
			* @param input input image to detect from
			*/
			std::vector<au_core::Roi> detect( const cv::Mat &input/*, cv::Mat &outFrame*/);

			/**
			 * @brief checks cosine between 3 points, angle of pt1
			 * @param pt1
			 * @param pt2
			 * @param pt0
			*/
			double angle(cv::Point &pt1, cv::Point &pt2, cv::Point &pt0);

			/**
			 * @brief checks if the rectangle inner is contained by the rectangle outer
			 * @param outer vector of 4 points that defines a rectangle
			 * @param inner another vector of 4 points that defines a rectangle
			*/
			bool containsRect(std::vector<cv::Point> &outer, std::vector<cv::Point> &inner);

			// was only used for testing
			/** 
			 * @brief draws rectanlges in green and contained rectangles in red, testing only
			 * @param rectangles a vector containing vectors of points that make a rectangle
			 * @param image the image that we draw on
			*/
			void drawSquares(std::vector<std::vector<cv::Point> > &rectangles/*, cv::Mat &image*/);

			/**
			 * @brief filters an image for bin detection
			 * @param input image to be filtered
			*/
			void filterImage(cv::Mat &input);

			/** 
			 * @brief converts found rectangles into ROIs
			 * @param rectangles a vector containing vectors of points that make a rectangle
			 * @param roiArray vector that holds ROIs
			 * @param yellowPoint a point that we use to check against to filter our rectangles that contain that point
			*/			
			void getSquares(std::vector<std::vector<cv::Point> > &rectangles, std::vector<au_core::Roi> &roiArray, const cv::Point yellowPoint); 
			
			/**
			 * @brief checks the ratio of the rectangle's height to width
			 * @param rectangle a vector of points that define a dectangle
			 * @param lower the lower bound of the allowed ratio
			 * @param upper the upper bound of the allowed ratio
			*/
			bool checkRatio(std::vector<cv::Point> &rectangle, float lower, float upper);

			/**
			 * @brief given a bounding rectangle, get the ROI
			 * @param rectangle a bounding rectangle
			 *
			*/
			au_core::Roi getParams(cv::Rect &rectangle); 

			/**
			 * @brief given a vector of bins, find the largest one
			 * @param bins a vector of bins
			 * @param highest a bin passed by reference to hold the largest bin
			*/
			void largestContour(std::vector<std::vector<cv::Point> > &bins, std::vector<cv::Point> &highest);


	};

} // end of namespace

#endif //AU_VISION_BIN_DETECTOR_H
