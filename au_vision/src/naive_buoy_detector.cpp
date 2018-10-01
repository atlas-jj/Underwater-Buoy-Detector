/**
 * @file buoy.cpp
 * @author Jacky Chung
 * @date 01 Jan 2017
 * @brief Implementation for the Buoy Detector class
 *
 */

#include <au_vision/naive_buoy_detector.h>
#include <opencv2/highgui/highgui.hpp>

namespace au_vision
{
    NaiveBuoyDetector::NaiveBuoyDetector(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
	  : Detector( nh, private_nh )
	{
		detectorType_ = "naive_buoy_detector";
	}

	NaiveBuoyDetector::~NaiveBuoyDetector()
	{

	}

	std::vector<au_core::Roi> NaiveBuoyDetector::detect(const cv::Mat &_input)
	{
	    
	    std::vector<au_core::Roi> roiArray;
	    
	    cv::Mat input;
		cvtColor(_input, input, cv::COLOR_RGB2HSV);

		cv::Mat lower_red, upper_red;
		cv::Mat red, yellow, green;

		cv::inRange(input, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red);
		cv::inRange(input, cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255), upper_red);
		cv::bitwise_or(lower_red, upper_red, red);
		
		cv::inRange(input, cv::Scalar(20, 100, 100), cv::Scalar(40, 255, 255), yellow);
		cv::inRange(input, cv::Scalar(50, 100, 100), cv::Scalar(70, 255, 255), green);
		
		std::vector<std::vector<cv::Point> > contours_red, contours_yellow, contours_green, rectangles;
		
		std::vector<cv::Point> highest_red, highest_yellow, highest_green;

		cv::findContours(red, contours_red, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point( 0, 0) );
		cv::findContours(yellow, contours_yellow, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point( 0, 0) );
		cv::findContours(green, contours_green, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point( 0, 0) );

		au_core::Roi roiRed, roiYellow, roiGreen;
		
		if(contours_red.size() > 0)
		{
			largestContour(contours_red, highest_red);
			if(highest_red.size() > 0)
			{
				getRoiParams(roiRed, highest_red);
				roiArray.push_back(roiRed);
			}
		}

		if(contours_yellow.size() > 0)
		{
			largestContour(contours_yellow, highest_yellow);
			if(highest_yellow.size() > 0)
			{
				getRoiParams(roiYellow, highest_yellow);
				roiArray.push_back(roiYellow);
			}
		}

		if(contours_green.size() > 0)
		{
			largestContour(contours_green, highest_green);
			if(highest_green.size() > 0)
			{
				getRoiParams(roiGreen, highest_green);
				roiArray.push_back(roiGreen);
			}
		}

	    return roiArray;
		
	}

	void NaiveBuoyDetector::getRoiParams(au_core::Roi &roi, std::vector<cv::Point> &highest) 
	{
		
		int lowX, lowY, highX, highY;
		int xOffSet, yOffSet;
		int width, height;
		int largeWidth, largeHeight;
		int factor = 2;

		lowX = lowY = 10000;
		highX = highY = 0;

		getBuoy(highest, lowX, lowY, highX, highY);
		largeWidth = highX - lowX; 
		largeHeight = highY - lowY;

		roi.width = largeWidth / factor;
		roi.height = largeHeight / factor;
		roi.xOffset = lowX + (largeWidth-roi.width)/2; 
		roi.yOffset = lowY + (largeHeight-roi.height)/2;
		
	}

	void NaiveBuoyDetector::getBuoy(std::vector<cv::Point> &highest, int &lowX, int &lowY, int &highX, int &highY) 
	{
		for(int i = 0; i < highest.size(); i++) 
		{
			if(highest[i].x > highX) 
			{
				highX = highest[i].x;
			}
			if(highest[i].y > highY) 
			{
				highY = highest[i].y;
			} 
	 		if(highest[i].x < lowX) 
	 		{
				lowX = highest[i].x;
			}
			if(highest[i].y < lowY) 
			{
				lowY = highest[i].y;
			} 
		}
	}

	void NaiveBuoyDetector::largestContour(std::vector<std::vector<cv::Point> > &contours, std::vector<cv::Point> &highest) 
	{
		
		std::vector<cv::Point> approx;
		int lowX, lowY;
		int highX, highY;
		
		int width, height;
		float ratio = 0.00;
		
		if(contours.size() > 0) 
		{
			highest = contours[0];
		}

		// go through the contours
		for( int i = 0; i < contours.size(); i++ )
		{
			// approximate contour based on the perimeter
			cv::approxPolyDP(cv::Mat(contours[i]), approx, arcLength(cv::Mat(contours[i]), true)*0.02, true);   
			
			// if the set of points is larger 	
			if( fabs(cv::contourArea(cv::Mat(approx))) > fabs(cv::contourArea(cv::Mat(highest)))) 
			{
				lowX = lowY = 10000; 
				highX = highY = 0;
				getBuoy(approx, lowX, lowY, highX, highY);
				width = highX - lowX;
				height = highY - lowY;
				ratio = height / width;
				std::cout << ratio << std::endl;
				
				if ( (ratio > 0.5) && (abs(ratio) < 1.5) )
				{
					highest = approx;
				}

			}
		}


	}

	bool NaiveBuoyDetector::checkRatio(std::vector<cv::Point> &rectangle, float lower, float upper)
	{
		float dx, dy;
		float height, width;
		float ratio;

		dx = rectangle[3].x - rectangle[2].x;
		dy = rectangle[3].y - rectangle[2].y;
		height = sqrt(dx*dx + dy*dy);
		

		dx = rectangle[2].x - rectangle[1].x;
		dy = rectangle[2].y - rectangle[1].y;
		width = sqrt(dx*dx + dy*dy);
		
		if(height < width) 
		{
			ratio = width/height;
		}
		else
		{
		 ratio = height/width;
		}

		return (ratio >=lower) && (ratio <= upper);
	}

}
