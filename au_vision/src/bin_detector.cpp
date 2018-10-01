/**
 * @file detector.cpp
 * @author Jacky Chung
 * @date 01 Jan 2017
 * @brief Implementation for the Contour Detector class
 *
 */

#include <au_vision/bin_detector.h>
#include <opencv2/opencv.hpp>

namespace au_vision
{
	BinDetector::BinDetector(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
	  : Detector( nh, private_nh )
	{
		detectorType_ = "bin_detector";
	}

	BinDetector::~BinDetector()
	{

	}

	std::vector<au_core::Roi> BinDetector::detect(const cv::Mat &_input)
	{

		std::vector<au_core::Roi> roiArray;
		
		cv::Mat inputBGR;
		cv::Mat inputHSV;
		cv::Mat yellowMask;
		cv::Rect boundingYellow;
		cv::Point yellowMiddle;

		cvtColor(_input, inputBGR, cv::COLOR_BGR2GRAY);
		cvtColor(_input, inputHSV, cv::COLOR_RGB2HSV);
		cv::inRange(inputHSV, cv::Scalar(0, 50, 100), cv::Scalar(30, 255, 255), yellowMask);
	
		// find the middle point of the yellow lid
		std::vector<std::vector<cv::Point> > yellowContours;
		std::vector<cv::Point> yellowHighest;
		cv::findContours(yellowMask, yellowContours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point( 0, 0) );
		largestContour(yellowContours, yellowHighest);
		boundingYellow = cv::boundingRect(yellowHighest);
		yellowMiddle.x = boundingYellow.x + boundingYellow.width/2;
		yellowMiddle.y = boundingYellow.y + boundingYellow.height/2;

		// find rectangles
		filterImage(inputBGR);		
		std::vector<std::vector<cv::Point> > bins, rectangles;
		cv::findContours(inputBGR, bins, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point( 0, 0) );
		std::vector<cv::Point> approx;

		// go through the bins
		for( int i = 0; i < bins.size(); i++ )
		{
			// approximate bin based on the perimeter
			cv::approxPolyDP(cv::Mat(bins[i]), approx, arcLength(cv::Mat(bins[i]), true)*0.02, true);   

			// if the set of points make a rectangle 	
			if(approx.size() == 4 && fabs(cv::contourArea(cv::Mat(approx))) > 1000 && cv::isContourConvex(cv::Mat(approx))) 
			{

				double maxCosine = 0;

				for( int j = 2; j < 5; j++ )
					{
					double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
					maxCosine = MAX(maxCosine, cosine);
				}

				// cosine 0 = 90 degrees
				if( maxCosine < 0.3 ) 
				{
					checkRatio(approx, 0, 30);
					rectangles.push_back(approx);
				}
			}

		}

		// gets the rectangles 
		getSquares(rectangles, roiArray, yellowMiddle);
		
		return roiArray;

	}

	// checks the angle given 3 points returns cosine of the angle
	double BinDetector::angle(cv::Point &pt1, cv::Point &pt2, cv::Point &pt0) 
	{
		double dx1 = pt1.x - pt0.x;
		double dy1 = pt1.y - pt0.y;
		double dx2 = pt2.x - pt0.x;
		double dy2 = pt2.y - pt0.y;
		return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
	}

	// change return value here
	void BinDetector::getSquares(std::vector<std::vector<cv::Point> > &rectangles, std::vector<au_core::Roi> &roiArray, const cv::Point yellowPoint) 
	{
		// goes through all the rectangles, i
		for( int i = 0; i < rectangles.size(); ++i) 
		{
			cv::Rect boundingRect = cv::boundingRect(rectangles[i]);

			if( ! boundingRect.contains(yellowPoint) )
			{
				au_core::Roi roi = getParams(boundingRect);
				roiArray.push_back(roi);
			}			
			// check rectangle i with all the other rectangles in the image
			for(int j = 0; j < rectangles.size(); ++j) 
			{
				// ignore itself
				if(rectangles[i] == rectangles[j]) 
				{
					continue;
				} // rectangle i contains rectangle j
				else if(containsRect(rectangles[i], rectangles[j])) 
				{	
					
				}
				
			} // secondary loop to check inner rectangles
			
		}

	}

	// get parameters for ROI 
	au_core::Roi BinDetector::getParams(cv::Rect &rectangle) 
	{
		
		int factor = 5; 
		au_core::Roi roi;
		roi.width = rectangle.width / factor;
		roi.height = rectangle.height / factor;
			
		roi.xOffset = rectangle.x + (rectangle.width-roi.width)/2;
		roi.yOffset = rectangle.y + (rectangle.height-roi.height)/2;;

		return roi;

	}

	// filter the image for bin detection
	void BinDetector::filterImage(cv::Mat &input) {
		
		cv::Canny(input, input, 100, 200, 3);	
		cv::dilate(input, input, cv::Mat(), cv::Point(-1, -1));
	
	}

	// checks if the outer rectangle contains the inner rectangle
	bool BinDetector::containsRect(std::vector<cv::Point> &outer, std::vector<cv::Point> &inner) 
	{
		// checks the points on the rectangle clockwise, starting from the left bottom
		if((outer[3].x < inner[3].x) && (outer[3].y < inner[3].y)) 
		{
			if((outer[2].x < inner[2].x) && (outer[2].y > inner[2].y)) 
			{
				if((outer[1].x > inner[1].x) && (outer[1].y > inner[1].y)) 
				{
					if((outer[0].x > inner[0].x) && (outer[0].y < inner[0].y)) 
					{
						return true;
					}
				}
			}
		}
		// outer does not contain inner
		return false;
		
	}

	// check the ratio of the rectangles height to width
	bool BinDetector::checkRatio(std::vector<cv::Point> &rectangle, float lower, float upper)
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

	// find the largest bin
	void BinDetector::largestContour(std::vector<std::vector<cv::Point> > &bins, std::vector<cv::Point> &highest) 
	{
		
		std::vector<cv::Point> approx;
		
		if(bins.size() > 0) 
		{
			highest = bins[0];
		}

		// go through the bins
		for( int i = 0; i < bins.size(); i++ )
		{
			// approximate bin based on the perimeter
			cv::approxPolyDP(cv::Mat(bins[i]), approx, arcLength(cv::Mat(bins[i]), true)*0.02, true);   
			
			// if it is the largest bin
			if( fabs(cv::contourArea(cv::Mat(approx))) > fabs(cv::contourArea(cv::Mat(highest)))) 
			{
				highest = approx;
			}

		}

	}


}