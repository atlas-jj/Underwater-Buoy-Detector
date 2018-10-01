/**
 * @file camshift_tracker.cpp
 * @author Sean Scheideman
 * @date 4 Sept 2016
 * @brief Implementation for CamshiftTracker class
 */

#include <au_vision/camshift_tracker.h>

namespace au_vision
{

CamshiftTracker::CamshiftTracker( ros::NodeHandle &nh, ros::NodeHandle &private_nh )
	: Tracker( nh, private_nh ),
	  valueMin_(0),valueMax_(255),
	  saturationMin_(0),saturationMax_(255)
{
	trackerType_ = "camshift_tracker";
}

CamshiftTracker::~CamshiftTracker()
{

}

bool CamshiftTracker::initializeTracker( cv::Mat& frame, cv::Rect roi )
{
	processFrame( frame );
	trackedWindow_= roi;
	cv::Mat roiSelect( hsv_, roi ), maskroi( mask_, roi );

	// Calculate historgram for hue model and normalize it
	cv::calcHist( &roiSelect, 1, channels_, maskroi, histogram_, 1, histogramSize_, ranges_ );
	cv::normalize( histogram_, histogram_, 0, 255, CV_MINMAX );
}

cv::Rect CamshiftTracker::track( cv::Mat& frame )
{
	processFrame( frame );

	// Calculate backprojection to find how well pixels match historgram model
	cv::calcBackProject( &hsv_, 1, channels_, histogram_, backproj_, ranges_ );

	// Mask out pixels not in valid Hue and Saturation ranges
	backproj_ &= mask_;

	if( trackedWindow_.area() > 1 )
	{
		// Pass backproj_ to camshift algorithm to find object center and size
		cv::RotatedRect trackbox = cv::CamShift(backproj_, trackedWindow_,
		                                        cv::TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));

		return trackbox.boundingRect();
	}
	else
	{
		return trackedWindow_;
	}
}

void CamshiftTracker::processFrame( cv::Mat& frame )
{
	cv::cvtColor( frame, hsv_, cv::COLOR_BGR2HSV );

	// Create mask for pixels in valid Value and Saturation ranges
	cv::inRange( hsv_, cv::Scalar( 0, MIN( saturationMin_, saturationMax_ ), MIN( valueMin_, valueMax_ )),
	             cv::Scalar( 180, MAX( saturationMin_, saturationMax_ ), MAX( valueMin_, valueMax_ ) ), mask_ );
}

} //namespace au_vision
