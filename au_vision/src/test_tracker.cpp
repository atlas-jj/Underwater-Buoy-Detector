/**
 * @file tracker.cpp
 * @author Rumman Waqar
 * @date 5 Sep 2015
 * @brief Implementation for the TestTracker class
 *
 */

#include <au_vision/test_tracker.h>

namespace au_vision
{

TestTracker::TestTracker(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
	: Tracker( nh, private_nh )
{
	trackerType_ = "test_tracker";
}

TestTracker::~TestTracker()
{

}

bool TestTracker::initializeTracker(cv::Mat &frame, cv::Rect roi)
{
	trackedWindow_ = roi; // set initial ROI
	return true;
}

cv::Rect TestTracker::track(cv::Mat &frame)
{
	cv::Rect roi = trackedWindow_; // ROI never changes
	return roi;
}

} // end of namespace
