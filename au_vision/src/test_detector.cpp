/**
 * @file test_detector.cpp
 * @author Sean Scheideman
 * @date 30 Oct 2016
 * @brief Implementation for the TestDetector class
 *
 */

#include <au_vision/test_detector.h>

namespace au_vision
{

TestDetector::TestDetector(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
	: Detector( nh, private_nh )
{
	detectorType_ = "test_detector";
}

TestDetector::~TestDetector()
{

}

std::vector<au_core::Roi> TestDetector::detect( const cv::Mat &frame )
{
    std::vector<au_core::Roi> roiArray;
    int xMax = frame.cols;
    int yMax = frame.rows;

    for(int i = 0 ; i < 3 ; i++)
    {
        int x = rand() % xMax;
        int y = rand() % yMax;
        
        au_core::Roi roi;
        roi.xOffset = x;
        roi.yOffset = y;
        roi.width = 25;
        roi.height = 25;
        roi.type = "Buoy";

        roiArray.push_back(roi);
    }
    return roiArray;
}

} // end of namespace
