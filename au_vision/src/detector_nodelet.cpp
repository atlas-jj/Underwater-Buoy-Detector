/**
 * @file detector_nodelet.cpp
 * @author Sean Scheideman
 * @date 30 Oct 2016
 * @brief Entry point for running detector as a nodelet
 *
 * Instantiates the detector object and runs it
 * as a ROS node.
 *
 * Essential ros params:
 *  * detector: type of detector (string)
 *
 */

#include <string>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <au_vision/detector.h>
#include <au_vision/test_detector.h>

namespace au_vision
{

class DetectorNodelet : public nodelet::Nodelet
{
 public:
	DetectorNodelet() {}
	~DetectorNodelet()
	{
		delete detector_;
	}

 private:
	virtual void onInit()
	{
		// initialize ros
		ros::NodeHandle& nh = getNodeHandle();
		ros::NodeHandle& private_nh = getPrivateNodeHandle();

		// load detector type from ros param
		std::string detectorType;
		if (private_nh.hasParam("detector")) {
			private_nh.getParam("detector", detectorType);
		} else {
			ROS_FATAL("No detector specified. Terminating!");
			ros::shutdown();
		}

		// create detector object
		try
		{
			if (detectorType == "test_detector") {
				ROS_INFO_STREAM( "Using tracker: " << detectorType.c_str() );
				detector_ = new au_vision::TestDetector( nh, private_nh );
			}
			else
			{
				ROS_FATAL_STREAM( detectorType << " not found. Terminating!" );
				ros::shutdown();
			}
		}
		catch( std::exception& e )
		{
			ROS_FATAL_STREAM( detectorType << " initialization error: " << e.what() );
			ros::shutdown();
		}
	}

	au_vision::Detector* detector_; /**< pointer to detector */
};

// package, class, namespace::class, nodelet::Nodelet
PLUGINLIB_DECLARE_CLASS( au_vision, DetectorNodelet, au_vision::DetectorNodelet, nodelet::Nodelet);

} // end of namespace
