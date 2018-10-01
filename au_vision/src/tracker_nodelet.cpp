/**
 * @file tracker_nodelet.cpp
 * @author Rumman Waqar
 * @date 5 Sep 2015
 * @brief Entry point for running tracker as a nodelet
 *
 * Instantiates the tracker object and runs it
 * as a ROS node.
 *
 * Essential ros params:
 *  * tracker: type of tracker (string)
 *
 */

#include <string>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <au_vision/tracker.h>
#include <au_vision/test_tracker.h>
#include <au_vision/camshift_tracker.h>

namespace au_vision
{

class TrackerNodelet : public nodelet::Nodelet
{
 public:
	TrackerNodelet() {}
	~TrackerNodelet()
	{
		delete tracker_;
	}

 private:
	virtual void onInit()
	{
		// initialize ros
		ros::NodeHandle& nh = getNodeHandle();
		ros::NodeHandle& private_nh = getPrivateNodeHandle();

		// load tracker type from ros param
		std::string trackerType;
		if (private_nh.hasParam("tracker")) {
			private_nh.getParam("tracker", trackerType);
		} else {
			ROS_FATAL("No tracker specified. Terminating!");
			ros::shutdown();
		}

		// create tracker object
		try
		{
			if (trackerType == "test_tracker") {
				ROS_INFO_STREAM( "Using tracker: " << trackerType.c_str() );
				tracker_ = new au_vision::TestTracker( nh, private_nh );
			}
            else if( trackerType == "camshift_tracker")
            {
                ROS_INFO_STREAM( "Using tracker: " << trackerType.c_str() );
    			tracker_ = new au_vision::CamshiftTracker( nh, private_nh );
            }
			else
			{
				ROS_FATAL_STREAM( trackerType << " not found. Terminating!" );
				ros::shutdown();
			}
		}
		catch( std::exception& e )
		{
			ROS_FATAL_STREAM( trackerType << " initialization error: " << e.what() );
			ros::shutdown();
		}
	}

	au_vision::Tracker* tracker_; /**< pointer to tracker */
};

// package, class, namespace::class, nodelet::Nodelet
PLUGINLIB_DECLARE_CLASS( au_vision, TrackerNodelet, au_vision::TrackerNodelet, nodelet::Nodelet);

} // end of namespace
