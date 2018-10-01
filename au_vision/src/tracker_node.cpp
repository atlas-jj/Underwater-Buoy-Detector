/**
 * @file tracker_node.cpp
 * @author Rumman Waqar
 * @date 2 Sep 2015
 * @brief Entry point for running tracker as a node
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

#include <au_vision/tracker.h>
#include <au_vision/test_tracker.h>
#include <au_vision/camshift_tracker.h>

int main( int argc, char** argv ) {
    // initialize ros
    ros::init(argc, argv, "tracker");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // load tracker type from ros param
    std::string trackerType;
    if (private_nh.hasParam("tracker")) {
        private_nh.getParam("tracker", trackerType);
    } else {
        ROS_FATAL("No tracker specified. Terminating!");
        ros::shutdown();
    }

    // create tracker object
    au_vision::Tracker* tracker;
    try
    {
        if (trackerType == "test_tracker") {
	        ROS_INFO_STREAM( "Using tracker: " << trackerType.c_str() );
			tracker = new au_vision::TestTracker( nh, private_nh );
        }
        else if( trackerType == "camshift_tracker")
        {
            ROS_INFO_STREAM( "Using tracker: " << trackerType.c_str() );
			tracker = new au_vision::CamshiftTracker( nh, private_nh );
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

    ros::spin();

    delete tracker;
    return 0;
}
