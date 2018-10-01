/**
 * @file detector_action_server.cpp
 * @author Sean Scheideman
 * @date 21 Nov 2016
 * @brief Creates action server using the DetectorAction class
 *
 * ROS node which instantiates the DetectorAction object and starts action server
 *
 * Essential ros params:
 *  * detector: type of detector (string)
 *
 */

#include <ros/ros.h>
#include <au_vision/DetectObjectsAction.h>
#include <actionlib/server/simple_action_server.h>
#include <au_vision/detector.h>
#include <au_vision/test_detector.h>
#include <au_vision/detector_action.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_detection_action_server");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // load detector type from ros param
    std::string detectorType;
    if (private_nh.hasParam("detector")) {
        private_nh.getParam("detector", detectorType);
    } else {
        ROS_FATAL("No detector specified. Terminating!");
        ros::shutdown();
    }

    au_vision::DetectorAction detectorAction( nh, private_nh, detectorType );

    while(ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}
