/**
 * @file detector_action.h
 * @author Sean Scheideman
 * @date 20 Nov 2016
 * @brief Header for the DetectorAction class
 *
 * DetectorAction object initializes action server for long running object detection
 * algorithms
 *
 */

#ifndef AU_VISION_DETECTOR_ACTION_H
#define AU_VISION_DETECTOR_ACTION_H

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <au_core/Roi.h>
#include <au_core/RoiArray.h>
#include <au_vision/DetectObjectsAction.h>
#include <actionlib/server/simple_action_server.h>

#include <au_vision/detector.h>
#include <au_vision/test_detector.h>
//#include <au_vision/surf_detector.h>

namespace au_vision
{

/**
 * @class DetectorAction
 * @ingroup au_vision
 * @brief Implementation of ROS action server for  object detection
 *
 * Instantiates a specific object detection method.
 *
 */
class DetectorAction
{

 public:

    /**
	 * @brief Create a detector
	 * @param nh node handler
	 * @param private_nh node handler inside the private namespace of the node
	 * @param detectorType string detector
     *
	 * Creates Detector object and simple action server and starts it.
	 */
	DetectorAction( ros::NodeHandle& nh, ros::NodeHandle& private_nh, std::string detectorType );

	/**
	 * @brief Destroy DetectorAction
	 */
	virtual ~DetectorAction();

    /**
     * @brief Callback function for when simple action server recieves goal
     *
     * Calls object detection method and sends result
     */
    void goalCB( void );

    /**
     * @brief Callback function for when simple action server is preempted
     */
    void preemptCB( void );

protected:

	std::vector<au_core::Roi> roiArray_;    /**< stores current ROI's */

    //action server
    actionlib::SimpleActionServer<au_vision::DetectObjectsAction> actionServer_;
    std::string actionName_;
    sensor_msgs::Image goal_;
    au_vision::DetectObjectsFeedback feedback_;
    au_vision::DetectObjectsResult result_;


private:
    au_vision::Detector* detector_;

}; // end of detector class

} // end of namespace

#endif //AU_VISION_DETECTOR_H
