/**
 * @file detector_action.cpp
 * @author Sean Scheideman
 * @date 20 Nov 2016
 * @brief Implementation for the DetectorAction class
 *
 */

#include <au_vision/detector_action.h>

namespace au_vision
{
    DetectorAction::DetectorAction(ros::NodeHandle& nh, ros::NodeHandle& private_nh, std::string detectorType)
        : actionServer_(nh, "object_detection_server", false)
    {
        try
        {
            if (detectorType == "test_detector") {
                detector_ = new au_vision::TestDetector( nh, private_nh );
            } 
             // add more detectors here
            else
            {
                if (detectorType == "surf_detector") {
                    /*std::string train_path = "";

                    //Get training data for surf_detector svm
                    if (private_nh.hasParam("train_path")) {
                        private_nh.getParam("train_path", train_path);
                    }

                    detector_ = new au_vision::SurfDetector( nh, private_nh, train_path );*/
                    ROS_INFO_STREAM( "Tried using disabled detector: surf_detector" );

                }
                ROS_FATAL_STREAM( detectorType << " not found. Terminating!" );
                ros::shutdown();
            }
        }
        catch( std::exception& e )
        {
            ROS_FATAL_STREAM( detectorType << " initialization error: " << e.what() );
            ros::shutdown();
        }

        actionServer_.registerGoalCallback( boost::bind(&DetectorAction::goalCB, this) );
        actionServer_.registerPreemptCallback( boost::bind(&DetectorAction::preemptCB, this) );

        actionServer_.start();
        ROS_INFO_STREAM( "Waiting for goals" );
    }

    DetectorAction::~DetectorAction()
    {
        delete(detector_);
    }


    void DetectorAction::goalCB()
    {
        ROS_INFO_STREAM( "New goal" );
        // accept the new goal
        goal_ = actionServer_.acceptNewGoal()->inputFrame;
        result_.roiArray.regionsOfInterest = detector_->inputImageDetectAction(goal_);;
        actionServer_.setSucceeded(result_);
    }

    void DetectorAction::preemptCB()
    {
        ROS_INFO("Detector action server preempted");
        // set the action state to preempted
        actionServer_.setPreempted();
    }
} // end of namespace
