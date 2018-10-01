/**
 * @file detector_node.cpp
 * @author Sean Scheideman
 * @date 30 Oct 2016
 * @brief Entry point for running detector as a node
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

#include <au_vision/detector.h>
#include <au_vision/test_detector.h>
#include <au_vision/arc_detector.h>
//#include <au_vision/surf_detector.h>


int main( int argc, char** argv ) {
    // initialize ros
    ros::init(argc, argv, "tracker");
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

    // create detector object
    au_vision::Detector* detector;
    try
    {
        if (detectorType == "test_detector") {
            detector = new au_vision::TestDetector( nh, private_nh );
        }
        else if(detectorType=="arc_detector"){
            detector = new au_vision::ArcDetector( nh, private_nh );
          }
        else
        {
            if (detectorType == "surf_detector") {
                /*std::string train_path = "";

                //Get training data for surf_detector svm
                if (private_nh.hasParam("train_path")) {
                    private_nh.getParam("train_path", train_path);
                }

                detector = new au_vision::SurfDetector( nh, private_nh, train_path );*/
                ROS_INFO_STREAM( "Tried to use disabled detector: " << detectorType.c_str() );

            }

            ROS_FATAL_STREAM( detectorType << " not found. Terminating!" );
            ros::shutdown();
        }

        ROS_INFO_STREAM( "Using Detector: " << detectorType.c_str() );
    }
    catch( std::exception& e )
    {
        ROS_FATAL_STREAM( detectorType << " initialization error: " << e.what() );
        ros::shutdown();
    }

    ros::spin();

    delete detector;
    return 0;
}
