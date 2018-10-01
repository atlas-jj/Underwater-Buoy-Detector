/**
 * @file detector_server.cpp
 * @author Sean Scheideman
 * @date 14 Nov 2016
 * @brief Service for object detection
 *
 * Instantiates the detector object and starts server
 *
 * Essential ros params:
 *  * detector: type of detector (string)
 *
 */

#include "ros/ros.h"
//#include <au_vision/DetectObjects.h>
#include <au_vision/detector.h>
#include <au_vision/test_detector.h>
#include <au_vision/bin_detector.h>
#include <au_vision/naive_buoy_detector.h>
#include <au_vision/arc_detector.h>
//#include <au_vision/surf_detector.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_detection_server");
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
      else if (detectorType == "bin_detector")
      {
          detector = new au_vision::BinDetector( nh, private_nh );
      }
      else if (detectorType == "naive_buoy_detector")
      {
          detector = new au_vision::NaiveBuoyDetector( nh, private_nh );
      }
      else if (detectorType == "arc_detector")
      {
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
              ROS_INFO_STREAM( "Tried using disabled detector: " << detectorType.c_str() );
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

  ROS_INFO_STREAM( "Using Detector: " << detectorType.c_str() );

  ros::ServiceServer service = nh.advertiseService("object_detection", &au_vision::Detector::inputImageDetectRequest, detector);
  ROS_INFO("Ready to detect.");
  ros::spin();

  return 0;
}
